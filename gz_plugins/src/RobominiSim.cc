/*
 * RobominiSim.cc — implementation of the Robomini world system plugin.
 *
 * Implements the Gazebo-side interface for the ROBOMINI 机甲杯
 * auto-aim simulation:
 *   A. energy rune: hub rotation modes, target lights, hit sweep detection,
 *      20 Hz state.
 *   B. launchers: 2-DOF gimbal velocity control, ball firing, ball recycling,
 *      20 Hz state + fire events.
 *
 * Gazebo-interaction notes (verified against gz-sim 8.15 / gz-physics 7):
 *  - components::JointVelocityCmd is consumed by the physics system during the
 *    Update phase and its data is zeroed at the end of each step, therefore it
 *    must be (re)written every PreUpdate for continuous speed control.
 *  - components::WorldPoseCmd (models) and components::LinearVelocityCmd
 *    (models: velocity is rotated by the model's Pose rotation, links: local
 *    frame) are one-shot commands; physics applies them during Update and
 *    LinearVelocityCmd is zeroed afterwards. The plugin writes them only when
 *    a ball is fired / recycled.
 *  - components::VisualCmd (msgs::Visual) on a visual entity is picked up by
 *    RenderUtil (SceneBroadcaster/Sensors) and applied to the render material
 *    (ambient/diffuse/specular/emissive). This is exactly the path used by the
 *    /world/<world>/material_color service, so the plugin takes the direct ECM
 *    route (no service round-trip, same-step effect).
 *
 * License: Apache-2.0.
 */
#include "RobominiSim.hh"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/color.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <sdf/Box.hh>
#include <sdf/Geometry.hh>
#include <sdf/JointAxis.hh>

#include "json_lite.hh"

namespace robomini
{
namespace sim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::size_t kTeamCount = 2;
constexpr std::size_t kTargetCount = 5;
constexpr std::size_t kBarCount = 4;

/// \brief Team index 0 = blue (-Y side), 1 = red (+Y side).
enum class Team : std::size_t
{
  kBlue = 0,
  kRed = 1
};

/// \brief The 4 light-bar visual names below each tgt_<team>_<i> link.
/// Order: 0,1 = radial (inner, outer), 2,3 = tangential.
constexpr std::array<const char *, kBarCount> kBarNames = {{
    "bar_radial_inner", "bar_radial_outer",
    "bar_tangential_1", "bar_tangential_2"}};

/// \brief Target polar angles (X-Z plane, atan2(dz, dx) in deg)
/// at hub_angle = 0, sorted ascending. Index == tgt_<team>_<i>.
constexpr std::array<double, kTargetCount> kBlueTargetAnglesDeg = {
    -121.504, -49.504, 22.496, 94.496, 166.496};
constexpr std::array<double, kTargetCount> kRedTargetAnglesDeg = {
    -157.504, -85.504, -13.504, 58.496, 130.496};

/// \brief Light states used by the referee state machine (rule 5.5.4).
enum class LightState
{
  kOff = 0,
  kPending = 1,
  kActive = 2
};

/// \brief Ball lifecycle state tracked by the plugin.
enum class BallState
{
  kParked = 0,
  kFlying = 1
};

/// \brief Parsed lights payload (transport thread -> sim thread).
struct LightsCommand
{
  Team team = Team::kBlue;
  std::array<LightState, kTargetCount> states{{
      LightState::kOff, LightState::kOff, LightState::kOff,
      LightState::kOff, LightState::kOff}};
};

/// \brief Parsed mode payload (transport thread -> sim thread).
struct ModeCommand
{
  Team team = Team::kBlue;
  std::string mode = "fixed";
  double a = 0.9;
  double omega = 1.95;
  double t0 = 0.0;
  bool hasT0 = false;
};

/// \brief Parsed gimbal payload (角度 + 可选速度/加速度前馈).
struct GimbalCommand
{
  Team team = Team::kBlue;
  double yaw = 0.0;
  double pitch = 0.0;
  /// \brief 可选速度前馈 rad/s（缺失/非有限值 = 0，等价纯角度指令）
  double yawVel = 0.0;
  double pitchVel = 0.0;
  /// \brief 可选加速度前馈 rad/s²（缺失/非有限值 = 0）
  double yawAcc = 0.0;
  double pitchAcc = 0.0;
};

/// \brief Parsed fire payload.
struct FireCommand
{
  Team team = Team::kBlue;
  double speed = 12.0;
};

/// \brief Format a double for JSON, guarding against nan/inf.
std::string Num(double _v)
{
  if (!std::isfinite(_v))
    _v = 0.0;
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(6);
  ss << _v;
  std::string out = ss.str();
  // Avoid "-0.000000" noise in state topics / logs.
  if (out == "-0.000000")
    out = "0.000000";
  return out;
}

std::string TeamName(Team _team)
{
  return _team == Team::kBlue ? "blue" : "red";
}

/// \brief Outward plate normal (world, hub_angle independent): blue faces -Y,
/// red faces +Y (靶面法向沿 Y 轴指向本方发射架).
double OutwardNormalY(Team _team)
{
  return _team == Team::kBlue ? -1.0 : 1.0;
}

double TargetAngleRad(Team _team, std::size_t _index)
{
  const auto &angles = (_team == Team::kBlue)
      ? kBlueTargetAnglesDeg : kRedTargetAnglesDeg;
  return angles.at(_index) * kDegToRad;
}

LightState ParseLightState(const std::string &_s, LightState _default)
{
  if (_s == "off")
    return LightState::kOff;
  if (_s == "pending")
    return LightState::kPending;
  if (_s == "active")
    return LightState::kActive;
  return _default;
}

const char *LightStateName(LightState _state)
{
  switch (_state)
  {
    case LightState::kOff: return "off";
    case LightState::kPending: return "pending";
    case LightState::kActive: return "active";
  }
  return "off";
}

/// \brief Fill a gz.msgs.Color from an RGB triple + alpha.
void SetColor(gz::msgs::Color *_color, double _r, double _g, double _b,
    double _a = 1.0)
{
  _color->set_r(_r);
  _color->set_g(_g);
  _color->set_b(_b);
  _color->set_a(_a);
}

/// \brief Clamp with optional NaN guard.
double Clamp(double _v, double _lo, double _hi)
{
  if (!std::isfinite(_v))
    return _lo;
  return std::max(_lo, std::min(_hi, _v));
}

/// \brief 有限值兜底：非有限值（inf/nan）时返回 _fallback。
/// 非有限的 gimbal 前馈字段被忽略，效果等同缺省 0。
double FiniteOr(double _v, double _fallback)
{
  return std::isfinite(_v) ? _v : _fallback;
}

}  // namespace

////////////////////////////////////////////////////////////////////////////////
// Impl
////////////////////////////////////////////////////////////////////////////////
class RobominiSim::Impl
{
  public: struct RuneTarget
  {
    gz::sim::Entity link = gz::sim::kNullEntity;
    std::array<gz::sim::Entity, kBarCount> bars{{
        gz::sim::kNullEntity, gz::sim::kNullEntity,
        gz::sim::kNullEntity, gz::sim::kNullEntity}};
  };

  public: struct RuneRuntime
  {
    Team team = Team::kBlue;
    gz::sim::Entity model = gz::sim::kNullEntity;
    gz::sim::Entity hubLink = gz::sim::kNullEntity;
    gz::sim::Entity hubJoint = gz::sim::kNullEntity;
    std::array<RuneTarget, kTargetCount> targets;
    bool found = false;
    bool visualsReady = false;

    // --- commands (written by transport callbacks) ---
    std::string mode = "fixed";
    double bigA = 0.9;
    double bigOmega = 1.95;
    double bigT0 = 0.0;
    bool bigWaitT0 = false;
    std::array<LightState, kTargetCount> lights{{
        LightState::kOff, LightState::kOff, LightState::kOff,
        LightState::kOff, LightState::kOff}};
    bool lightsDirty = false;

    // --- feedback ---
    double hubAngle = 0.0;
    double hubSpeed = 0.0;
    std::array<LightState, kTargetCount> appliedLights{{
        LightState::kOff, LightState::kOff, LightState::kOff,
        LightState::kOff, LightState::kOff}};
    bool geomLogged = false;
  };

  public: struct LauncherRuntime
  {
    Team team = Team::kBlue;
    gz::sim::Entity model = gz::sim::kNullEntity;
    gz::sim::Entity yawLink = gz::sim::kNullEntity;
    gz::sim::Entity pitchLink = gz::sim::kNullEntity;
    gz::sim::Entity muzzleLink = gz::sim::kNullEntity;
    gz::sim::Entity yawJoint = gz::sim::kNullEntity;
    gz::sim::Entity pitchJoint = gz::sim::kNullEntity;
    bool found = false;

    double yawTarget = 0.0;
    double pitchTarget = 0.0;

    // 前馈目标（rad/s、rad/s²；0 = 纯角度指令 = 原有行为）
    double yawTargetVel = 0.0;
    double pitchTargetVel = 0.0;
    double yawTargetAcc = 0.0;
    double pitchTargetAcc = 0.0;

    // 伺服内部状态：上一步下发的速度命令（供 a_max·dt 限幅）与实测角速度
    // （关节位置差分 + 一阶低通，见 UpdateLaunchers / 前馈接口）。
    double yawCmdVel = 0.0;
    double pitchCmdVel = 0.0;
    double yawVel = 0.0;
    double pitchVel = 0.0;
    bool havePrevJoints = false;

    double yaw = 0.0;
    double pitch = 0.0;
    bool ready = false;
    std::uint64_t ballsFired = 0;
    double lastFireTime = -1.0e9;
    gz::math::Vector3d muzzlePos = gz::math::Vector3d::Zero;
    gz::math::Vector3d muzzleDir = gz::math::Vector3d::UnitY;
    bool haveMuzzle = false;
  };

  public: struct BallRuntime
  {
    std::string name;
    gz::sim::Entity model = gz::sim::kNullEntity;
    /// \brief the (single) link of the ball model, used for velocity feedback
    gz::sim::Entity link = gz::sim::kNullEntity;
    BallState state = BallState::kParked;
    gz::math::Pose3d home = gz::math::Pose3d::Zero;
    gz::math::Vector3d prevPos = gz::math::Vector3d::Zero;
    bool hasPrev = false;
    double launchTime = 0.0;
    double launchSpeed = 0.0;
    Team team = Team::kBlue;
    bool hitReported = false;
    double lowZStart = -1.0;
    double nextTraceTime = 0.0;
    /// \brief True between the fire command and the step where physics has
    /// actually applied the WorldPoseCmd teleport. Prevents the ball from being
    /// recycled/analysed while it is still sitting in the ball park.
    bool pendingTeleport = false;
    /// \brief PreUpdate step at which the one-shot WorldPoseCmd /
    /// LinearVelocityCmd components must be removed again (see comment in
    /// UpdateBalls). -1 = nothing to clean up.
    std::int64_t cleanupStep = -1;
    /// \brief World position the ball was spawned at when fired; used to
    /// detect when physics has actually applied the teleport.
    gz::math::Vector3d spawnPos = gz::math::Vector3d::Zero;
    /// \brief stepCount when the ball was fired (teleport watchdog).
    std::int64_t launchStep = -1;
  };

  public: struct FireResult
  {
    bool ok = false;
    std::string ball;
    double speed = 0.0;
    gz::math::Vector3d muzzle = gz::math::Vector3d::Zero;
  };

  // ------------------------------------------------------------------------
  // Configuration (SDF parameters, all optional)
  // ------------------------------------------------------------------------
  public: std::string worldName{"robomini"};
  /// \brief Optional prefix for every contract topic, e.g. "/test" ->
  /// /test/rune/blue/mode. Empty by default = strict contract topics.
  public: std::string topicPrefix;
  public: std::string runeModelName{"energy_rune"};
  public: double publishRate = 20.0;
  public: double targetRadius = 0.35;
  public: double plateDiameter = 0.195;
  public: double plateThickness = 0.022;
  public: double ballRadius = 0.02;
  public: int ballCount = 24;
  public: std::string ballParkName{"ball_park"};
  public: double hitPlaneSlack = 0.01;

  public: double launchTargetSpeedDefault = 12.0;
  public: double muzzleOffset = 0.025;
  public: double muzzleSpreadDeg = 0.0;
  public: double fireRate = 3.0;
  public: double yawMaxVel = 180.0 * kDegToRad;
  public: double pitchMaxVel = 120.0 * kDegToRad;
  public: double gimbalGain = 12.0;
  public: double gimbalTolerance = 0.2 * kDegToRad;
  /// \brief 云台速度命令的加速度限幅默认值 rad/s²（SDF: max_yaw_accel /
  /// max_pitch_accel）；实际限幅取 max(|a_ff|, 本值)。
  public: double yawMaxAccel = 300.0;
  public: double pitchMaxAccel = 300.0;
  public: double yawLimit = 90.0 * kDegToRad;
  public: double pitchMin = -10.0 * kDegToRad;
  public: double pitchMax = 45.0 * kDegToRad;
  public: gz::math::Vector3d barrelAxisLocal{0.0, 1.0, 0.0};

  public: bool ballTrace = false;
  public: bool stateStamp = false;
  /// \brief One-shot GEOMDBG diagnostics for asset integration; off by default.
  public: bool geometryDebug = false;
  public: double parkResetInterval = 5.0;
  public: double ballFlightTimeout = 6.0;
  public: double ballRestTimeout = 2.0;
  public: double fieldHalfX = 3.2;
  public: double fieldHalfY = 3.2;
  public: double fieldMinZ = -2.0;

  public: gz::math::Vector3d runeAxisBlue{0.0, -0.116, 0.928};
  public: gz::math::Vector3d runeAxisRed{0.0, 0.116, 0.928};

  // Runtime state -----------------------------------------------------------
  public: std::array<RuneRuntime, kTeamCount> runes;
  public: std::array<LauncherRuntime, kTeamCount> launchers;
  public: std::vector<BallRuntime> balls;

  public: bool runeVisualsWarned = false;
  public: bool rateWarned[kTeamCount] = {false, false};
  public: bool noBallWarned[kTeamCount] = {false, false};
  public: bool notFoundWarned = false;
  public: bool discovered = false;
  public: double lastDiscoverAttempt = -1.0;
  public: double lastStatePublish = -1.0e9;
  public: double lastParkReset = -1.0e9;
  public: double simTime = 0.0;
  public: double stepDt = 0.001;
  public: std::int64_t stepCount = 0;

  // Transport ---------------------------------------------------------------
  public: std::unique_ptr<gz::transport::Node> node;
  public: std::array<gz::transport::Node::Publisher, kTeamCount> runeStatePub;
  public: std::array<gz::transport::Node::Publisher, kTeamCount> launcherStatePub;
  public: std::array<gz::transport::Node::Publisher, kTeamCount> hitPub;

  // Commands coming from transport threads, guarded by mutex_.
  public: std::mutex mutex;
  public: std::vector<ModeCommand> pendingModes;
  public: std::vector<LightsCommand> pendingLights;
  public: std::vector<GimbalCommand> pendingGimbals;
  public: std::vector<FireCommand> pendingFires;
  public: std::mt19937 rng{std::random_device{}()};

  // ------------------------------------------------------------------------
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm)
  {
    this->runes[0].team = Team::kBlue;
    this->runes[1].team = Team::kRed;
    this->launchers[0].team = Team::kBlue;
    this->launchers[1].team = Team::kRed;

    // World name: prefer the ECM name of the entity this plugin is attached to,
    // fall back to the SDF / default. The world name is only used for logging.
    auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    if (nameComp != nullptr && !nameComp->Data().empty())
      this->worldName = nameComp->Data();

    if (_sdf != nullptr)
    {
      this->ReadSdf(_sdf);
      if (_sdf->HasElement("world_name"))
        this->worldName = _sdf->Get<std::string>("world_name");
    }

    this->node = std::make_unique<gz::transport::Node>();
    if (!this->topicPrefix.empty())
    {
      gzmsg << "[RobominiSim] NOTE: contract topics are prefixed with '"
            << this->topicPrefix << "' (test/isolated world mode)"
            << std::endl;
    }

    for (std::size_t t = 0; t < kTeamCount; ++t)
    {
      const Team team = static_cast<Team>(t);
      const std::string teamName = TeamName(team);

      this->node->Subscribe<gz::msgs::StringMsg>(
          this->TopicPath("/rune/" + teamName + "/mode"),
          [this, team](const gz::msgs::StringMsg &_msg)
          { this->OnRuneMode(team, _msg); });
      this->node->Subscribe<gz::msgs::StringMsg>(
          this->TopicPath("/rune/" + teamName + "/lights"),
          [this, team](const gz::msgs::StringMsg &_msg)
          { this->OnRuneLights(team, _msg); });
      this->node->Subscribe<gz::msgs::StringMsg>(
          this->TopicPath("/launcher/" + teamName + "/gimbal"),
          [this, team](const gz::msgs::StringMsg &_msg)
          { this->OnGimbal(team, _msg); });
      this->node->Subscribe<gz::msgs::StringMsg>(
          this->TopicPath("/launcher/" + teamName + "/fire"),
          [this, team](const gz::msgs::StringMsg &_msg)
          { this->OnFire(team, _msg); });

      this->runeStatePub[t] = this->node->Advertise<gz::msgs::StringMsg>(
          this->TopicPath("/rune/" + teamName + "/state"));
      this->launcherStatePub[t] = this->node->Advertise<gz::msgs::StringMsg>(
          this->TopicPath("/launcher/" + teamName + "/state"));
      this->hitPub[t] = this->node->Advertise<gz::msgs::StringMsg>(
          this->TopicPath("/rune/" + teamName + "/hit"));
    }

    gzmsg << "[RobominiSim] Configured world '" << this->worldName
          << "' (publish_rate=" << this->publishRate
          << " Hz, balls=" << this->ballCount
          << ", fire_rate=" << this->fireRate << " 1/s, gimbal accel limits="
          << this->yawMaxAccel << "/" << this->pitchMaxAccel
          << " rad/s^2)" << std::endl;

    this->Discover(_ecm);
  }

  /// \brief Build a contract topic name, applying the optional test prefix.
  public: std::string TopicPath(const std::string &_path) const
  {
    return this->topicPrefix + _path;
  }

  // ------------------------------------------------------------------------
  public: void ReadSdf(const std::shared_ptr<const sdf::Element> &_sdf)
  {
    auto getD = [&](const std::string &_key, double &_v)
    {
      if (_sdf->HasElement(_key))
        _v = _sdf->Get<double>(_key);
    };
    auto getI = [&](const std::string &_key, int &_v)
    {
      if (_sdf->HasElement(_key))
        _v = _sdf->Get<int>(_key);
    };
    auto getS = [&](const std::string &_key, std::string &_v)
    {
      if (_sdf->HasElement(_key))
        _v = _sdf->Get<std::string>(_key);
    };
    auto getV3 = [&](const std::string &_key, gz::math::Vector3d &_v)
    {
      if (_sdf->HasElement(_key))
        _v = _sdf->Get<gz::math::Vector3d>(_key);
    };

    getS("topic_prefix", this->topicPrefix);
    if (!this->topicPrefix.empty())
    {
      if (this->topicPrefix.front() != '/')
        this->topicPrefix = "/" + this->topicPrefix;
      while (!this->topicPrefix.empty() &&
             this->topicPrefix.back() == '/')
      {
        this->topicPrefix.pop_back();
      }
    }
    getS("rune_model", this->runeModelName);
    getS("ball_park_model", this->ballParkName);
    getD("publish_rate", this->publishRate);
    getD("target_radius", this->targetRadius);
    getD("plate_diameter", this->plateDiameter);
    getD("plate_thickness", this->plateThickness);
    getD("ball_radius", this->ballRadius);
    getI("ball_count", this->ballCount);
    getD("hit_plane_slack", this->hitPlaneSlack);

    getD("default_fire_speed", this->launchTargetSpeedDefault);
    getD("muzzle_offset", this->muzzleOffset);
    getD("muzzle_spread_deg", this->muzzleSpreadDeg);
    getD("fire_rate", this->fireRate);
    getD("yaw_max_vel", this->yawMaxVel);
    getD("pitch_max_vel", this->pitchMaxVel);
    getD("gimbal_gain", this->gimbalGain);
    getD("gimbal_tolerance", this->gimbalTolerance);
    // 前馈伺服：速度命令的加速度限幅默认值（缺省 300）
    getD("max_yaw_accel", this->yawMaxAccel);
    getD("max_pitch_accel", this->pitchMaxAccel);
    {
      double yawLimitDeg = this->yawLimit / kDegToRad;
      double pitchMinDeg = this->pitchMin / kDegToRad;
      double pitchMaxDeg = this->pitchMax / kDegToRad;
      getD("yaw_limit_deg", yawLimitDeg);
      getD("pitch_min_deg", pitchMinDeg);
      getD("pitch_max_deg", pitchMaxDeg);
      this->yawLimit = yawLimitDeg * kDegToRad;
      this->pitchMin = pitchMinDeg * kDegToRad;
      this->pitchMax = pitchMaxDeg * kDegToRad;
    }
    getV3("barrel_axis_local", this->barrelAxisLocal);
    if (this->barrelAxisLocal.Length() < 1e-6)
      this->barrelAxisLocal = gz::math::Vector3d::UnitY;
    this->barrelAxisLocal.Normalize();

    if (_sdf->HasElement("ball_trace"))
      this->ballTrace = _sdf->Get<bool>("ball_trace");
    if (_sdf->HasElement("state_include_stamp"))
      this->stateStamp = _sdf->Get<bool>("state_include_stamp");
    if (_sdf->HasElement("geometry_debug"))
      this->geometryDebug = _sdf->Get<bool>("geometry_debug");
    getD("park_reset_interval", this->parkResetInterval);
    getD("ball_flight_timeout", this->ballFlightTimeout);
    getD("ball_rest_timeout", this->ballRestTimeout);
    getD("field_half_x", this->fieldHalfX);
    getD("field_half_y", this->fieldHalfY);
    getD("field_min_z", this->fieldMinZ);

    getV3("rune_axis_blue", this->runeAxisBlue);
    getV3("rune_axis_red", this->runeAxisRed);

    this->publishRate = std::max(1.0, this->publishRate);
    this->fireRate = std::max(0.1, this->fireRate);
    this->ballCount = std::max(0, this->ballCount);
    // 加速度限幅必须为正，否则速度命令无法变化（云台冻结）；给一个保守下限。
    this->yawMaxAccel = std::max(1.0, this->yawMaxAccel);
    this->pitchMaxAccel = std::max(1.0, this->pitchMaxAccel);
  }

  // ------------------------------------------------------------------------
  // Transport callbacks (transport thread!)
  // ------------------------------------------------------------------------
  public: void OnRuneMode(Team _team, const gz::msgs::StringMsg &_msg)
  {
    robomini::json::Value root;
    std::string err;
    if (!robomini::json::Parse(_msg.data(), root, err))
    {
      gzwarn << "[RobominiSim] /rune/" << TeamName(_team)
             << "/mode: ignoring malformed JSON (" << err << "): "
             << _msg.data() << std::endl;
      return;
    }
    ModeCommand cmd;
    cmd.team = _team;
    cmd.mode = root.StringOr("mode", "");
    cmd.a = root.NumberOrCoerce("a", 0.9);
    cmd.omega = root.NumberOrCoerce("omega", 1.95);
    const auto *t0Val = root.Find("t0");
    if (t0Val != nullptr && t0Val->IsNumber())
    {
      cmd.t0 = t0Val->number;
      cmd.hasT0 = true;
    }
    if (cmd.mode != "fixed" && cmd.mode != "random_fixed" &&
        cmd.mode != "small" && cmd.mode != "big")
    {
      gzwarn << "[RobominiSim] /rune/" << TeamName(_team)
             << "/mode: unknown mode '" << cmd.mode
             << "' (expected fixed|random_fixed|small|big), ignoring"
             << std::endl;
      return;
    }
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingModes.push_back(cmd);
  }

  public: void OnRuneLights(Team _team, const gz::msgs::StringMsg &_msg)
  {
    robomini::json::Value root;
    std::string err;
    if (!robomini::json::Parse(_msg.data(), root, err))
    {
      gzwarn << "[RobominiSim] /rune/" << TeamName(_team)
             << "/lights: ignoring malformed JSON (" << err << "): "
             << _msg.data() << std::endl;
      return;
    }
    const auto *targets = root.Find("targets");
    if (targets == nullptr || !targets->IsArray())
    {
      gzwarn << "[RobominiSim] /rune/" << TeamName(_team)
             << "/lights: missing 'targets' array, ignoring: "
             << _msg.data() << std::endl;
      return;
    }
    LightsCommand cmd;
    cmd.team = _team;
    std::size_t n = std::min(targets->array.size(), kTargetCount);
    for (std::size_t i = 0; i < n; ++i)
    {
      if (!targets->array[i].IsString())
      {
        gzwarn << "[RobominiSim] /rune/" << TeamName(_team)
               << "/lights: targets[" << i
               << "] is not a string, using 'off'" << std::endl;
        continue;
      }
      cmd.states[i] = ParseLightState(targets->array[i].string,
          LightState::kOff);
    }
    if (n < kTargetCount)
    {
      gzwarn << "[RobominiSim] /rune/" << TeamName(_team)
             << "/lights: expected " << kTargetCount << " targets, got "
             << n << " (missing entries default to 'off')" << std::endl;
    }
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingLights.push_back(cmd);
  }

  public: void OnGimbal(Team _team, const gz::msgs::StringMsg &_msg)
  {
    robomini::json::Value root;
    std::string err;
    if (!robomini::json::Parse(_msg.data(), root, err))
    {
      gzwarn << "[RobominiSim] /launcher/" << TeamName(_team)
             << "/gimbal: ignoring malformed JSON (" << err << "): "
             << _msg.data() << std::endl;
      return;
    }
    GimbalCommand cmd;
    cmd.team = _team;
    cmd.yaw = root.NumberOrCoerce("yaw", 0.0);
    cmd.pitch = root.NumberOrCoerce("pitch", 0.0);
    // 可选前馈字段：缺失=0（纯角度指令）；
    // 非有限值（如 1e999 解析成 inf）忽略，同样按 0 处理。
    cmd.yawVel = FiniteOr(root.NumberOrCoerce("yaw_vel", 0.0), 0.0);
    cmd.pitchVel = FiniteOr(root.NumberOrCoerce("pitch_vel", 0.0), 0.0);
    cmd.yawAcc = FiniteOr(root.NumberOrCoerce("yaw_acc", 0.0), 0.0);
    cmd.pitchAcc = FiniteOr(root.NumberOrCoerce("pitch_acc", 0.0), 0.0);
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->pendingGimbals.push_back(cmd);
    }
  }

  public: void OnFire(Team _team, const gz::msgs::StringMsg &_msg)
  {
    robomini::json::Value root;
    std::string err;
    FireCommand cmd;
    cmd.team = _team;
    cmd.speed = this->launchTargetSpeedDefault;
    if (!_msg.data().empty())
    {
      if (!robomini::json::Parse(_msg.data(), root, err))
      {
        gzwarn << "[RobominiSim] /launcher/" << TeamName(_team)
               << "/fire: ignoring malformed JSON (" << err << "): "
               << _msg.data() << std::endl;
        return;
      }
      cmd.speed = Clamp(root.NumberOrCoerce("speed",
          this->launchTargetSpeedDefault), 0.5, 100.0);
    }
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingFires.push_back(cmd);
  }

  // ------------------------------------------------------------------------
  // Main loop
  // ------------------------------------------------------------------------
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
  {
    ++this->stepCount;
    this->simTime = std::chrono::duration<double>(_info.simTime).count();
    this->stepDt = std::chrono::duration<double>(_info.dt).count();
    if (!std::isfinite(this->stepDt) || this->stepDt <= 0.0)
      this->stepDt = 0.001;

    if (!this->discovered &&
        (this->lastDiscoverAttempt < 0.0 ||
         this->simTime - this->lastDiscoverAttempt > 0.5))
    {
      this->lastDiscoverAttempt = this->simTime;
      this->Discover(_ecm);
    }

    this->ConsumeCommands(_ecm);
    this->UpdateRunes(_ecm);
    this->UpdateLaunchers(_ecm);
    this->UpdateBalls(_ecm);
    this->MaybePublishState(_ecm);
  }

  // ------------------------------------------------------------------------
  // Discovery of the contract entities. Called until everything is found, so
  // the plugin also survives being loaded before the models are spawned.
  // ------------------------------------------------------------------------
  public: void Discover(gz::sim::EntityComponentManager &_ecm)
  {
    using namespace gz::sim;  // NOLINT

    // --- energy rune -------------------------------------------------------
    auto runeModel = _ecm.EntityByComponents(components::Model(),
        components::Name(this->runeModelName));
    for (std::size_t t = 0; t < kTeamCount; ++t)
    {
      auto &rune = this->runes[t];
      if (rune.found || runeModel == kNullEntity)
        continue;
      rune.model = runeModel;
      rune.hubLink = this->FindChildLink(_ecm, runeModel,
          "hub_" + TeamName(rune.team));
      rune.hubJoint = this->FindChildJoint(_ecm, runeModel,
          "hub_" + TeamName(rune.team) + "_joint");
      bool ok = rune.hubLink != kNullEntity && rune.hubJoint != kNullEntity;
      for (std::size_t i = 0; i < kTargetCount; ++i)
      {
        const std::string linkName =
            "tgt_" + TeamName(rune.team) + "_" + std::to_string(i);
        rune.targets[i].link = this->FindChildLink(_ecm, runeModel, linkName);
        if (rune.targets[i].link == kNullEntity)
          ok = false;
      }
      if (ok)
      {
        this->EnsureJointStateComponents(_ecm, rune.hubJoint);
        rune.found = true;
        gzmsg << "[RobominiSim] energy rune side '" << TeamName(rune.team)
              << "' found (hub link=" << rune.hubLink
              << ", hub joint=" << rune.hubJoint << ")" << std::endl;
        this->DiscoverRuneVisuals(_ecm, rune);
      }
    }

    // --- launchers ---------------------------------------------------------
    for (std::size_t t = 0; t < kTeamCount; ++t)
    {
      auto &ln = this->launchers[t];
      if (ln.found)
        continue;
      const std::string modelName = "launcher_" + TeamName(ln.team);
      auto model = _ecm.EntityByComponents(components::Model(),
          components::Name(modelName));
      if (model == kNullEntity)
        continue;
      ln.model = model;
      ln.yawLink = this->FindChildLink(_ecm, model, "yaw_link");
      ln.pitchLink = this->FindChildLink(_ecm, model, "pitch_link");
      ln.muzzleLink = this->FindChildLink(_ecm, model, "muzzle_link");
      ln.yawJoint = this->FindChildJoint(_ecm, model, "yaw_joint");
      ln.pitchJoint = this->FindChildJoint(_ecm, model, "pitch_joint");
      if (ln.yawJoint == kNullEntity || ln.pitchJoint == kNullEntity ||
          ln.muzzleLink == kNullEntity)
      {
        continue;
      }
      this->EnsureJointStateComponents(_ecm, ln.yawJoint);
      this->EnsureJointStateComponents(_ecm, ln.pitchJoint);
      ln.found = true;
      gzmsg << "[RobominiSim] " << modelName << " found (yaw/pitch joints="
            << ln.yawJoint << "/" << ln.pitchJoint
            << ", muzzle link=" << ln.muzzleLink << ")" << std::endl;
    }

    // --- ball pool ---------------------------------------------------------
    if (this->balls.empty() && this->ballCount > 0)
    {
      this->balls.resize(static_cast<std::size_t>(this->ballCount));
      for (int i = 0; i < this->ballCount; ++i)
      {
        std::ostringstream name;
        name << "ball_" << std::setw(2) << std::setfill('0') << i;
        this->balls[static_cast<std::size_t>(i)].name = name.str();
      }
    }
    bool allBallsFound = true;
    for (auto &ball : this->balls)
    {
      if (ball.model != kNullEntity)
        continue;
      auto model = _ecm.EntityByComponents(components::Model(),
          components::Name(ball.name));
      if (model == kNullEntity)
      {
        allBallsFound = false;
        continue;
      }
      ball.model = model;
      ball.home = gz::sim::worldPose(model, _ecm);
      ball.state = BallState::kParked;
      ball.hasPrev = false;
      gzmsg << "[RobominiSim] ball '" << ball.name
            << "' found at " << ball.home.Pos() << std::endl;
    }

    // Ball park is only a visual resting area; the plugin remembers the
    // initial pose of every ball instead of querying the park slots directly.
    const bool runesFound = this->runes[0].found && this->runes[1].found;
    const bool launchersFound =
        this->launchers[0].found && this->launchers[1].found;
    if (runesFound && launchersFound && allBallsFound)
    {
      this->discovered = true;
      gzmsg << "[RobominiSim] All contract entities discovered ("
            << this->balls.size() << " balls). Rune/launcher control active."
            << std::endl;
      this->ApplyAllLights(_ecm, true);
    }
    else if (!this->notFoundWarned &&
             this->simTime > 5.0)
    {
      this->notFoundWarned = true;
      gzwarn << "[RobominiSim] Still missing entities: runes("
             << this->runes[0].found << "," << this->runes[1].found
             << ") launchers(" << this->launchers[0].found << ","
             << this->launchers[1].found << ") ballsAllFound="
             << allBallsFound << ". Control inactive until they appear."
             << std::endl;
    }
  }

  public: void DiscoverRuneVisuals(gz::sim::EntityComponentManager &_ecm,
                                   RuneRuntime &_rune)
  {
    using namespace gz::sim;  // NOLINT
    bool allOk = true;
    for (std::size_t i = 0; i < kTargetCount; ++i)
    {
      auto link = _rune.targets[i].link;
      if (link == kNullEntity)
      {
        allOk = false;
        continue;
      }
      for (std::size_t b = 0; b < kBarCount; ++b)
      {
        if (_rune.targets[i].bars[b] != kNullEntity)
          continue;
        auto visuals = _ecm.ChildrenByComponents(link, components::Visual(),
            components::Name(std::string(kBarNames[b])));
        if (!visuals.empty())
          _rune.targets[i].bars[b] = visuals.front();
        else
          allOk = false;
      }
    }
    _rune.visualsReady = allOk;
    if (allOk)
    {
      gzmsg << "[RobominiSim] rune " << TeamName(_rune.team)
            << ": 20 light-bar visuals resolved (5 targets x 4 bars)"
            << std::endl;
      _rune.lightsDirty = false;
      _rune.appliedLights = _rune.lights;
      this->ApplyLights(_ecm, _rune);
    }
    else if (!this->runeVisualsWarned)
    {
      this->runeVisualsWarned = true;
      gzwarn << "[RobominiSim] rune " << TeamName(_rune.team)
             << ": some light-bar visuals not found yet "
             << "(check contract visual names bar_radial_inner/"
             << "bar_radial_outer/bar_tangential_1/bar_tangential_2)"
             << std::endl;
    }
  }

  public: gz::sim::Entity FindChildLink(
      gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _parent,
      const std::string &_name)
  {
    auto entities = _ecm.ChildrenByComponents(_parent,
        gz::sim::components::Link(), gz::sim::components::Name(_name));
    return entities.empty() ? gz::sim::kNullEntity : entities.front();
  }

  public: gz::sim::Entity FindChildJoint(
      gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _parent,
      const std::string &_name)
  {
    auto entities = _ecm.ChildrenByComponents(_parent,
        gz::sim::components::Joint(), gz::sim::components::Name(_name));
    return entities.empty() ? gz::sim::kNullEntity : entities.front();
  }

  /// \brief Make sure physics keeps updating JointPosition/JointVelocity for a
  /// joint. gz-sim only fills these components if they already exist.
  public: void EnsureJointStateComponents(
      gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _joint)
  {
    if (_ecm.Component<gz::sim::components::JointPosition>(_joint) == nullptr)
    {
      _ecm.CreateComponent(_joint,
          gz::sim::components::JointPosition(std::vector<double>{0.0}));
    }
    if (_ecm.Component<gz::sim::components::JointVelocity>(_joint) == nullptr)
    {
      _ecm.CreateComponent(_joint,
          gz::sim::components::JointVelocity(std::vector<double>{0.0}));
    }
  }

  public: double JointPosition(gz::sim::EntityComponentManager &_ecm,
                               gz::sim::Entity _joint) const
  {
    const auto *comp =
        _ecm.Component<gz::sim::components::JointPosition>(_joint);
    if (comp == nullptr || comp->Data().empty())
      return 0.0;
    return comp->Data()[0];
  }

  public: double JointVelocity(gz::sim::EntityComponentManager &_ecm,
                               gz::sim::Entity _joint) const
  {
    const auto *comp =
        _ecm.Component<gz::sim::components::JointVelocity>(_joint);
    if (comp == nullptr || comp->Data().empty())
      return 0.0;
    return comp->Data()[0];
  }

  // ------------------------------------------------------------------------
  // Command consumption (sim thread)
  // ------------------------------------------------------------------------
  public: void ConsumeCommands(gz::sim::EntityComponentManager &_ecm)
  {
    std::vector<ModeCommand> modes;
    std::vector<LightsCommand> lights;
    std::vector<GimbalCommand> gimbals;
    std::vector<FireCommand> fires;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      modes.swap(this->pendingModes);
      lights.swap(this->pendingLights);
      gimbals.swap(this->pendingGimbals);
      fires.swap(this->pendingFires);
    }

    for (const auto &cmd : modes)
      this->ApplyModeCommand(cmd);
    for (const auto &cmd : lights)
      this->ApplyLightsCommand(cmd, _ecm);
    for (const auto &cmd : gimbals)
      this->ApplyGimbalCommand(cmd);
    for (const auto &cmd : fires)
      this->TryFire(cmd, _ecm);
  }

  public: void ApplyModeCommand(const ModeCommand &_cmd)
  {
    auto &rune = this->runes[static_cast<std::size_t>(_cmd.team)];
    rune.mode = _cmd.mode;
    rune.bigA = _cmd.a;
    rune.bigOmega = _cmd.omega;
    if (_cmd.mode == "big")
    {
      if (_cmd.hasT0 && std::isfinite(_cmd.t0))
      {
        rune.bigT0 = _cmd.t0;
        rune.bigWaitT0 = false;
      }
      else
      {
        // t0 omitted -> phase starts when the command is received.
        rune.bigWaitT0 = true;
      }
    }
    else
    {
      rune.bigWaitT0 = false;
    }
    gzmsg << "[RobominiSim] /rune/" << TeamName(_cmd.team)
          << "/mode -> '" << _cmd.mode << "'"
          << (_cmd.mode == "big"
              ? (" a=" + Num(_cmd.a) + " omega=" + Num(_cmd.omega) +
                 (_cmd.hasT0 ? (" t0=" + Num(_cmd.t0)) : " t0=<now>"))
              : "")
          << std::endl;
  }

  public: void ApplyLightsCommand(const LightsCommand &_cmd,
                                  gz::sim::EntityComponentManager &_ecm)
  {
    auto &rune = this->runes[static_cast<std::size_t>(_cmd.team)];
    bool changed = false;
    for (std::size_t i = 0; i < kTargetCount; ++i)
    {
      if (rune.lights[i] != _cmd.states[i])
        changed = true;
      rune.lights[i] = _cmd.states[i];
    }
    if (!changed)
      return;
    rune.lightsDirty = true;
    if (rune.visualsReady)
    {
      this->ApplyLights(_ecm, rune);
      rune.lightsDirty = false;
    }
  }

  public: void ApplyGimbalCommand(const GimbalCommand &_cmd)
  {
    auto &ln = this->launchers[static_cast<std::size_t>(_cmd.team)];
    ln.yawTarget = Clamp(_cmd.yaw, -this->yawLimit, this->yawLimit);
    ln.pitchTarget = Clamp(_cmd.pitch, this->pitchMin, this->pitchMax);
    // 前馈：缺省 0 = 只跟随角度目标，与原有行为一致（回归基线）。
    ln.yawTargetVel = _cmd.yawVel;
    ln.pitchTargetVel = _cmd.pitchVel;
    ln.yawTargetAcc = _cmd.yawAcc;
    ln.pitchTargetAcc = _cmd.pitchAcc;
  }

  public: void TryFire(const FireCommand &_cmd,
                       gz::sim::EntityComponentManager &_ecm)
  {
    const std::size_t ti = static_cast<std::size_t>(_cmd.team);
    auto &ln = this->launchers[ti];
    if (!ln.found)
    {
      gzwarn << "[RobominiSim] /launcher/" << TeamName(_cmd.team)
             << "/fire: launcher model 'launcher_" << TeamName(_cmd.team)
             << "' not found, command dropped" << std::endl;
      return;
    }
    const double minInterval = 1.0 / this->fireRate;
    if (this->simTime - ln.lastFireTime < minInterval - 1e-9)
    {
      if (!this->rateWarned[ti])
      {
        this->rateWarned[ti] = true;
        gzwarn << "[RobominiSim] /launcher/" << TeamName(_cmd.team)
               << "/fire: rate limited to " << this->fireRate
               << " shots/s, command dropped (further drops are silent)"
               << std::endl;
      }
      return;
    }

    BallRuntime *ball = nullptr;
    for (auto &b : this->balls)
    {
      if (b.model != gz::sim::kNullEntity && b.state == BallState::kParked)
      {
        ball = &b;
        break;
      }
    }
    if (ball == nullptr)
    {
      if (!this->noBallWarned[ti])
      {
        this->noBallWarned[ti] = true;
        gzwarn << "[RobominiSim] /launcher/" << TeamName(_cmd.team)
               << "/fire: no parked ball available (pool exhausted/not "
               << "discovered), command dropped" << std::endl;
      }
      return;
    }

    const FireResult result = this->Fire(ln, *ball, _cmd.speed, _ecm);
    if (!result.ok)
      return;

    ln.lastFireTime = this->simTime;
    ln.ballsFired++;

    gzmsg << "[RobominiSim] launcher " << TeamName(_cmd.team)
          << " fired " << ball->name << " at speed " << Num(result.speed)
          << " m/s (total=" << ln.ballsFired << ")" << std::endl;
  }

  /// \brief Teleport a ball in front of the muzzle and give it muzzle speed.
  public: FireResult Fire(LauncherRuntime &_ln, BallRuntime &_ball,
                          double _speed,
                          gz::sim::EntityComponentManager &_ecm)
  {
    FireResult result;
    if (!this->ComputeMuzzle(_ln, _ecm))
    {
      gzwarn << "[RobominiSim] launcher " << TeamName(_ln.team)
             << ": cannot compute muzzle pose (link chain incomplete), "
             << "fire dropped" << std::endl;
      return result;
    }

    gz::math::Vector3d dir = this->ApplyMuzzleSpread(_ln.muzzleDir);
    const double speed = Clamp(_speed, 0.5, 100.0);
    const gz::math::Vector3d spawnPos =
        _ln.muzzlePos + dir * this->muzzleOffset;

    // One-shot teleport + velocity. The model orientation is kept identity so
    // the ball always restarts unrotated.
    //
    // NOTE: the physics system converts a *model-level* LinearVelocityCmd with
    // the model's components::Pose rotation. The pose command above is applied
    // to the physics free group during UpdatePhysics, but the ECM Pose
    // component still holds the orientation from the ball's previous flight
    // (bouncing adds random spin), and the velocity command is processed after
    // the pose command using that stale rotation. Compensate by expressing the
    // command in the model's *current* frame:
    gz::math::Quaterniond modelRot = gz::math::Quaterniond::Identity;
    const auto *poseComp =
        _ecm.Component<gz::sim::components::Pose>(_ball.model);
    if (poseComp != nullptr && poseComp->Data().Rot().IsFinite())
      modelRot = poseComp->Data().Rot();
    _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(_ball.model,
        gz::math::Pose3d(spawnPos, gz::math::Quaterniond::Identity));
    _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(_ball.model,
        modelRot.Inverse() * (dir * speed));

    _ball.state = BallState::kFlying;
    _ball.launchTime = this->simTime;
    _ball.launchSpeed = speed;
    _ball.team = _ln.team;
    _ball.hitReported = false;
    _ball.prevPos = spawnPos;
    _ball.hasPrev = false;
    _ball.lowZStart = -1.0;
    _ball.nextTraceTime = 0.0;
    _ball.pendingTeleport = true;
    _ball.cleanupStep = this->stepCount + 1;
    _ball.spawnPos = spawnPos;
    _ball.launchStep = this->stepCount;

    if (this->ballTrace)
    {
      gzmsg << "[FIREDBG] " << _ball.name << " spawn=" << Num(spawnPos.X())
            << " " << Num(spawnPos.Y()) << " " << Num(spawnPos.Z())
            << " dir=" << Num(dir.X()) << " " << Num(dir.Y()) << " "
            << Num(dir.Z()) << " speed=" << Num(speed) << std::endl;
      std::cout.flush();
    }

    result.ok = true;
    result.ball = _ball.name;
    result.speed = speed;
    result.muzzle = spawnPos;
    return result;
  }

  public: gz::math::Vector3d ApplyMuzzleSpread(
      const gz::math::Vector3d &_dir)
  {
    if (this->muzzleSpreadDeg <= 1e-9)
      return _dir;
    const double maxRad = this->muzzleSpreadDeg * kDegToRad;
    std::uniform_real_distribution<double> dist(-maxRad, maxRad);
    const gz::math::Vector3d ref =
        std::abs(_dir.Z()) < 0.9 ? gz::math::Vector3d::UnitZ
                                 : gz::math::Vector3d::UnitX;
    gz::math::Vector3d axis1 = _dir.Cross(ref);
    if (axis1.Length() < 1e-6)
      return _dir;
    axis1.Normalize();
    gz::math::Vector3d axis2 = _dir.Cross(axis1);
    axis2.Normalize();
    gz::math::Vector3d out = _dir;
    out = gz::math::Quaterniond(axis1, dist(this->rng)) * out;
    out = gz::math::Quaterniond(axis2, dist(this->rng)) * out;
    out.Normalize();
    return out;
  }

  /// \brief Compute muzzle world position + barrel direction.
  /// Barrel direction is the interface definition: at yaw=pitch=0 the barrel
  /// points along +Y in the launcher model frame; the model pose (identity for
  /// blue, rotated 180 deg for red) is applied last.
  public: bool ComputeMuzzle(LauncherRuntime &_ln,
                             gz::sim::EntityComponentManager &_ecm)
  {
    if (_ln.model == gz::sim::kNullEntity ||
        _ln.muzzleLink == gz::sim::kNullEntity)
    {
      return false;
    }
    const auto muzzlePose = gz::sim::worldPose(_ln.muzzleLink, _ecm);
    if (!muzzlePose.Pos().IsFinite())
      return false;

    // Barrel direction: the muzzle/barrel is rigidly attached to pitch_link and
    // the contract fixes the barrel axis at +Y in the pitch_link frame, so use
    // the *actual* pitch_link world rotation (this is robust to whether the
    // asset authored the pitch joint/child poses with extra static offsets).
    gz::math::Vector3d dir = gz::math::Vector3d::UnitY;
    if (_ln.pitchLink != gz::sim::kNullEntity)
    {
      const auto pitchPose = gz::sim::worldPose(_ln.pitchLink, _ecm);
      if (pitchPose.Rot().IsFinite())
        dir = pitchPose.Rot() * this->barrelAxisLocal;
    }
    else
    {
      const auto modelPose = gz::sim::worldPose(_ln.model, _ecm);
      const double yaw = this->JointPosition(_ecm, _ln.yawJoint);
      const double pitch = this->JointPosition(_ecm, _ln.pitchJoint);
      const gz::math::Quaterniond localRot =
          gz::math::Quaterniond(gz::math::Vector3d::UnitZ, yaw) *
          gz::math::Quaterniond(gz::math::Vector3d::UnitX, pitch);
      dir = modelPose.Rot() * (localRot * this->barrelAxisLocal);
    }
    if (!dir.IsFinite() || dir.Length() < 1e-6)
      return false;
    dir.Normalize();

    _ln.muzzlePos = muzzlePose.Pos();
    _ln.muzzleDir = dir;
    _ln.haveMuzzle = true;
    return true;
  }

  // ------------------------------------------------------------------------
  // Rune control + lights
  // ------------------------------------------------------------------------
  public: void UpdateRunes(gz::sim::EntityComponentManager &_ecm)
  {
    for (std::size_t t = 0; t < kTeamCount; ++t)
    {
      auto &rune = this->runes[t];
      if (!rune.found)
        continue;

      rune.hubAngle = this->JointPosition(_ecm, rune.hubJoint);
      rune.hubSpeed = this->JointVelocity(_ecm, rune.hubJoint);

      if (this->geometryDebug && !rune.geomLogged && this->simTime > 0.02)
      {
        rune.geomLogged = true;
        this->LogRuneGeometry(_ecm, rune);
      }

      double targetSpeed = 0.0;
      if (rune.mode == "small")
      {
        targetSpeed = kPi / 3.0;
      }
      else if (rune.mode == "big")
      {
        if (rune.bigWaitT0)
        {
          rune.bigT0 = this->simTime;
          rune.bigWaitT0 = false;
        }
        targetSpeed = rune.bigA *
            std::sin(rune.bigOmega * (this->simTime - rune.bigT0)) +
            (2.090 - rune.bigA);
      }
      // fixed / random_fixed -> 0, hold current angle.

      _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
          rune.hubJoint, std::vector<double>{targetSpeed});

      // lazily resolve light-bar visuals (models may be spawned after config)
      if (!rune.visualsReady &&
          (this->lastDiscoverAttempt < 0.0 ||
           this->simTime - this->lastDiscoverAttempt > 0.5))
      {
        this->DiscoverRuneVisuals(_ecm, rune);
      }
      if (rune.lightsDirty && rune.visualsReady)
      {
        this->ApplyLights(_ecm, rune);
        rune.lightsDirty = false;
      }
    }
  }

  /// \brief One-shot diagnostic (contract integration aid): prints the world
  /// pose of the hub link, of every target collision and the plate centre
  /// computed from the contract formula.  Used to verify that the delivered
  /// energy_rune asset matches the documented frame conventions.
  public: void LogRuneGeometry(gz::sim::EntityComponentManager &_ecm,
                               RuneRuntime &_rune)
  {
    const auto hub = gz::sim::worldPose(_rune.hubLink, _ecm);
    gzmsg << "[GEOMDBG] rune " << TeamName(_rune.team)
          << " hub_world_pos=" << Num(hub.Pos().X()) << " "
          << Num(hub.Pos().Y()) << " " << Num(hub.Pos().Z())
          << " hub_world_rpy="
          << Num(hub.Rot().Roll()) << " " << Num(hub.Rot().Pitch())
          << " " << Num(hub.Rot().Yaw())
          << " hub_joint_angle=" << Num(_rune.hubAngle) << std::endl;
    const auto axis = this->RuneAxis(_rune.team);
    for (std::size_t i = 0; i < kTargetCount; ++i)
    {
      if (_rune.targets[i].link == gz::sim::kNullEntity)
        continue;
      const auto tgt = gz::sim::worldPose(_rune.targets[i].link, _ecm);

      gzmsg << "[GEOMDBG]   tgt_" << TeamName(_rune.team) << "_" << i
            << " link_world_pos=" << Num(tgt.Pos().X()) << " "
            << Num(tgt.Pos().Y()) << " " << Num(tgt.Pos().Z());
      const auto cols = _ecm.ChildrenByComponents(_rune.targets[i].link,
          gz::sim::components::Collision(),
          gz::sim::components::Name("plate"));
      if (!cols.empty())
      {
        const auto col = gz::sim::worldPose(cols.front(), _ecm);
        gzmsg << " collision_world_pos=" << Num(col.Pos().X()) << " "
              << Num(col.Pos().Y()) << " " << Num(col.Pos().Z());
      }
      const double theta = TargetAngleRad(_rune.team, i) - _rune.hubAngle;
      gzmsg << " formula_center="
            << Num(axis.X() + this->targetRadius * std::cos(theta)) << " "
            << Num(axis.Y()) << " "
            << Num(axis.Z() + this->targetRadius * std::sin(theta))
            << std::endl;
    }
  }

  public: void ApplyAllLights(gz::sim::EntityComponentManager &_ecm,
                              bool _force)
  {
    for (auto &rune : this->runes)
    {
      if (!rune.found)
        continue;
      if (!rune.visualsReady)
        this->DiscoverRuneVisuals(_ecm, rune);
      if (rune.visualsReady)
      {
        if (_force)
          rune.lightsDirty = true;
        this->ApplyLights(_ecm, rune);
        rune.lightsDirty = false;
      }
    }
  }

  /// \brief Apply the 5 target light states of one rune side to the 40
  /// light-bar visuals (state mapping: rule 5.5.4).
  public: void ApplyLights(gz::sim::EntityComponentManager &_ecm,
                           RuneRuntime &_rune)
  {
    const double litR = (_rune.team == Team::kBlue) ? 0.0 : 1.0;
    const double litG = (_rune.team == Team::kBlue) ? 0.25 : 0.08;
    const double litB = (_rune.team == Team::kBlue) ? 1.0 : 0.05;
    constexpr double offR = 0.02, offG = 0.02, offB = 0.02;

    for (std::size_t i = 0; i < kTargetCount; ++i)
    {
      const LightState st = _rune.lights[i];
      const bool radialOn = (st == LightState::kPending);
      const bool tangentialOn =
          (st == LightState::kPending || st == LightState::kActive);
      for (std::size_t b = 0; b < kBarCount; ++b)
      {
        gz::sim::Entity visual = _rune.targets[i].bars[b];
        if (visual == gz::sim::kNullEntity)
          continue;
        const bool radial = (b < 2);
        const bool on = radial ? radialOn : tangentialOn;

        gz::msgs::Visual visualMsg;
        visualMsg.set_id(static_cast<std::uint64_t>(visual));
        auto *material = visualMsg.mutable_material();
        if (on)
        {
          // Lit bar: colored ambient/diffuse + full emissive so the state is
          // clearly visible in the (headless) camera image.
          SetColor(material->mutable_ambient(), 0.4 * litR, 0.4 * litG,
              0.4 * litB);
          SetColor(material->mutable_diffuse(), litR, litG, litB);
          SetColor(material->mutable_specular(), 0.05, 0.05, 0.05);
          SetColor(material->mutable_emissive(), litR, litG, litB);
        }
        else
        {
          SetColor(material->mutable_ambient(), offR, offG, offB);
          SetColor(material->mutable_diffuse(), offR, offG, offB);
          SetColor(material->mutable_specular(), 0.0, 0.0, 0.0);
          SetColor(material->mutable_emissive(), 0.0, 0.0, 0.0);
        }
        _ecm.SetComponentData<gz::sim::components::VisualCmd>(visual,
            visualMsg);
      }
    }
    _rune.appliedLights = _rune.lights;
    gzmsg << "[RobominiSim] rune " << TeamName(_rune.team)
          << " lights applied: ";
    for (std::size_t i = 0; i < kTargetCount; ++i)
      gzmsg << LightStateName(_rune.lights[i]) << (i + 1 < kTargetCount ? " " : "");
    gzmsg << std::endl;
  }

  // ------------------------------------------------------------------------
  // Launcher gimbal control (: 角度 + 速度/加速度前馈伺服)
  // ------------------------------------------------------------------------
  /// \brief 单轴伺服律：
  ///   v_cmd = clamp(Kp·err + v_ff, ±v_max)，再按 a_limit·dt 做速度限幅
  ///   （a_limit = max(|a_ff|, a_max)），最后再夹一次 ±v_max；
  ///   |err| < 0.5·tol 且 |v_ff| < 0.02 rad/s 时置零（到位保持，与旧版一致）。
  /// \param[in] _err 角度误差 rad
  /// \param[in] _velFF 速度前馈 rad/s（0=纯角度指令）
  /// \param[in] _accFF 加速度前馈 rad/s²（只在大于默认 a_max 时放宽限幅）
  /// \param[in] _vMax 该轴最大角速度 rad/s
  /// \param[in] _aMax 该轴默认加速度限幅 rad/s²（SDF max_{yaw,pitch}_accel）
  /// \param[in,out] _prevCmd 上一步下发命令；返回本步命令（供下一步限幅）
  /// \return 本步下发的关节角速度命令 rad/s
  public: double ServoAxis(double _err, double _velFF, double _accFF,
                           double _vMax, double _aMax,
                           double &_prevCmd) const
  {
    if (std::abs(_err) < 0.5 * this->gimbalTolerance &&
        std::abs(_velFF) < 0.02)
    {
      _prevCmd = 0.0;
      return 0.0;
    }
    const double target =
        Clamp(this->gimbalGain * _err + _velFF, -_vMax, _vMax);
    const double aLimit = std::max(std::abs(_accFF), _aMax);
    const double slew = aLimit * this->stepDt;
    const double cmd = Clamp(Clamp(target, _prevCmd - slew, _prevCmd + slew),
                             -_vMax, _vMax);
    _prevCmd = cmd;
    return cmd;
  }

  public: void UpdateLaunchers(gz::sim::EntityComponentManager &_ecm)
  {
    for (std::size_t t = 0; t < kTeamCount; ++t)
    {
      auto &ln = this->launchers[t];
      if (!ln.found)
        continue;

      // 实测角速度反馈（前馈接口）：关节位置差分 + 一阶低通
      //   v = 0.8·v_prev + 0.2·Δp/dt
      // （physics 的 JointVelocity 组件同样可用，但接口约定指定位置差分+低通）。
      const double lastYaw = ln.yaw;
      const double lastPitch = ln.pitch;
      ln.yaw = this->JointPosition(_ecm, ln.yawJoint);
      ln.pitch = this->JointPosition(_ecm, ln.pitchJoint);
      if (ln.havePrevJoints && this->stepDt > 1e-9)
      {
        const double invDt = 1.0 / this->stepDt;
        ln.yawVel = 0.8 * ln.yawVel + 0.2 * (ln.yaw - lastYaw) * invDt;
        ln.pitchVel = 0.8 * ln.pitchVel +
            0.2 * (ln.pitch - lastPitch) * invDt;
      }
      ln.havePrevJoints = true;

      const double errYaw = ln.yawTarget - ln.yaw;
      const double errPitch = ln.pitchTarget - ln.pitch;
      if (ln.yawJoint != gz::sim::kNullEntity)
      {
        const double cmdYaw = this->ServoAxis(errYaw, ln.yawTargetVel,
            ln.yawTargetAcc, this->yawMaxVel, this->yawMaxAccel,
            ln.yawCmdVel);
        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
            ln.yawJoint, std::vector<double>{cmdYaw});
      }
      if (ln.pitchJoint != gz::sim::kNullEntity)
      {
        const double cmdPitch = this->ServoAxis(errPitch, ln.pitchTargetVel,
            ln.pitchTargetAcc, this->pitchMaxVel, this->pitchMaxAccel,
            ln.pitchCmdVel);
        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
            ln.pitchJoint, std::vector<double>{cmdPitch});
      }

      ln.ready = std::abs(errYaw) < this->gimbalTolerance &&
                 std::abs(errPitch) < this->gimbalTolerance;
      this->ComputeMuzzle(ln, _ecm);
    }
  }

  // ------------------------------------------------------------------------
  // Ball tracking, swept hit detection, recycling
  // ------------------------------------------------------------------------
  public: void UpdateBalls(gz::sim::EntityComponentManager &_ecm)
  {
    this->MaybeResetParkedBalls(_ecm);
    for (auto &ball : this->balls)
    {
      if (ball.model == gz::sim::kNullEntity)
        continue;

      // One-shot physics commands are applied by the physics system during the
      // Update phase and, at the end of the step, physics only *zeroes* the
      // LinearVelocityCmd data and re-applies+removes WorldPoseCmd one step
      // later ("let them rotate one iteration"). A component that stays around
      // with zero data would therefore force the ball to a full stop on every
      // following step, and the pose command would undo the first integration
      // step. Remove both components here, one step after they were issued.
      if (ball.cleanupStep >= 0 && this->stepCount >= ball.cleanupStep)
      {
        _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(ball.model);
        _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
            ball.model);
        ball.cleanupStep = -1;
      }

      if (ball.state != BallState::kFlying)
        continue;

      const gz::math::Pose3d pose = gz::sim::worldPose(ball.model, _ecm);
      const gz::math::Vector3d pos = pose.Pos();
      if (!pos.IsFinite())
        continue;

      // The one-shot WorldPoseCmd/LinVelCmd are applied by the physics system
      // during this step's Update phase, i.e. after PreUpdate. Until the
      // teleport has visibly taken effect the ball is still somewhere in the
      // ball park; do not trace/hit-test/recycle it during that step.
      //
      // The check compares against the *spawn position* (not the recorded
      // home) because parked balls get shoved around by collisions, so their
      // current position can legitimately be far away from `home`.
      if (ball.pendingTeleport)
      {
        if ((pos - ball.spawnPos).Length() < 0.2)
        {
          // physics has applied the pose command: the ball is now at the
          // muzzle spawn point instead of somewhere in the ball park
          ball.pendingTeleport = false;
          ball.hasPrev = false;
          ball.prevPos = pos;
        }
        else if (this->stepCount - ball.launchStep > 8)
        {
          gzwarn << "[RobominiSim] ball " << ball.name
                 << ": WorldPoseCmd teleport not observed after 8 steps, "
                 << "clearing pendingTeleport (pos=" << pos << ")"
                 << std::endl;
          ball.pendingTeleport = false;
        }
        else
        {
          continue;
        }
      }

      if (!ball.hasPrev)
      {
        ball.prevPos = pos;
        ball.hasPrev = true;
      }

      if (!ball.hitReported)
        this->DetectHit(ball, ball.prevPos, pos, _ecm);

      if (this->ballTrace && (this->simTime - ball.launchTime) < 0.006)
      {
        const bool hasPose =
            _ecm.Component<gz::sim::components::WorldPoseCmd>(ball.model)
                != nullptr;
        const bool hasVel =
            _ecm.Component<gz::sim::components::LinearVelocityCmd>(ball.model)
                != nullptr;
        gzmsg << "[STEPTRACE] " << ball.name << " t="
              << Num(this->simTime - ball.launchTime) << " pos=" << Num(pos.X())
              << " " << Num(pos.Y()) << " " << Num(pos.Z())
              << " poseCmd=" << (hasPose ? 1 : 0)
              << " velCmd=" << (hasVel ? 1 : 0) << std::endl;
        std::cout.flush();
      }

      if (this->ballTrace && this->simTime >= ball.nextTraceTime)
      {
        ball.nextTraceTime = this->simTime + 0.02;
        gzmsg << "[BALLTRACE] " << ball.name << " t="
              << Num(this->simTime - ball.launchTime) << " pos="
              << Num(pos.X()) << " " << Num(pos.Y()) << " " << Num(pos.Z())
              << std::endl;
        // flush so shell/python readers can consume the trace live
        std::cout.flush();
      }

      const double flightTime = this->simTime - ball.launchTime;
      // Double safety: never recycle a ball during the first 50 ms after the
      // shot (teleport application + first integration steps).
      if (flightTime < 0.05)
      {
        ball.prevPos = pos;
        continue;
      }
      bool recycle = false;
      if (flightTime > this->ballFlightTimeout)
        recycle = true;
      if (std::abs(pos.X()) > this->fieldHalfX ||
          std::abs(pos.Y()) > this->fieldHalfY ||
          pos.Z() < this->fieldMinZ)
      {
        recycle = true;
      }
      if (pos.Z() < 0.05)
      {
        if (ball.lowZStart < 0.0)
          ball.lowZStart = this->simTime;
        else if (this->simTime - ball.lowZStart > this->ballRestTimeout)
          recycle = true;
      }
      else
      {
        ball.lowZStart = -1.0;
      }

      if (recycle)
      {
        this->RecycleBall(ball, _ecm);
      }
      else
      {
        ball.prevPos = pos;
      }
    }
  }

  /// \brief Periodically put parked balls back on their recorded home poses.
  /// Balls pile up / get shoved in the ball park over a long session, which
  /// makes the pool layout drift (and would make repeated firing unreliable).
  /// Running this only touches balls in the hidden park (no gameplay effect).
  public: void MaybeResetParkedBalls(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->parkResetInterval <= 0.0)
      return;
    if (this->simTime - this->lastParkReset < this->parkResetInterval)
      return;
    this->lastParkReset = this->simTime;
    for (auto &ball : this->balls)
    {
      if (ball.model == gz::sim::kNullEntity ||
          ball.state != BallState::kParked ||
          ball.pendingTeleport || ball.cleanupStep >= 0)
      {
        continue;
      }
      const auto pos = gz::sim::worldPose(ball.model, _ecm).Pos();
      if (!pos.IsFinite())
        continue;
      if ((pos - ball.home.Pos()).Length() < 0.08)
        continue;
      _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(ball.model,
          ball.home);
      _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(
          ball.model, gz::math::Vector3d::Zero);
      ball.cleanupStep = this->stepCount + 1;
    }
  }

  public: void RecycleBall(BallRuntime &_ball,
                           gz::sim::EntityComponentManager &_ecm)
  {
    _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(_ball.model,
        _ball.home);
    _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(
        _ball.model, gz::math::Vector3d::Zero);
    _ball.state = BallState::kParked;
    _ball.hasPrev = false;
    _ball.hitReported = false;
    _ball.lowZStart = -1.0;
    _ball.pendingTeleport = false;
    _ball.cleanupStep = this->stepCount + 1;
  }

  /// \brief Swept-ball vs. plate test against the 5 target plates of the
  /// ball's own team.
  ///
  /// Plate geometry (): target i is a phi=0.195 m disk (thickness
  /// 0.022 m) whose centre lies on the rune circle of radius 0.35 m at polar
  /// angle theta(t) = theta0_i - hub_angle, in a plane parallel to X-Z.
  ///
  /// Two acceptance cases are handled:
  ///   a) the swept segment crosses the plate centre plane (ball flies through
  ///      / collision disabled asset) -- radial disk test at the crossing;
  ///   b) the swept segment ends inside the plate slab expanded by the ball
  ///      radius (ball is stopped by the collision at the plate surface, so its
  ///      centre never reaches the centre plane) -- radial disk test at the
  ///      point closest to the centre plane.
  /// This makes the detection robust both for a few-mm-per-step low speed ball
  /// and for a 12+ m/s ball that advances > ball diameter per step.
  public: void DetectHit(BallRuntime &_ball,
                         const gz::math::Vector3d &_prev,
                         const gz::math::Vector3d &_curr,
                         gz::sim::EntityComponentManager &_ecm)
  {
    (void)_ecm;
    const std::size_t ti = static_cast<std::size_t>(_ball.team);
    auto &rune = this->runes[ti];
    if (!rune.found)
      return;

    const gz::math::Vector3d seg = _curr - _prev;
    if (seg.Length() < 1e-9)
      return;
    const gz::math::Vector3d segDir = seg.Normalized();
    const double hitRadius = 0.5 * this->plateDiameter + this->ballRadius;
    const double slabHalf = 0.5 * this->plateThickness + this->ballRadius +
        this->hitPlaneSlack;
    const gz::math::Vector3d axis = this->RuneAxis(_ball.team);

    double bestT = 2.0;
    std::size_t bestTarget = 0;
    gz::math::Vector3d bestPoint = gz::math::Vector3d::Zero;
    for (std::size_t i = 0; i < kTargetCount; ++i)
    {
      const double theta = TargetAngleRad(_ball.team, i) - rune.hubAngle;
      const gz::math::Vector3d center(
          axis.X() + this->targetRadius * std::cos(theta),
          axis.Y(),
          axis.Z() + this->targetRadius * std::sin(theta));
      const double d0 = _prev.Y() - center.Y();
      const double d1 = _curr.Y() - center.Y();

      gz::math::Vector3d point;
      double t = 2.0;
      bool candidate = false;
      if ((d0 > 0.0 && d1 <= 0.0) || (d0 < 0.0 && d1 >= 0.0))
      {
        // (a) crossing the plate centre plane
        const double denom = d0 - d1;
        if (std::abs(denom) > 1e-12)
        {
          t = d0 / denom;
          point = _prev + t * seg;
          candidate = true;
        }
      }
      if (!candidate)
      {
        // (b) segment endpoint inside the slab expanded by ball radius
        const double ad0 = std::abs(d0);
        const double ad1 = std::abs(d1);
        if (ad0 <= slabHalf || ad1 <= slabHalf)
        {
          const double ad = std::min(ad0, ad1);
          point = (ad0 <= ad1) ? _prev : _curr;
          // map to a pseudo parameter so that the nearest plate wins
          t = (ad0 <= ad1) ? 0.0 : 1.0;
          if (ad > slabHalf)
            candidate = false;
          else
            candidate = true;
        }
      }
      if (!candidate || t >= bestT)
        continue;

      const gz::math::Vector3d radial(point.X() - center.X(), 0.0,
                                      point.Z() - center.Z());
      if (radial.Length() > hitRadius)
        continue;
      bestT = t;
      bestTarget = i;
      bestPoint = point;
    }

    if (bestT > 1.0)
      return;

    const bool direct = this->IsDirectShot(_ball, segDir);
    _ball.hitReported = true;
    this->PublishHit(_ball, bestTarget, bestPoint, segDir, direct);
  }

  public: bool IsDirectShot(const BallRuntime &_ball,
                            const gz::math::Vector3d &_segDir) const
  {
    // approximation: no contact with ground/guard/highland
    // detection, so approximate "direct" as: fast ball, still within 3 s of
    // launch and travelling towards the plate face.
    const double speed = _ball.launchSpeed;
    const double outwardY = OutwardNormalY(_ball.team);
    const double dot = _segDir.Y() * outwardY;
    const double dtHit = this->simTime - _ball.launchTime;
    return speed > 3.0 && dot < 0.0 && dtHit < 3.0;
  }

  public: void PublishHit(const BallRuntime &_ball, std::size_t _target,
                          const gz::math::Vector3d &_pos,
                          const gz::math::Vector3d &_segDir, bool _direct)
  {
    const std::size_t ti = static_cast<std::size_t>(_ball.team);
    const double speed = _ball.launchSpeed;
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(6);
    ss << "{\"target\":" << _target
       << ",\"ball\":\"" << robomini::json::Escape(_ball.name) << "\""
       << ",\"speed\":" << Num(speed)
       << ",\"direct\":" << (_direct ? "true" : "false")
       << ",\"pos\":[" << Num(_pos.X()) << "," << Num(_pos.Y())
       << "," << Num(_pos.Z()) << "]"
       << ",\"stamp\":" << Num(this->simTime) << "}";
    gz::msgs::StringMsg msg;
    msg.set_data(ss.str());
    if (this->hitPub[ti].Valid())
      this->hitPub[ti].Publish(msg);
    gzmsg << "[RobominiSim] HIT /rune/" << TeamName(_ball.team) << "/hit "
          << ss.str() << " segDir=[" << Num(_segDir.X()) << ","
          << Num(_segDir.Y()) << "," << Num(_segDir.Z()) << "]"
          << std::endl;
  }

  public: gz::math::Vector3d RuneAxis(Team _team) const
  {
    return _team == Team::kBlue ? this->runeAxisBlue : this->runeAxisRed;
  }

  // ------------------------------------------------------------------------
  // 20 Hz state publication
  // ------------------------------------------------------------------------
  public: void MaybePublishState(gz::sim::EntityComponentManager &_ecm)
  {
    (void)_ecm;
    const double period = 1.0 / this->publishRate;
    if (this->simTime - this->lastStatePublish < period - 1e-9)
      return;
    this->lastStatePublish = this->simTime;

    for (std::size_t t = 0; t < kTeamCount; ++t)
    {
      const auto &rune = this->runes[t];
      if (rune.found)
      {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(6);
        ss << "{\"hub_angle\":" << Num(rune.hubAngle)
           << ",\"speed\":" << Num(rune.hubSpeed)
           << ",\"mode\":\"" << robomini::json::Escape(rune.mode) << "\""
           << ",\"targets\":[";
        for (std::size_t i = 0; i < kTargetCount; ++i)
        {
          ss << "\"" << LightStateName(rune.lights[i]) << "\""
             << (i + 1 < kTargetCount ? "," : "");
        }
        ss << "],\"valid\":true";
        if (this->stateStamp)
          ss << ",\"stamp\":" << Num(this->simTime);
        ss << "}";
        gz::msgs::StringMsg msg;
        msg.set_data(ss.str());
        if (this->runeStatePub[t].Valid())
          this->runeStatePub[t].Publish(msg);
      }

      const auto &ln = this->launchers[t];
      if (ln.found)
      {
        gz::math::Vector3d muzzle = ln.muzzlePos;
        if (!ln.haveMuzzle)
          muzzle = gz::math::Vector3d::Zero;
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(6);
        ss << "{\"yaw\":" << Num(ln.yaw)
           << ",\"pitch\":" << Num(ln.pitch)
           // 实测关节角速度（位置差分 + 低通）
           << ",\"yaw_vel\":" << Num(ln.yawVel)
           << ",\"pitch_vel\":" << Num(ln.pitchVel)
           << ",\"muzzle\":[" << Num(muzzle.X()) << ","
           << Num(muzzle.Y()) << "," << Num(muzzle.Z()) << "]"
           << ",\"balls_fired\":" << ln.ballsFired
           << ",\"ready\":" << (ln.ready ? "true" : "false");
        if (this->stateStamp)
          ss << ",\"stamp\":" << Num(this->simTime);
        ss << "}";
        gz::msgs::StringMsg msg;
        msg.set_data(ss.str());
        if (this->launcherStatePub[t].Valid())
          this->launcherStatePub[t].Publish(msg);
      }
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
// RobominiSim forwarding
////////////////////////////////////////////////////////////////////////////////
RobominiSim::RobominiSim() : dataPtr(std::make_unique<Impl>())
{
}

RobominiSim::~RobominiSim() = default;

void RobominiSim::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->Configure(_entity, _sdf, _ecm);
}

void RobominiSim::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
  this->dataPtr->PreUpdate(_info, _ecm);
}

}  // namespace sim
}  // namespace robomini

GZ_ADD_PLUGIN(robomini::sim::RobominiSim,
              gz::sim::System,
              robomini::sim::RobominiSim::ISystemConfigure,
              robomini::sim::RobominiSim::ISystemPreUpdate)
