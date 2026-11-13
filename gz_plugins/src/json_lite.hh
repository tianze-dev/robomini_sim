/*
 * json_lite.hh — minimal, dependency-free, robust JSON reader/writer helpers.
 *
 * Why: the Gazebo system plugin must decode/encode the JSON payloads of the
 * contract topics (/rune/<team>/mode, /lights, /hit, ...) without pulling in an
 * extra dependency. gz-transport only carries gz.msgs.StringMsg, so the JSON
 * is plain text.
 *
 * Design goals (contract requirement "所有话题 JSON 编解码要健壮"):
 *   - never throw: malformed JSON returns false + error string, caller logs a
 *     warning and keeps the previous/default values;
 *   - missing fields -> typed accessors return caller-provided defaults;
 *   - bounded recursion depth, so a hostile/int-heap message can't blow the
 *     stack;
 *   - accepts trailing whitespace, rejects trailing garbage.
 *
 * This is a "good enough for the contract payloads" parser: objects, arrays,
 * strings (with \" \\ \/ \b \f \n \r \t and \uXXXX escapes), numbers, true,
 * false, null. No comments, no big-number exactness beyond double.
 *
 * Part of robomini_sim Gazebo plugin. License: Apache-2.0 (same as gz-sim).
 */
#ifndef ROBOMINI_JSON_LITE_HH_
#define ROBOMINI_JSON_LITE_HH_

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace robomini
{
namespace json
{

/// \brief A parsed JSON value (also used as a small DOM).
class Value
{
  public: enum class Type
  {
    kNull,
    kBool,
    kNumber,
    kString,
    kArray,
    kObject
  };

  public: Type type = Type::kNull;
  public: bool boolean = false;
  public: double number = 0.0;
  public: std::string string;
  public: std::vector<Value> array;
  public: std::map<std::string, Value> object;

  public: bool IsNull() const { return this->type == Type::kNull; }
  public: bool IsNumber() const { return this->type == Type::kNumber; }
  public: bool IsString() const { return this->type == Type::kString; }
  public: bool IsArray() const { return this->type == Type::kArray; }
  public: bool IsObject() const { return this->type == Type::kObject; }
  public: bool IsBool() const { return this->type == Type::kBool; }

  /// \brief Object member lookup; nullptr when absent or not an object.
  public: const Value *Find(const std::string &_key) const
  {
    if (this->type != Type::kObject)
      return nullptr;
    auto it = this->object.find(_key);
    return it == this->object.end() ? nullptr : &it->second;
  }

  /// \brief Object member lookup by dotted path ("a.b"); nullptr on miss.
  public: const Value *FindPath(const std::string &_path) const
  {
    const Value *cur = this;
    std::size_t begin = 0;
    while (begin <= _path.size())
    {
      std::size_t dot = _path.find('.', begin);
      std::string key = _path.substr(begin, dot == std::string::npos
          ? std::string::npos : dot - begin);
      cur = cur->Find(key);
      if (cur == nullptr)
        return nullptr;
      if (dot == std::string::npos)
        return cur;
      begin = dot + 1;
    }
    return cur;
  }

  /// \brief Number at _key, or _default when absent/not a number.
  public: double NumberOr(const std::string &_key, double _default) const
  {
    const Value *v = this->Find(_key);
    return (v != nullptr && v->IsNumber()) ? v->number : _default;
  }

  /// \brief Number at _key coerced from bool/string as a convenience.
  public: double NumberOrCoerce(const std::string &_key, double _default) const
  {
    const Value *v = this->Find(_key);
    if (v == nullptr)
      return _default;
    if (v->IsNumber())
      return v->number;
    if (v->IsBool())
      return v->boolean ? 1.0 : 0.0;
    if (v->IsString())
    {
      try
      {
        std::size_t used = 0;
        double d = std::stod(v->string, &used);
        return used > 0 ? d : _default;
      }
      catch (...)
      {
        return _default;
      }
    }
    return _default;
  }

  public: bool BoolOr(const std::string &_key, bool _default) const
  {
    const Value *v = this->Find(_key);
    return (v != nullptr && v->IsBool()) ? v->boolean : _default;
  }

  public: std::string StringOr(const std::string &_key,
      const std::string &_default) const
  {
    const Value *v = this->Find(_key);
    return (v != nullptr && v->IsString()) ? v->string : _default;
  }
};

namespace detail
{

/// \brief Recursive-descent parser state.
class Parser
{
  public: explicit Parser(const std::string &_text) : text(_text) {}

  public: bool Parse(Value &_out, std::string &_err)
  {
    this->SkipWs();
    if (!this->ParseValue(_out, 0))
    {
      _err = this->error.empty() ? "invalid JSON" : this->error;
      return false;
    }
    this->SkipWs();
    if (this->pos != this->text.size())
    {
      this->Fail("trailing characters after JSON value");
      _err = this->error;
      return false;
    }
    return true;
  }

  private: void Fail(const std::string &_why)
  {
    if (this->error.empty())
    {
      this->error = _why + " at offset " + std::to_string(this->pos);
    }
  }

  private: void SkipWs()
  {
    while (this->pos < this->text.size() &&
           std::isspace(static_cast<unsigned char>(this->text[this->pos])))
    {
      ++this->pos;
    }
  }

  private: bool Eat(char _c)
  {
    if (this->pos < this->text.size() && this->text[this->pos] == _c)
    {
      ++this->pos;
      return true;
    }
    return false;
  }

  private: bool ParseValue(Value &_out, int _depth)
  {
    if (_depth > 64)
    {
      this->Fail("JSON nesting too deep");
      return false;
    }
    if (this->pos >= this->text.size())
    {
      this->Fail("unexpected end of JSON");
      return false;
    }
    const char c = this->text[this->pos];
    switch (c)
    {
      case '{': return this->ParseObject(_out, _depth);
      case '[': return this->ParseArray(_out, _depth);
      case '"':
      {
        _out.type = Value::Type::kString;
        return this->ParseString(_out.string);
      }
      case 't':
        if (this->text.compare(this->pos, 4, "true") == 0)
        {
          this->pos += 4;
          _out.type = Value::Type::kBool;
          _out.boolean = true;
          return true;
        }
        this->Fail("invalid literal (expected true)");
        return false;
      case 'f':
        if (this->text.compare(this->pos, 5, "false") == 0)
        {
          this->pos += 5;
          _out.type = Value::Type::kBool;
          _out.boolean = false;
          return true;
        }
        this->Fail("invalid literal (expected false)");
        return false;
      case 'n':
        if (this->text.compare(this->pos, 4, "null") == 0)
        {
          this->pos += 4;
          _out.type = Value::Type::kNull;
          return true;
        }
        this->Fail("invalid literal (expected null)");
        return false;
      default:
        return this->ParseNumber(_out);
    }
  }

  private: bool ParseObject(Value &_out, int _depth)
  {
    _out.type = Value::Type::kObject;
    _out.object.clear();
    if (!this->Eat('{'))
    {
      this->Fail("expected '{'");
      return false;
    }
    this->SkipWs();
    if (this->Eat('}'))
      return true;
    while (true)
    {
      this->SkipWs();
      if (this->pos >= this->text.size() || this->text[this->pos] != '"')
      {
        this->Fail("expected object key string");
        return false;
      }
      std::string key;
      if (!this->ParseString(key))
        return false;
      this->SkipWs();
      if (!this->Eat(':'))
      {
        this->Fail("expected ':' after object key");
        return false;
      }
      this->SkipWs();
      Value member;
      if (!this->ParseValue(member, _depth + 1))
        return false;
      _out.object[key] = std::move(member);
      this->SkipWs();
      if (this->Eat(','))
        continue;
      if (this->Eat('}'))
        return true;
      this->Fail("expected ',' or '}' in object");
      return false;
    }
  }

  private: bool ParseArray(Value &_out, int _depth)
  {
    _out.type = Value::Type::kArray;
    _out.array.clear();
    if (!this->Eat('['))
    {
      this->Fail("expected '['");
      return false;
    }
    this->SkipWs();
    if (this->Eat(']'))
      return true;
    while (true)
    {
      this->SkipWs();
      Value element;
      if (!this->ParseValue(element, _depth + 1))
        return false;
      _out.array.push_back(std::move(element));
      this->SkipWs();
      if (this->Eat(','))
        continue;
      if (this->Eat(']'))
        return true;
      this->Fail("expected ',' or ']' in array");
      return false;
    }
  }

  private: bool ParseString(std::string &_out)
  {
    _out.clear();
    if (!this->Eat('"'))
    {
      this->Fail("expected string");
      return false;
    }
    while (this->pos < this->text.size())
    {
      const unsigned char c =
          static_cast<unsigned char>(this->text[this->pos++]);
      if (c == '"')
        return true;
      if (c == '\\')
      {
        if (this->pos >= this->text.size())
        {
          this->Fail("unterminated escape in string");
          return false;
        }
        const char e = this->text[this->pos++];
        switch (e)
        {
          case '"': _out.push_back('"'); break;
          case '\\': _out.push_back('\\'); break;
          case '/': _out.push_back('/'); break;
          case 'b': _out.push_back('\b'); break;
          case 'f': _out.push_back('\f'); break;
          case 'n': _out.push_back('\n'); break;
          case 'r': _out.push_back('\r'); break;
          case 't': _out.push_back('\t'); break;
          case 'u':
          {
            unsigned int cp = 0;
            if (!this->ParseHex4(cp))
              return false;
            // Handle UTF-16 surrogate pairs.
            if (cp >= 0xD800 && cp <= 0xDBFF &&
                this->pos + 1 < this->text.size() &&
                this->text[this->pos] == '\\' &&
                this->text[this->pos + 1] == 'u')
            {
              this->pos += 2;
              unsigned int low = 0;
              if (!this->ParseHex4(low))
                return false;
              if (low >= 0xDC00 && low <= 0xDFFF)
              {
                cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
              }
            }
            this->AppendUtf8(_out, cp);
            break;
          }
          default:
            this->Fail("invalid escape sequence in string");
            return false;
        }
      }
      else if (c < 0x20)
      {
        this->Fail("unescaped control character in string");
        return false;
      }
      else
      {
        _out.push_back(static_cast<char>(c));
      }
    }
    this->Fail("unterminated string");
    return false;
  }

  private: bool ParseHex4(unsigned int &_cp)
  {
    if (this->pos + 4 > this->text.size())
    {
      this->Fail("truncated \\u escape");
      return false;
    }
    unsigned int v = 0;
    for (int i = 0; i < 4; ++i)
    {
      const char h = this->text[this->pos++];
      v <<= 4;
      if (h >= '0' && h <= '9')
        v |= static_cast<unsigned int>(h - '0');
      else if (h >= 'a' && h <= 'f')
        v |= static_cast<unsigned int>(h - 'a' + 10);
      else if (h >= 'A' && h <= 'F')
        v |= static_cast<unsigned int>(h - 'A' + 10);
      else
      {
        this->Fail("invalid hex digit in \\u escape");
        return false;
      }
    }
    _cp = v;
    return true;
  }

  private: static void AppendUtf8(std::string &_out, unsigned int _cp)
  {
    if (_cp <= 0x7F)
    {
      _out.push_back(static_cast<char>(_cp));
    }
    else if (_cp <= 0x7FF)
    {
      _out.push_back(static_cast<char>(0xC0 | (_cp >> 6)));
      _out.push_back(static_cast<char>(0x80 | (_cp & 0x3F)));
    }
    else if (_cp <= 0xFFFF)
    {
      _out.push_back(static_cast<char>(0xE0 | (_cp >> 12)));
      _out.push_back(static_cast<char>(0x80 | ((_cp >> 6) & 0x3F)));
      _out.push_back(static_cast<char>(0x80 | (_cp & 0x3F)));
    }
    else
    {
      _out.push_back(static_cast<char>(0xF0 | (_cp >> 18)));
      _out.push_back(static_cast<char>(0x80 | ((_cp >> 12) & 0x3F)));
      _out.push_back(static_cast<char>(0x80 | ((_cp >> 6) & 0x3F)));
      _out.push_back(static_cast<char>(0x80 | (_cp & 0x3F)));
    }
  }

  private: bool ParseNumber(Value &_out)
  {
    const std::size_t start = this->pos;
    if (this->pos < this->text.size() &&
        (this->text[this->pos] == '-' || this->text[this->pos] == '+'))
    {
      ++this->pos;
    }
    bool digits = false;
    while (this->pos < this->text.size() &&
           std::isdigit(static_cast<unsigned char>(this->text[this->pos])))
    {
      ++this->pos;
      digits = true;
    }
    if (this->pos < this->text.size() && this->text[this->pos] == '.')
    {
      ++this->pos;
      while (this->pos < this->text.size() &&
             std::isdigit(static_cast<unsigned char>(this->text[this->pos])))
      {
        ++this->pos;
        digits = true;
      }
    }
    if (digits && this->pos < this->text.size() &&
        (this->text[this->pos] == 'e' || this->text[this->pos] == 'E'))
    {
      ++this->pos;
      if (this->pos < this->text.size() &&
          (this->text[this->pos] == '-' || this->text[this->pos] == '+'))
      {
        ++this->pos;
      }
      bool expDigits = false;
      while (this->pos < this->text.size() &&
             std::isdigit(static_cast<unsigned char>(this->text[this->pos])))
      {
        ++this->pos;
        expDigits = true;
      }
      if (!expDigits)
      {
        this->Fail("malformed number exponent");
        return false;
      }
    }
    if (!digits)
    {
      this->Fail("invalid JSON token");
      return false;
    }
    const std::string token = this->text.substr(start, this->pos - start);
    char *end = nullptr;
    const double d = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0')
    {
      this->Fail("malformed number");
      return false;
    }
    _out.type = Value::Type::kNumber;
    _out.number = d;
    return true;
  }

  private: const std::string &text;
  private: std::size_t pos = 0;
  private: std::string error;
};

}  // namespace detail

/// \brief Parse a JSON document from text. Never throws.
/// \param[in] _text UTF-8 JSON text.
/// \param[out] _out Parsed document (only valid when true is returned).
/// \param[out] _error Human readable error description.
/// \return true on success.
inline bool Parse(const std::string &_text, Value &_out, std::string &_error)
{
  detail::Parser parser(_text);
  return parser.Parse(_out, _error);
}

/// \brief Escape a string for embedding in a JSON document.
inline std::string Escape(const std::string &_in)
{
  std::string out;
  out.reserve(_in.size() + 8);
  for (const char ch : _in)
  {
    const unsigned char c = static_cast<unsigned char>(ch);
    switch (c)
    {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20)
        {
          const char hex[] = "0123456789abcdef";
          out += "\\u00";
          out.push_back(hex[(c >> 4) & 0xF]);
          out.push_back(hex[c & 0xF]);
        }
        else
        {
          out.push_back(ch);
        }
    }
  }
  return out;
}

}  // namespace json
}  // namespace robomini

#endif  // ROBOMINI_JSON_LITE_HH_
