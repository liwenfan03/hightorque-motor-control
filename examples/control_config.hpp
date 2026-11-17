// Config file for `motor_tool control` (config/control.conf): INI-style sections, key = value.
//
//   [motors]          <id> = <model>
//   [common]          loop-wide and per-motor defaults for every mode
//   [common.<id>]     ... for one motor
//   [<mode>]          position / velocity / torque / impedance
//   [<mode>.<id>]     ... for one motor
//
// Lookup for a per-motor value: [<mode>.<id>], [<mode>], [common.<id>], [common].  The command line
// (--key value) always wins; that part is done by the caller.  '#' starts a comment.
#pragma once

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace cli {

class ControlConfig {
 public:
  ControlConfig() = default;

  // Throws std::runtime_error with the file name and line on a syntax error or an unknown key.
  explicit ControlConfig(const std::string& path) : path_(path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot read config file " + path);
    std::string line, section;
    int lineno = 0;
    while (std::getline(f, line)) {
      lineno++;
      const auto hash = line.find('#');
      if (hash != std::string::npos) line.erase(hash);
      line = trim(line);
      if (line.empty()) continue;
      auto where = [&] { return path + ":" + std::to_string(lineno) + ": "; };
      if (line.front() == '[') {
        if (line.back() != ']') throw std::runtime_error(where() + "expected [section]");
        section = trim(line.substr(1, line.size() - 2));
        const std::string base = section.substr(0, section.find('.'));
        if (!kSections.count(base)) throw std::runtime_error(where() + "unknown section [" + section + "]");
        if (section.find('.') != std::string::npos && !is_id(section.substr(section.find('.') + 1))) {
          throw std::runtime_error(where() + "expected [" + base + ".<motor id>]");
        }
        if (base == "motors" && section != "motors") throw std::runtime_error(where() + "use [motors]");
        continue;
      }
      const auto eq = line.find('=');
      if (eq == std::string::npos) throw std::runtime_error(where() + "expected key = value");
      const std::string key = trim(line.substr(0, eq)), value = trim(line.substr(eq + 1));
      if (section.empty()) throw std::runtime_error(where() + "key outside a [section]");
      if (section == "motors") {
        if (!is_id(key)) throw std::runtime_error(where() + "[motors] keys are motor IDs");
      } else if (!kKeys.count(key)) {
        throw std::runtime_error(where() + "unknown key '" + key + "'");
      }
      values_[section][key] = value;
    }
  }

  const std::string& path() const { return path_; }

  // Per-motor lookup (without the command line).
  std::optional<std::string> get(const std::string& mode, int id, const std::string& key) const {
    const std::string sid = std::to_string(id);
    for (const std::string& s : {mode + "." + sid, mode, "common." + sid, std::string("common")}) {
      if (auto v = find(s, key)) return v;
    }
    return std::nullopt;
  }
  // Loop-wide lookup: [<mode>], then [common].
  std::optional<std::string> get(const std::string& mode, const std::string& key) const {
    if (auto v = find(mode, key)) return v;
    return find("common", key);
  }
  std::optional<std::string> model(int id) const { return find("motors", std::to_string(id)); }

 private:
  static std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) a++;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) b--;
    return s.substr(a, b - a);
  }
  static bool is_id(const std::string& s) {
    if (s.empty() || s.size() > 3) return false;
    for (char c : s) {
      if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    return std::atoi(s.c_str()) <= 127;
  }
  std::optional<std::string> find(const std::string& section, const std::string& key) const {
    const auto s = values_.find(section);
    if (s == values_.end()) return std::nullopt;
    const auto k = s->second.find(key);
    if (k == s->second.end()) return std::nullopt;
    return k->second;
  }

  inline static const std::set<std::string> kSections = {"motors", "common", "position", "velocity", "torque",
                                                         "impedance"};
  inline static const std::set<std::string> kKeys = {
      "rate", "duration", "ramp", "safe-vel", "max-start-deg", "max-missed", "resend-every", "max-stale", "max-outstanding", "warmup", "shape", "pipeline", "brake", "tqe-frame", "profile", "bias", "amp", "freq",
      "kp", "kd", "max-tqe", "gain-ramp", "vel-ramp",
      "vel", "ff-vel", "tqe", "kp-scale", "kd-scale", "stop-pos", "vel-max", "acc"};

  std::string path_;
  std::map<std::string, std::map<std::string, std::string>> values_;
};

}  // namespace cli
