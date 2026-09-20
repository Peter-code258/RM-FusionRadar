// config_loader.h - Simple YAML-style configuration loader (key: value pairs)
#pragma once

#include <string>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rm_radar {

class Config {
public:
    bool load(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return false;
        std::string line;
        while (std::getline(f, line)) {
            // Remove comments
            auto pos = line.find('#');
            if (pos != std::string::npos) line = line.substr(0, pos);
            // Trim
            size_t s = 0, e = line.size();
            while (s < e && (line[s] == ' ' || line[s] == '\t')) ++s;
            while (e > s && (line[e-1] == ' ' || line[e-1] == '\t' || line[e-1] == '\r')) --e;
            line = line.substr(s, e - s);
            if (line.empty()) continue;

            auto colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string key = line.substr(0, colon);
            std::string val = line.substr(colon + 1);
            // Trim key and val
            auto trim = [](std::string& t) {
                size_t a = 0, b = t.size();
                while (a < b && (t[a] == ' ' || t[a] == '\t')) ++a;
                while (b > a && (t[b-1] == ' ' || t[b-1] == '\t')) --b;
                t = t.substr(a, b - a);
            };
            trim(key); trim(val);
            data_[key] = val;
        }
        return true;
    }

    std::string get(const std::string& key, const std::string& def = "") const {
        auto it = data_.find(key);
        return it != data_.end() ? it->second : def;
    }

    float getFloat(const std::string& key, float def = 0.0f) const {
        auto it = data_.find(key);
        if (it == data_.end()) return def;
        try { return std::stof(it->second); } catch (...) { return def; }
    }

    int getInt(const std::string& key, int def = 0) const {
        auto it = data_.find(key);
        if (it == data_.end()) return def;
        try { return std::stoi(it->second); } catch (...) { return def; }
    }

    bool getBool(const std::string& key, bool def = false) const {
        auto it = data_.find(key);
        if (it == data_.end()) return def;
        return it->second == "true" || it->second == "1" || it->second == "yes";
    }

    void set(const std::string& key, const std::string& val) { data_[key] = val; }

private:
    std::unordered_map<std::string, std::string> data_;
};

} // namespace rm_radar
