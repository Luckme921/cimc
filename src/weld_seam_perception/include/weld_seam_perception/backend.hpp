#pragma once
#include <stdexcept>
#include <string>
#include <vector>

namespace weld_backend {
struct Selection {
    std::string name = "corrugated";
    std::vector<std::string> parameters;
};
inline Selection select(const std::vector<std::string>& values) {
    Selection result;
    bool seen = false;
    for (const auto& item : values) {
        if (item.rfind("pipeline.algorithm=", 0) == 0) {
            if (seen) throw std::invalid_argument("Duplicate pipeline.algorithm");
            result.name = item.substr(19);
            seen = true;
        }
    }
    if (result.name != "corrugated" && result.name != "gooseneck")
        throw std::invalid_argument("pipeline.algorithm must be corrugated or gooseneck");
    for (const auto& item : values) {
        if (item.rfind("pipeline.algorithm=", 0) == 0) continue;
        const bool goose = item.rfind("gooseneck.", 0) == 0;
        if (result.name == "corrugated" ? !goose :
            (goose || item.rfind("roi.", 0) == 0)) result.parameters.push_back(item);
    }
    return result;
}
}
