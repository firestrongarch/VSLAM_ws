#pragma once
#include <memory>
#include <unordered_map>
#include <functional>
#include "dataset_reader/dataset.hpp"
class DatasetFactory {
public:
    using Creator = std::function<std::unique_ptr<Dataset>(const std::string&)>;
    
    template <DatasetConcept T>
    static void register_type(const std::string& name) {
        creators_[name] = [](const std::string& path) {
            return std::make_unique<T>(path);
        };
    }
    
    static std::unique_ptr<Dataset> create(
        const std::string& type, 
        const std::string& path
    ) {
        return creators_.at(type)(path);
    }

private:
    static inline std::unordered_map<std::string, Creator> creators_;
};