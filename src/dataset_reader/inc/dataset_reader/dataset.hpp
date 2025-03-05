#pragma once
#include <concepts>
#include <optional>
#include "dataset_reader/frame.hpp"


namespace fsa {
// C++20概念：约束数据集必须实现的方法
template <typename T>
concept DatasetConcept = requires(T ds) {
    { ds.load_next() } -> std::same_as<std::optional<Frame>>;
    { ds.has_next() } -> std::same_as<bool>;
};

class Dataset {
public:
    virtual ~Dataset() = default;
    virtual std::optional<Frame> load_next() = 0;
    virtual bool has_next() const = 0;
    
    // 可扩展方法
    virtual void seek_to(uint64_t index) { /* 默认实现 */ }
    virtual void set_playback_speed(double speed) { /* ... */ }
};
}
