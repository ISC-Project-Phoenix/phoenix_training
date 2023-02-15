#pragma once
#include "cstdint"
#include "rclcpp/time.hpp"

/// Abstraction over score algorithms
class IScoreMgr {
public:
    /// Gets the current score
    virtual uint16_t get_score() = 0;

    /// Performs an action on the score when the kart is out of the desired area
    virtual uint16_t out_of_bounds(rclcpp::Time stamp) = 0;
};

//TODO make impl of above