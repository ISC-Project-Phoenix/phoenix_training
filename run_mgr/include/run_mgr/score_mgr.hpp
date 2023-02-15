#pragma once
#include "cstdint"
#include "opencv2/core.hpp"
#include "rclcpp/time.hpp"

namespace rm {
/// Abstraction over score algorithms. Implementers should be threadsafe.
class IScoreMgr {
public:
    enum class State { InBounds, OutOfBounds, FinishLine };

    /// Gets the current score.
    [[nodiscard]] virtual uint16_t get_score() = 0;

    /// Lowers the score by the passed amount. If less than 0, returns 0. Returns new score
    virtual uint16_t dec_score(uint16_t amount) = 0;

    /// Raises the score by the passed amount. Returns new score
    virtual uint16_t inc_score(uint16_t amount) = 0;

    /// Performs an action on the score when the kart is out of the desired area. Returns new score
    virtual uint16_t out_of_bounds(rclcpp::Time stamp) = 0;

    /// Determines the state of the kart based on an image. Ie. are we in bounds.
    [[nodiscard]] virtual State determine_state(const cv::Mat& img) = 0;
};

class BasicScoreMgr : public IScoreMgr {
    uint16_t score;
    std::shared_mutex mut;

    static uint16_t sat_add(uint16_t a, uint16_t b) {
        uint16_t c = a + b;
        if (c < a) /* Can only happen due to overflow */
            c = -1;
        return c;
    }

    static uint16_t sat_sub(uint16_t a, uint16_t b) {
        uint16_t c = a - b;
        if (c > a) /* Can only happen due to underflow */
            c = 0;
        return c;
    }

public:
    [[nodiscard]] uint16_t get_score() override {
        std::shared_lock lk{mut};
        return this->score;
    }

    uint16_t dec_score(uint16_t amount) override {
        std::unique_lock lk{mut};
        this->score = sat_sub(this->score, amount);

        return this->score;
    }

    uint16_t inc_score(uint16_t amount) override {
        std::unique_lock lk{mut};
        this->score = sat_add(this->score, amount);

        return this->score;
    }

    uint16_t out_of_bounds(rclcpp::Time) override {
        std::unique_lock lk{mut};
        this->score = sat_sub(this->score, 100);

        return this->score;
    }

    [[nodiscard]] State determine_state(const cv::Mat& img) override {
        //TODO analise for colors
        return State::OutOfBounds;
    }

    virtual ~BasicScoreMgr() = default;
};
}  // namespace rm