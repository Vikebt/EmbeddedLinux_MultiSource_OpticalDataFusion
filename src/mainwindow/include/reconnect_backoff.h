#ifndef RECONNECT_BACKOFF_H
#define RECONNECT_BACKOFF_H

#include <algorithm>
#include <chrono>

class ReconnectBackoff {
public:
    ReconnectBackoff(std::chrono::milliseconds initial,
                     std::chrono::milliseconds maximum)
        : initial_(initial), maximum_(maximum), current_(initial) {}

    std::chrono::milliseconds nextDelay() {
        const auto result = current_;
        current_ = std::min(maximum_, current_ * 2);
        return result;
    }

    void reset() { current_ = initial_; }

private:
    const std::chrono::milliseconds initial_;
    const std::chrono::milliseconds maximum_;
    std::chrono::milliseconds current_;
};

#endif
