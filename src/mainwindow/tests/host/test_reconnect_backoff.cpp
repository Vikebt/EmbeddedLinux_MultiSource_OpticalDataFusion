#include "reconnect_backoff.h"

#include <chrono>
#include <iostream>

#define CHECK(expression) do { \
    if (!(expression)) { \
        std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ << ": " #expression << '\n'; \
        return 1; \
    } \
} while (false)

int main()
{
    using namespace std::chrono;
    ReconnectBackoff backoff(milliseconds(250), milliseconds(2000));
    CHECK(backoff.nextDelay() == milliseconds(250));
    CHECK(backoff.nextDelay() == milliseconds(500));
    CHECK(backoff.nextDelay() == milliseconds(1000));
    CHECK(backoff.nextDelay() == milliseconds(2000));
    CHECK(backoff.nextDelay() == milliseconds(2000));
    backoff.reset();
    CHECK(backoff.nextDelay() == milliseconds(250));
    return 0;
}
