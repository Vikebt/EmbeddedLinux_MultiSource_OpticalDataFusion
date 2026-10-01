#include "reconnect_backoff.h"

#include <cassert>
#include <chrono>

int main()
{
    using namespace std::chrono;
    ReconnectBackoff backoff(milliseconds(250), milliseconds(2000));
    assert(backoff.nextDelay() == milliseconds(250));
    assert(backoff.nextDelay() == milliseconds(500));
    assert(backoff.nextDelay() == milliseconds(1000));
    assert(backoff.nextDelay() == milliseconds(2000));
    assert(backoff.nextDelay() == milliseconds(2000));
    backoff.reset();
    assert(backoff.nextDelay() == milliseconds(250));
    return 0;
}
