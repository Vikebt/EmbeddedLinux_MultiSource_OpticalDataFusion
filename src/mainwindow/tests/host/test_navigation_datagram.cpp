#include "navigation_datagram.h"

#include <array>
#include <iostream>
#include <limits>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define CHECK(expression) do { \
    if (!(expression)) { \
        std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ << ": " #expression << '\n'; \
        return 1; \
    } \
} while (false)

int main()
{
    std::array<double, navigation_wire::kSampleCount> values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        values[i] = static_cast<double>(i);
    }
    values[19] = 1.0;
    values[27] = 123456789.0;

    data_recv output{};
    output.INS_lon = -123.0;
    CHECK(!navigation_wire::decode(values.data(),
                                   navigation_wire::kDatagramBytes - 1, output));
    CHECK(output.INS_lon == -123.0);
    CHECK(!navigation_wire::decode(values.data(),
                                   navigation_wire::kDatagramBytes + 1, output));
    CHECK(output.INS_lon == -123.0);
    CHECK(!navigation_wire::decode(nullptr,
                                   navigation_wire::kDatagramBytes, output));

    CHECK(navigation_wire::decode(values.data(),
                                  navigation_wire::kDatagramBytes, output));
    CHECK(output.time_sec == 0.0f);
    CHECK(output.INS_lon == 1.0);
    CHECK(output.INS_WIbb_z == 12.0);
    CHECK(output.INS_vu == 15.0);
    CHECK(output.Is_GPS_valid == 1);
    CHECK(output.GPS_vu == 25.0f);
    CHECK(output.RA == 26.0f);
    CHECK(output.real_time == 123456789);
    CHECK(output.real_lon == 0.0);

    values[19] = std::numeric_limits<double>::quiet_NaN();
    CHECK(!navigation_wire::decode(values.data(),
                                   navigation_wire::kDatagramBytes, output));
    CHECK(output.INS_lon == 1.0);
    values[19] = 1.0;
    values[27] = 9223372036854775808.0;
    CHECK(!navigation_wire::decode(values.data(),
                                   navigation_wire::kDatagramBytes, output));
    CHECK(output.real_time == 123456789);

    const int receiver = socket(AF_INET, SOCK_DGRAM, 0);
    const int sender = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(receiver >= 0 && sender >= 0);
    const timeval timeout{1, 0};
    CHECK(setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout)) == 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    CHECK(bind(receiver, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) == 0);
    socklen_t address_length = sizeof(address);
    CHECK(getsockname(receiver, reinterpret_cast<sockaddr*>(&address),
                      &address_length) == 0);
    std::array<unsigned char, navigation_wire::kDatagramBytes + 1> oversized{};
    CHECK(sendto(sender, oversized.data(), oversized.size(), 0,
                 reinterpret_cast<sockaddr*>(&address), sizeof(address)) ==
          static_cast<ssize_t>(oversized.size()));
    double receive_buffer[navigation_wire::kSampleCount]{};
    const ssize_t wire_bytes = recvfrom(receiver, receive_buffer,
                                        sizeof(receive_buffer), MSG_TRUNC,
                                        nullptr, nullptr);
    CHECK(wire_bytes == static_cast<ssize_t>(oversized.size()));
    CHECK(!navigation_wire::decode(receive_buffer,
                                   static_cast<std::size_t>(wire_bytes), output));
    // Without MSG_TRUNC the same oversized UDP packet looks exactly 224 bytes.
    CHECK(sendto(sender, oversized.data(), oversized.size(), 0,
                 reinterpret_cast<sockaddr*>(&address), sizeof(address)) ==
          static_cast<ssize_t>(oversized.size()));
    const ssize_t truncated_bytes = recvfrom(receiver, receive_buffer,
                                             sizeof(receive_buffer), 0,
                                             nullptr, nullptr);
    CHECK(truncated_bytes == static_cast<ssize_t>(sizeof(receive_buffer)));
    close(sender);
    close(receiver);
    return 0;
}
