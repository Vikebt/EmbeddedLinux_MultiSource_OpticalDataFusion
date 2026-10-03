#pragma once

#include "data_struct.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace navigation_wire {

// The existing sender uses 28 native-endian IEEE-754 doubles, not data_recv's
// packed in-memory layout. This is a length-checked decoder for that protocol.
constexpr std::size_t kSampleCount = 28;
constexpr std::size_t kDatagramBytes = kSampleCount * sizeof(double);
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
              "navigation datagram requires 64-bit IEEE-754 doubles");

inline bool decode(const double* values, std::size_t bytes, data_recv& output)
{
    if (values == nullptr || bytes != kDatagramBytes ||
        !std::isfinite(values[19]) ||
        values[19] < static_cast<double>(std::numeric_limits<int>::min()) ||
        values[19] > static_cast<double>(std::numeric_limits<int>::max()) ||
        !std::isfinite(values[27]) ||
        values[27] < -9223372036854775808.0 ||
        values[27] >= 9223372036854775808.0) {
        return false;
    }

    data_recv candidate{};
    candidate.time_sec = static_cast<float>(values[0]);
    candidate.INS_lon = values[1];
    candidate.INS_lat = values[2];
    candidate.INS_heig = values[3];
    candidate.INS_head = values[4];
    candidate.INS_pitch = values[5];
    candidate.INS_roll = values[6];
    candidate.INS_Fb_x = values[7];
    candidate.INS_Fb_y = values[8];
    candidate.INS_Fb_z = values[9];
    candidate.INS_Wibb_x = values[10];
    candidate.INS_Wibb_y = values[11];
    candidate.INS_WIbb_z = values[12];
    candidate.INS_ve = values[13];
    candidate.INS_vn = values[14];
    candidate.INS_vu = values[15];
    candidate.BA = static_cast<float>(values[16]);
    candidate.real_time_temp = static_cast<float>(values[17]);
    candidate.real_time_press = static_cast<float>(values[18]);
    candidate.Is_GPS_valid = static_cast<int>(values[19]);
    candidate.GPS_lon = static_cast<float>(values[20]);
    candidate.GPS_lat = static_cast<float>(values[21]);
    candidate.GPS_heig = static_cast<float>(values[22]);
    candidate.GPS_ve = static_cast<float>(values[23]);
    candidate.GPS_vn = static_cast<float>(values[24]);
    candidate.GPS_vu = static_cast<float>(values[25]);
    candidate.RA = static_cast<float>(values[26]);
    candidate.real_time = static_cast<std::int64_t>(values[27]);
    output = candidate;
    return true;
}

} // namespace navigation_wire
