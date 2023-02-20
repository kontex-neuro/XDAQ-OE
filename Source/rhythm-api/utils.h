#pragma once
#include <cstdint>
namespace utils
{
namespace endian
{

/**
 * @brief Convert little endian to host endian, mordern compiler can do a zero cost conversion on a
 * little endian machine
 */
inline uint64_t little2host64(const uint8_t *data)
{
    return (uint64_t) data[0] << 0 | (uint64_t) data[1] << 8 | (uint64_t) data[2] << 16 |
           (uint64_t) data[3] << 24 | (uint64_t) data[4] << 32 | (uint64_t) data[5] << 40 |
           (uint64_t) data[6] << 48 | (uint64_t) data[7] << 56;
}

inline uint32_t little2host32(const uint8_t *data)
{
    return (uint32_t) data[0] << 0 | (uint32_t) data[1] << 8 | (uint32_t) data[2] << 16 |
           (uint32_t) data[3] << 24;
}

inline uint16_t little2host16(const uint8_t *data)
{
    return (uint16_t) data[0] << 0 | (uint16_t) data[1] << 8;
}

}  // namespace endian
}  // namespace utils