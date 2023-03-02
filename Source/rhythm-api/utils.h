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
inline std::uint64_t little2host64(const std::uint8_t *data)
{
    return (std::uint64_t) data[0] << 0 | (std::uint64_t) data[1] << 8 | (std::uint64_t) data[2] << 16 |
           (std::uint64_t) data[3] << 24 | (std::uint64_t) data[4] << 32 | (std::uint64_t) data[5] << 40 |
           (std::uint64_t) data[6] << 48 | (std::uint64_t) data[7] << 56;
}

inline std::uint32_t little2host32(const std::uint8_t *data)
{
    return (std::uint32_t) data[0] << 0 | (std::uint32_t) data[1] << 8 | (std::uint32_t) data[2] << 16 |
           (std::uint32_t) data[3] << 24;
}

inline std::uint16_t little2host16(const std::uint8_t *data)
{
    return (std::uint16_t) data[0] << 0 | (std::uint16_t) data[1] << 8;
}

}  // namespace endian
}  // namespace utils