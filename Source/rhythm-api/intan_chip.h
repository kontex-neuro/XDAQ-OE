#pragma once
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <optional>
#include <vector>

namespace IntanChip
{
enum class ChipID : unsigned char { NA = 0, RHD2132 = 1, RHD2216 = 2, RHD2164 = 4, RHS2116 = 32 };

enum class ChipMISO : unsigned char { NA = 0, A = 53, B = 58 };

struct Chip {
    ChipID id = ChipID::NA;
    ChipMISO miso = ChipMISO::NA;
};

/**
 * @brief This method is highly dependent on the commands uploaded to the RHD2000 chip.
 */
template <typename T>
std::optional<Chip> parse_device_id(const T *aux)
{
    // First, check ROM registers 32-36 to verify that they hold 'INTAN', and
    // the initial chip name ROM registers 24-26 that hold 'RHD'.
    // This is just used to verify that we are getting good data over the SPI
    // communication channel.

    bool intanChipPresent =
        ((char) aux[32] == 'I' && (char) aux[33] == 'N' && (char) aux[34] == 'T' &&
         (char) aux[35] == 'A' && (char) aux[36] == 'N' && (char) aux[24] == 'R' &&
         (char) aux[25] == 'H' && (char) aux[26] == 'D');

    if (!intanChipPresent) return std::nullopt;

    unsigned char id = aux[19];  // chip ID (Register 63)
    // check if id is valid
    if (id == (unsigned char) ChipID::RHD2132) {
        return Chip{ChipID::RHD2132, ChipMISO::NA};
    } else if (id == (unsigned char) ChipID::RHD2216) {
        return Chip{ChipID::RHD2216, ChipMISO::NA};
    } else if (id == (unsigned char) ChipID::RHD2164) {
        unsigned char miso = aux[23];  // Register 59
        if (miso == (unsigned char) ChipMISO::A) {
            return Chip{ChipID::RHD2164, ChipMISO::A};
        } else if (miso == (unsigned char) ChipMISO::B) {
            return Chip{ChipID::RHD2164, ChipMISO::B};
        } else {
            return std::nullopt;
        }
        // }else if(id == (unsigned char)ChipID::RHS2116){ is not supposed to detect RHS2116
    } else {
        return std::nullopt;
    }
}

inline float amp2uV(uint16_t i) { return (i - 32768) * 0.195f; }

inline float adc2V(uint16_t i) { return (i - 32768) * 0.0003125f; }

inline float aux2V(uint16_t i) { return i * 0.0000374f; }

template <typename InIter, typename OutIter>
void vamp2uV(InIter begin, InIter end, OutIter out)
{
    std::transform(begin, end, out, amp2uV);
}

template <typename InIter, typename OutIter>
void vadc2V(InIter begin, InIter end, OutIter out)
{
    std::transform(begin, end, out, adc2V);
}

}  // namespace IntanChip