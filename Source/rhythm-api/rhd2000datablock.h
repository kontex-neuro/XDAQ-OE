#pragma once

#include <cstdint>
#include <vector>

#include "intan_chip.h"
#include "utils.h"

constexpr int SAMPLES_PER_DATA_BLOCK = 128;
constexpr int CHANNELS_PER_STREAM = 32;
constexpr std::uint64_t RHD2000_HEADER_MAGIC_NUMBER = 0xd7a22aaa38132a53;

template <typename Unit>
std::size_t sample_size(int streams, int channels_per_stream, bool device_timestamp)
{
    // magic number 8 bytes; time stamp 4 bytes;
    // (amp channels + 3 aux commands) * size of amp
    int sample_size = 8 + 4 + (streams * (channels_per_stream + 3) * sizeof(std::uint16_t));
    // 0-3 filler words; ADCs 8 * 2 bytes; TTL in/out 16+16 bits or 32+32 bits
    sample_size += ((streams + 2) % 4) * 2 + 8 * device_timestamp + 16 + 8;
    return sample_size / sizeof(Unit);
}


template <std::uint64_t Magic, auto &as_ts,
          auto &cast_ts_f,                                  // cast to desired type, OE use int64_t
          auto &as_amp, auto &cast_amp_f, auto &amp_index,  // deseralize and cast ampifier data
          auto &as_adc, auto &cast_adc_f, auto &adc_index>
class DataBlock
{
public:
    using ts_orig_t = decltype(as_ts(nullptr));
    using amp_orig_t = decltype(as_amp(nullptr));
    using adc_orig_t = decltype(as_adc(nullptr));
    using ts_t = decltype(cast_ts_f(as_ts(nullptr)));
    using amp_t = decltype(cast_amp_f(as_amp(nullptr)));
    using adc_t = decltype(cast_adc_f(as_adc(nullptr)));

    const int num_samples;
    const int num_streams;
    const int num_adc = 8;
    // const int num_aux = 3;

    std::vector<ts_t> timeStamp;
    std::vector<std::uint64_t> device_timestamp;
    std::vector<std::vector<std::uint16_t>> aux;
    std::vector<amp_t> amp;
    std::vector<adc_t> adc;
    std::vector<std::uint32_t> ttlIn;
    std::vector<std::uint32_t> ttlOut;

    DataBlock(int numDataStreams, int num_samples, const unsigned char *buffer,
              bool has_device_timestamp)
        : num_samples(num_samples),
          num_streams(numDataStreams),
          timeStamp(num_samples),
          device_timestamp(has_device_timestamp ? num_samples : 0),
          amp(numDataStreams * CHANNELS_PER_STREAM * num_samples),
          adc(num_samples * 8),
          ttlIn(num_samples),
          ttlOut(num_samples)
    {
        for (int i = 0; i < 3; ++i) aux.emplace_back(numDataStreams * num_samples);

        if (buffer != nullptr) from_buffer(buffer, has_device_timestamp);
    }

    bool from_buffer(const unsigned char *buffer, bool has_device_timestamp)
    {
        using namespace utils::endian;
        for (int t = 0; t < num_samples; ++t) {
            if (little2host64(buffer) != Magic) {
                return false;
            }
            buffer += 8;
            timeStamp[t] = cast_ts_f(as_ts(buffer));
            buffer += sizeof(ts_orig_t);
            // Read auxiliary results
            for (int channel = 0; channel < 3; ++channel) {
                for (int stream = 0; stream < num_streams; ++stream) {
                    aux[channel][stream * num_samples + t] = little2host16(buffer);
                    buffer += 2;
                }
            }
            // Read amplifier channels
            for (int channel = 0; channel < CHANNELS_PER_STREAM; ++channel) {
                for (int stream = 0; stream < num_streams; ++stream) {
                    amp[amp_index(num_samples, CHANNELS_PER_STREAM, num_streams, t, channel,
                                  stream)] = cast_amp_f(as_amp(buffer));
                    buffer += sizeof(amp_orig_t);
                }
            }
            // skip filler words in each data stream
            buffer += 2 * ((num_streams + 2) % 4);
            if (has_device_timestamp) {
                device_timestamp[t] = little2host64(buffer);
                buffer += 8;
            }
            // Read from ADCs
            for (int i = 0; i < num_adc; ++i) {
                adc[adc_index(num_samples, num_adc, t, i)] = cast_adc_f(as_adc(buffer));
                buffer += sizeof(adc_orig_t);
            }
            // Read TTL input and output values
            ttlIn[t] = little2host32(buffer);
            buffer += 4;
            ttlOut[t] = little2host32(buffer);
            buffer += 4;
        }

        return true;
    }
};

inline long long cast_ts(std::uint32_t x) { return x; }


// Original Order
inline int amp_index_time_channel_stream(int samples, int channels, int streams, int t, int c,
                                         int s)
{
    return t * channels * streams + c * streams + s;
}
// Transposed, Stream x Channel x Time
inline int amp_index_stream_channel_time(int samples, int channels, int streams, int t, int c,
                                         int s)
{
    return t + c * samples + s * samples * channels;
}

// Original Order
inline int adc_index_time_channel(int samples, int channels, int t, int c)
{
    return t * channels + c;
}
// Transposed, Channel x Time
inline int adc_index_channel_time(int samples, int channels, int t, int c)
{
    return t + c * samples;
}

// clang-format off
using Rhd2000DataBlock = DataBlock<RHD2000_HEADER_MAGIC_NUMBER,
    utils::endian::little2host32, cast_ts,
    utils::endian::little2host16, IntanChip::amp2uV, amp_index_stream_channel_time,
    utils::endian::little2host16, IntanChip::adc2V, adc_index_channel_time>;
// clang-format on