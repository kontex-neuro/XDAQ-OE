#include "DeviceThread.h"

#include <fmt/format.h>

#include <algorithm>
#include <chrono>

#include "DeviceEditor.h"

using namespace RhythmNode;


DeviceThread::DeviceThread(SourceNode *sn) : DataThread(sn)
{
    sourceBuffers.add(new DataBuffer(1, 8192));
}

DeviceThread::~DeviceThread() {}

void DeviceThread::initialize(bool signalChainIsLoading) {}

std::unique_ptr<GenericEditor> DeviceThread::createEditor(SourceNode *sn)
{
    return std::make_unique<DeviceEditor>(sn, this);
}

void DeviceThread::handleBroadcastMessage(String msg) {}

void DeviceThread::updateSettings(OwnedArray<ContinuousChannel> *continuousChannels,
                                  OwnedArray<EventChannel> *eventChannels,
                                  OwnedArray<SpikeChannel> *spikeChannels,
                                  OwnedArray<DataStream> *sourceStreams,
                                  OwnedArray<DeviceInfo> *devices,
                                  OwnedArray<ConfigurationObject> *configurationObjects)
{
    continuousChannels->clear();
    eventChannels->clear();
    spikeChannels->clear();
    sourceStreams->clear();
    devices->clear();
    configurationObjects->clear();

    // create device
    // CODE GOES HERE

    DataStream::Settings dataStreamSettings{
        "Rhythm Data", "Continuous and event data from a device running Rhythm FPGA firmware",
        "rhythm-fpga-device.data",

        static_cast<float>(30000)

    };

    DataStream *stream = new DataStream(dataStreamSettings);

    sourceStreams->add(stream);

    int hsIndex = -1;
    int cc = 0;
    for (int ch = 0; ch < num_channels; ch++) {
        // if (headstage.getHalfChannels() && ch >= 16) continue;

        ContinuousChannel::Settings channelSettings{ContinuousChannel::ELECTRODE,
                                                    // headstage.getChannelName(ch),
                                                    "C" + std::to_string(cc++),
                                                    "Headstage channel from a Rhythm FPGA device",
                                                    "rhythm-fpga-device.continuous.headstage",

                                                    0.195,

                                                    stream};

        continuousChannels->add(new ContinuousChannel(channelSettings));
        continuousChannels->getLast()->setUnits("uV");
    }
}

bool DeviceThread::startAcquisition()
{
    sourceBuffers[0]->resize(num_channels, 8192);
    output_buffer.resize(num_channels * num_samples);
    ttl.resize(num_samples, 0);
    tsd.resize(num_samples, 0);
    ts.resize(num_samples);
    time_idx = 0;
    startThread();

    return true;
}

bool DeviceThread::stopAcquisition()
{
    if (isThreadRunning()) {
        signalThreadShouldExit();
    }

    waitForThreadToExit(500);

    output_buffer.clear();
    sourceBuffers[0]->clear();
    return true;
}

bool DeviceThread::updateBuffer()
{
    using Clock = std::chrono::high_resolution_clock;

    auto start = Clock::now();
    if (start - last_update_time < std::chrono::microseconds(num_samples * 100 / 3)) {
        return true;
    }

    last_update_time = start;

    for (int i = 0; i < num_samples; ++i) {
        ts[i] = time_idx++;
    }

    auto begin = output_buffer.begin();
    if (modified_layout) {
        for (int i = 0; i < num_samples; ++i) *begin++ = std::sin(2 * M_PI * ts[i] / 300.0) * 2000;
        for (int c = 0; c < num_channels - 1; ++c) {
            begin = std::copy(output_buffer.begin(), output_buffer.begin() + num_samples, begin);
        }
    } else {
        for (int i = 0; i < num_samples; ++i) {
            for (int c = 0; c < num_channels; ++c) {
                *begin++ = std::sin(2 * M_PI * ts[i] / 300.0) * 2000;
            }
        }
    }

    auto start_time = Clock::now();
    sourceBuffers[0]->addToBuffer(&output_buffer[0], &ts[0], &tsd[0], &ttl[0], num_samples,
                                  chunk_size);
    const auto passed = Clock::now() - start_time;

    if (ts[0] % 30000 < num_samples) accumulated_time = Clock::duration::zero();

    accumulated_time += passed;

    if (ts[0] % 30000 > (30000 - 1 - num_samples)) {
        const auto avg =
            std::chrono::duration_cast<std::chrono::nanoseconds>(accumulated_time).count() /
            30000.0;
        if (avg > 10000) {
            fmt::print("addToBuffer took {:8.1f} us/sample, {:.3f}%\n", avg / 1000,
                       3 * avg / 100000);
        } else {
            fmt::print("addToBuffer took {:8.1f} ns/sample, {:.3f}%\n", avg, 3 * avg / 100000);
        }
    }

    return true;
}
