#include "DeviceThread.h"

#include <fmt/format.h>

#include <algorithm>
#include <chrono>

#include "DeviceEditor.h"

using namespace RhythmNode;


DeviceThread::DeviceThread(SourceNode *sn) : DataThread(sn)
{
    sourceBuffers.add(new DataBuffer(num_channels, 10000));
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
    output_buffer.resize(num_channels * 300);
    ttl.resize(300, 0);
    tsd.resize(300, 0);
    ts.resize(300);
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
    const int num_samples = 300;

    auto start = Clock::now();
    if (start - last_update_time < std::chrono::milliseconds(10)) {
        return true;
    }

    last_update_time = start;

    for (int i = 0; i < num_samples; ++i) {
        ts[i] = time_idx++;
    }

    auto begin = output_buffer.begin();
    if (modified_layout) {
        for (int c = 0; c < num_channels; ++c) {
            for (int i = 0; i < num_samples; ++i) {
                *begin++ = std::sin(2 * M_PI * ts[i] / 300) * 2000;
            }
        }
    } else {
        for (int i = 0; i < num_samples; ++i) {
            for (int c = 0; c < num_channels; ++c) {
                *begin++ = std::sin(2 * M_PI * ts[i] / 300) * 2000;
            }
        }
    }

    auto start_time = Clock::now();
    sourceBuffers[0]->addToBuffer(&output_buffer[0], &ts[0], &tsd[0], &ttl[0], num_samples,
                                  chunk_size);
    if (ts[0] % 3000 == 0) {
        const auto passed = Clock::now() - start_time;
        if (passed > std::chrono::microseconds(100)) {
            fmt::print("addToBuffer took {} us\n",
                       std::chrono::duration_cast<std::chrono::microseconds>(passed).count());
        } else {
            fmt::print("addToBuffer took {} ns\n",
                       std::chrono::duration_cast<std::chrono::nanoseconds>(passed).count());
        }
    }

    return true;
}
