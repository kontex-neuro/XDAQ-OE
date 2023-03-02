#pragma once
#include <DataThreadHeaders.h>

#include <array>
#include <atomic>
#include <optional>
#include <vector>

namespace RhythmNode
{

class DeviceThread : public DataThread
{
public:
    using Clock = std::chrono::high_resolution_clock;
    /** Constructor; must specify the type of board used */
    DeviceThread(SourceNode *sn);

    /** Destructor */
    ~DeviceThread();

    /** Creates the UI for this plugin */
    std::unique_ptr<GenericEditor> createEditor(SourceNode *sn) override;

    /** Fills the DataBuffer with incoming data */
    bool updateBuffer() override;

    /** Initializes data transfer*/
    bool startAcquisition() override;

    /** Stops data transfer */
    bool stopAcquisition() override;
    bool foundInputSource() override { return true; }

    /* Passes the processor's info objects to DataThread, to allow them to be configured */
    void updateSettings(OwnedArray<ContinuousChannel> *continuousChannels,
                        OwnedArray<EventChannel> *eventChannels,
                        OwnedArray<SpikeChannel> *spikeChannels,
                        OwnedArray<DataStream> *sourceStreams, OwnedArray<DeviceInfo> *devices,
                        OwnedArray<ConfigurationObject> *configurationObjects) override;

    /** Allow the thread to respond to messages sent by other plugins */
    void handleBroadcastMessage(String msg) override;

    /** Informs the DataThread about whether to expect saved settings to be loaded*/
    void initialize(bool signalChainIsLoading) override;

    int chunk_size = 1;
    bool modified_layout = false;

private:
    const int num_channels = 1024;
    int64_t time_idx = 0;
    Clock::time_point last_update_time = Clock::now();

    std::vector<float> output_buffer;
    std::vector<uint64_t> ttl;
    std::vector<double> tsd;
    std::vector<int64_t> ts;


    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DeviceThread);
};

}  // namespace RhythmNode