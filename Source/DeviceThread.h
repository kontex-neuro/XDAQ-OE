/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2020 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include <DataThreadHeaders.h>
#include <xdaq/device_manager.h>

#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "Headstage.h"
#include "rhythm-api/rhd2000evalboard.h"
#include "rhythm-api/rhd2000registers.h"


#define CHIP_ID_RHD2132 1
#define CHIP_ID_RHD2216 2
#define CHIP_ID_RHD2164 4
#define CHIP_ID_RHD2164_B 1000
#define REGISTER_59_MISO_A 53
#define REGISTER_59_MISO_B 58
#define RHD2132_16CH_OFFSET 8

constexpr int MAX_NUM_DATA_STREAMS_USB3 = 32;
#define MAX_NUM_CHANNELS MAX_NUM_DATA_STREAMS_USB3 * 35 + 16

namespace RhythmNode
{

class Headstage;
class ImpedanceMeter;

struct Impedances {
    std::vector<int> stream_indices;
    std::vector<std::vector<float>> magnitudes_by_stream;
    std::vector<std::vector<float>> phases_by_stream;
};

/**
    Communicates with a device running Intan's Rhythm Firmware

    @see DataThread, SourceNode
*/
class DeviceThread : public DataThread
{
    friend class ImpedanceMeter;

public:
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

    /* Passes the processor's info objects to DataThread, to allow them to be configured */
    void updateSettings(OwnedArray<ContinuousChannel> *continuousChannels,
                        OwnedArray<EventChannel> *eventChannels,
                        OwnedArray<SpikeChannel> *spikeChannels,
                        OwnedArray<DataStream> *sourceStreams, OwnedArray<DeviceInfo> *devices,
                        OwnedArray<ConfigurationObject> *configurationObjects) override;

    /** Updates the measured impedance values for each channel*/
    void update_impedances(std::optional<Impedances> impedances);

    /** Returns an array of connected headstages*/
    Array<const Headstage *> getConnectedHeadstages();

    /** Sets the method for determining channel names*/
    void setNamingScheme(ChannelNamingScheme scheme);

    /** Gets the method for determining channel names*/
    ChannelNamingScheme getNamingScheme();

    /** Allow the thread to respond to messages sent by other plugins */
    void handleBroadcastMessage(const String &msg, const int64 messageTimeMilliseconds) override;

    /** Informs the DataThread about whether to expect saved settings to be loaded*/
    void initialize(bool signalChainIsLoading) override;

    int getNumChannels();

    int getNumDataOutputs(ContinuousChannel::Type type);

    bool isHeadstageEnabled(int hsNum) const;

    int getChannelsInHeadstage(int hsNum) const;

    // for communication with SourceNode processors:
    bool foundInputSource() override;

    void scanPorts();

    void saveImpedances(File &file);

    String getChannelName(int ch) const;

    float getAdcBitVolts(int channelNum) const;

    void setSampleRate(Rhd2000EvalBoard::SampleRate index, bool temporary = false);

    double setUpperBandwidth(double upper);  // set desired BW, returns actual BW
    double setLowerBandwidth(double lower);

    double setDspCutoffFreq(double freq);
    double getDspCutoffFreq() const;

    void setDSPOffset(bool state);

    int setNoiseSlicerLevel(int level);
    void setFastTTLSettle(bool state, int channel);
    void setTTLoutputMode(bool state);
    void setDAChpf(float cutoff, bool enabled);

    void enableAuxs(bool);
    void enableAdcs(bool);

    int TTL_OUTPUT_STATE[16];

    bool isAuxEnabled();
    bool isAcquisitionActive() const;

    Array<int> getDACchannels() const;

    void setDACchannel(int dacOutput, int channel);
    void setDACthreshold(int dacOutput, float threshold);

    int getActiveChannelsInHeadstage(int hsNum) const;

    void runImpedanceTest();

    void setAdcRange(int adcChannel, short rangeType);

    short getAdcRange(int adcChannel) const;

    class DigitalOutputTimer : public Timer
    {
    public:
        /** Constructor */
        DigitalOutputTimer(DeviceThread *, int tllLine, int eventDurationMs);

        /** Destructor*/
        ~DigitalOutputTimer() {}

        /** Sends signal to turn off event channel*/
        void timerCallback();

    private:
        DeviceThread *board;

        int tllOutputLine;
    };

    struct DigitalOutputCommand {
        int ttlLine;
        bool state;
    };

    void addDigitalOutputCommand(DigitalOutputTimer *timerToDelete, int ttlLine, bool state);
    const Ports &get_ports() const { return evalBoard->ports; }
    const std::vector<Headstage> &get_headstages() const { return headstages; }

    std::string serial_number;
    std::string xdaq_model_name;

    void set_xdaq_timestamp(bool use_xdaq_timestamp)
    {
        this->use_xdaq_timestamp = use_xdaq_timestamp;
    }

private:
    std::optional<std::unique_ptr<xdaq::DeviceManager::OwnedDevice::element_type::DataStream>>
        stream;

    std::queue<DigitalOutputCommand> digitalOutputCommands;

    OwnedArray<DigitalOutputTimer> digitalOutputTimers;

    // void setCableLength(int hsNum, float length);

    std::unique_ptr<Rhd2000EvalBoard> evalBoard;
    Rhd2000Registers chipRegisters;

    /** Custom classes*/
    std::vector<Headstage> headstages;
    ScopedPointer<ImpedanceMeter> impedanceThread;

    /** True if device is available*/
    bool deviceFound;

    /** True if data is streaming*/
    std::atomic_bool isTransmitting;

    /** True if change in settings is needed during acquisition*/
    bool updateSettingsDuringAcquisition = false;

    // aux inputs are only sampled every 4th sample, so use this to buffer the
    // samples so they can be handles just like the regular neural channels later
    float auxBuffer[MAX_NUM_CHANNELS] = {0};


    /** Cable length settings */
    struct CableLength {
        float portA = 0.914f;
        float portB = 0.914f;
        float portC = 0.914f;
        float portD = 0.914f;
        float portE = 0.914f;
        float portF = 0.914f;
        float portG = 0.914f;
        float portH = 0.914f;
    };

    /** Dsp settings*/
    struct Dsp {
        bool enabled = true;
        double cutoffFreq = 0.5;
        double upperBandwidth = 7500.0f;
        double lowerBandwidth = 1.0f;
    };

    /** struct containing board settings*/
    struct Settings {
        bool acquireAux = false;
        bool acquireAdc = false;

        bool fastSettleEnabled = false;
        bool fastTTLSettleEnabled = false;
        int fastSettleTTLChannel = -1;
        bool ttlMode = false;

        Dsp dsp;

        int noiseSlicerLevel;

        bool desiredDAChpfState;
        double desiredDAChpf;
        float boardSampleRate = 30000.f;
        Rhd2000EvalBoard::SampleRate savedSampleRate;

        CableLength cableLength;

        int audioOutputL = -1;
        int audioOutputR = -1;
        bool ledsEnabled = true;
        bool newScan = true;
        int numberingScheme = 1;
        uint16 clockDivideFactor;

    } settings;

    /** Open the connection to the acquisition board*/
    bool openBoard(String pathToLibrary);

    /** Initialize the board*/
    void initializeBoard();

    /** Update register settings*/
    void updateRegisters();

    int dacChannels[8] = {0};
    int dacStream[8] = {0};
    float dacThresholds[8] = {0};
    bool dacChannelsToUpdate[8] = {0};

    Array<int> chipId;  // TODO: Fill this in

    ChannelNamingScheme channelNamingScheme;

    /** ADC info */
    std::array<std::atomic_short, 8> adcRangeSettings;
    Array<float> adcBitVolts;
    StringArray adcChannelNames;
    StringArray ttlLineNames;

    /** Impedance data*/
    std::optional<Impedances> impedances = std::nullopt;

    StringArray channelNames;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DeviceThread);

    bool use_xdaq_timestamp = false;
};

}  // namespace RhythmNode