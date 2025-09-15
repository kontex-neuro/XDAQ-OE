/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2016 Open Ephys

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

#include <VisualizerEditorHeaders.h>
#include <xdaq/device.h>

#include "rhythm-api/intan_chip.h"
#include "rhythm-api/rhd2000datablock.h"
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif


#include <fmt/format.h>
#include <xdaq/data_streams.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <nlohmann/json.hpp>
#include <ranges>

#include "DeviceEditor.h"
#include "DeviceThread.h"
#include "Headstage.h"
#include "ImpedanceMeter.h"
#include "UI/DeviceSelector.h"
#include "rhythm-api/utils.h"



using json = nlohmann::json;

using namespace RhythmNode;
using namespace std::string_literals;

// #define DEBUG_EMULATE_HEADSTAGES 8
// #define DEBUG_EMULATE_64CH

constexpr int INIT_STEP = 128;

DeviceThread::DeviceThread(SourceNode *sn)
    : DataThread(sn),
      deviceFound(false),
      chipRegisters(30000.0f),
      isTransmitting(false),
      impedanceThread(new ImpedanceMeter(this)),
      channelNamingScheme(GLOBAL_INDEX)
{
    for (int i = 0; i < 8; i++) adcRangeSettings[i] = 0;

    sourceBuffers.add(new DataBuffer(2, 10000));  // start with 2 channels and automatically resize

#if defined(__APPLE__)
    File appBundle = File::getSpecialLocation(File::currentApplicationFile);
    const String executableDirectory =
        appBundle.getChildFile("Contents/Resources").getFullPathName();
#else
    File executable = File::getSpecialLocation(File::currentExecutableFile);
    const String executableDirectory = executable.getParentDirectory().getFullPathName();
#endif

    if (openBoard("")) {
        // upload bitfile and restore default settings
        initializeBoard();
        for (int i = 0; i < evalBoard->ports.max_chips; i++) {
            headstages.emplace_back(
                "Port " + std::to_string(evalBoard->ports.port_from_chip(i) + 1),
                fmt::format("P{}-{}", evalBoard->ports.port_from_chip(i) + 1,
                            i % evalBoard->ports.max_chips_per_port + 1),
                evalBoard->ports.max_streams_per_chip);
        }

        // automatically find connected headstages
        scanPorts();  // things would appear to run more smoothly if this were done after the editor
                      // has been created
        for (int k = 0; k < 8; k++) {
            dacChannelsToUpdate[k] = true;
            dacStream[k] = 0;
            setDACthreshold(k, 65534);
            dacChannels[k] = -1;
            dacThresholds[k] = 0;
        }
    }
}

DeviceThread::~DeviceThread() { LOGD("RHD2000 interface destroyed."); }

void DeviceThread::initialize(bool signalChainIsLoading) {}

std::unique_ptr<GenericEditor> DeviceThread::createEditor(SourceNode *sn)
{
    std::unique_ptr<DeviceEditor> editor = std::make_unique<DeviceEditor>(sn, this);

    return editor;
}

void DeviceThread::handleBroadcastMessage(const String &msg, const int64 messageTimeMilliseconds)
{
    StringArray parts = StringArray::fromTokens(msg, " ", "");

    // std::cout << "Received " << msg << std::endl;

    if (parts[0].equalsIgnoreCase("ACQBOARD")) {
        if (parts.size() > 1) {
            String command = parts[1];

            if (command.equalsIgnoreCase("TRIGGER")) {
                if (parts.size() == 4) {
                    int ttlLine = parts[2].getIntValue() - 1;

                    if (ttlLine < 0 || ttlLine > 7) return;

                    int eventDurationMs = parts[3].getIntValue();

                    if (eventDurationMs < 10 || eventDurationMs > 5000) return;

                    DigitalOutputCommand command;
                    command.ttlLine = ttlLine;
                    command.state = true;

                    digitalOutputCommands.push(command);

                    DigitalOutputTimer *timer =
                        new DigitalOutputTimer(this, ttlLine, eventDurationMs);

                    digitalOutputTimers.add(timer);
                }
            }
        }
    }
}


void DeviceThread::addDigitalOutputCommand(DigitalOutputTimer *timerToDelete, int ttlLine,
                                           bool state)
{
    DigitalOutputCommand command;
    command.ttlLine = ttlLine;
    command.state = state;

    digitalOutputCommands.push(command);

    digitalOutputTimers.removeObject(timerToDelete);
}

DeviceThread::DigitalOutputTimer::DigitalOutputTimer(DeviceThread *board_, int ttlLine_,
                                                     int eventDurationMs)
    : board(board_)
{
    tllOutputLine = ttlLine_;

    startTimer(eventDurationMs);
}

void DeviceThread::DigitalOutputTimer::timerCallback()
{
    stopTimer();

    board->addDigitalOutputCommand(this, tllOutputLine, false);
}

void DeviceThread::setDACthreshold(int dacOutput, float threshold)
{
    dacThresholds[dacOutput] = threshold;
    dacChannelsToUpdate[dacOutput] = true;
    updateSettingsDuringAcquisition = true;

    // evalBoard->setDacThresholdVoltage(dacOutput,threshold);
}

void DeviceThread::setDACchannel(int dacOutput, int channel)
{
    dacChannelsToUpdate[dacOutput] = true;
    updateSettingsDuringAcquisition = true;
    if (channel >= getNumDataOutputs(ContinuousChannel::ELECTRODE)) {
        dacChannels[dacOutput] = -1;
        return;
    }
    if (channel < 0) {
        dacChannels[dacOutput] = -1;
        return;
    }
    dacChannels[dacOutput] = channel % CHANNELS_PER_STREAM;
    for (int s = 0; s < evalBoard->ports.max_streams; ++s) {
        if (!evalBoard->isStreamEnabled(s)) continue;
        if (channel >= CHANNELS_PER_STREAM) {
            channel -= CHANNELS_PER_STREAM;
            continue;
        }
        dacStream[dacOutput] = s;
        return;
    }
}

Array<int> DeviceThread::getDACchannels() const
{
    Array<int> dacChannelsArray;

    for (int k = 0; k < 8; ++k) {
        dacChannelsArray.add(dacChannels[k]);
    }

    return dacChannelsArray;
}

bool DeviceThread::openBoard(String pathToLibrary)
{
    // TODO: handle no devices, adding refresh button
    // TODO: handle only one device, auto-select
    auto r = CoreServices::getDefaultUserSaveDirectory();
    auto device_options = get_device_options();
    DialogWindow::LaunchOptions options;
    options.content.setOwned(new DeviceSelector(device_options));
    options.content->setSize(472, 260);
    options.dialogTitle = "Choose XDAQ";
    options.dialogBackgroundColour = Colours::darkgrey;
    options.escapeKeyTriggersCloseButton = true;
    options.useNativeTitleBar = false;
    options.resizable = false;

    int result = options.runModal();
    if (result == 0 || result > device_options.size()) return false;
    const auto &selected_device = device_options[result - 1];
    try {
        auto device_manager = xdaq::get_device_manager(selected_device.device_manager_path);
        auto dev = device_manager->create_device(selected_device.device_config.dump());
        auto has_device_timestamp =
            selected_device.device_status.contains("Capabilities") &&
            selected_device.device_status["Capabilities"].contains("Device Timestamp") &&
            selected_device.device_status["Capabilities"]["Device Timestamp"].is_array();

        use_xdaq_timestamp = has_device_timestamp;

        evalBoard = std::make_unique<Rhd2000EvalBoard>(
            std::move(dev), has_device_timestamp,
            selected_device.device_status["Expander"].get<bool>());
    } catch (const std::exception &e) {
        AlertWindow::showMessageBox(AlertWindow::WarningIcon, "Error", e.what(), "OK");
        return false;
    }
    serial_number = selected_device.device_info["Serial Number"].get<std::string>();
    xdaq_model_name = selected_device.device_info["XDAQ Model"].get<std::string>();
    return true;
}


void DeviceThread::initializeBoard()
{
    String bitfilename;

    File sharedDir = CoreServices::getSavedStateDirectory();
    if (!sharedDir.getFullPathName().contains("plugin-GUI" + File::getSeparatorString() + "Build"))
        sharedDir = sharedDir.getChildFile("shared-api" + String(PLUGIN_API_VER));
    else
        sharedDir = sharedDir.getChildFile("shared");

    bitfilename = sharedDir.getFullPathName() + File::getSeparatorString() + "xdaq.bit";

    deviceFound = true;

    // Initialize the board
    LOGD("Initializing XDAQ.");
    evalBoard->initialize();
    // This applies the following settings:
    //  - sample rate to 30 kHz
    //  - aux command banks to zero
    //  - aux command lengths to zero
    //  - continuous run mode to 'true'
    //  - maxTimeStep to 2^32 - 1
    //  - all cable lengths to 3 feet
    //  - dspSettle to 'false'
    //  - data source mapping as 0->PortA1, 1->PortB1, 2->PortC1, 3->PortD1, etc.
    //  - enables all data streams
    //  - clears the ttlOut
    //  - disables all DACs and sets gain to 0

    setSampleRate(Rhd2000EvalBoard::SampleRate::s30000Hz);

    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortA, settings.cableLength.portA);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortB, settings.cableLength.portB);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortC, settings.cableLength.portC);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortD, settings.cableLength.portD);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortE, settings.cableLength.portE);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortF, settings.cableLength.portF);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortG, settings.cableLength.portG);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortH, settings.cableLength.portH);

    // Select RAM Bank 0 for AuxCmd3 initially, so the ADC is calibrated.
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                    Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 0);

    // Since our longest command sequence is 60 commands, run the SPI interface for
    // 60 samples (64 for usb3 power-of two needs)
    auto db = evalBoard->run_and_read_samples(INIT_STEP);
    // Now that ADC calibration has been performed, we switch to the command sequence
    // that does not execute ADC calibration.
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                    Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3,
                                    settings.fastSettleEnabled ? 2 : 1);

    adcChannelNames.clear();
    ttlLineNames.clear();

    for (int i = 0; i < 8; i++) {
        adcChannelNames.add("ADC" + String(i + 1));
        ttlLineNames.add("TTL" + String(i + 1));
    }
}

void DeviceThread::scanPorts()
{
    if (!deviceFound)  // Safety to avoid crashes if board not present
    {
        return;
    }

    impedanceThread->stopThreadSafely();

    for (auto &headstage : headstages) {
        headstage.setNumStreams(0);  // reset stream count
    }

    setSampleRate(Rhd2000EvalBoard::SampleRate::s30000Hz, true);  // set to 30 kHz temporarily

    LOGD("Checking for connected amplifier chips...");

    evalBoard->scan_chips();

    int channels_enabled = 0;
    for (int i = 0; i < headstages.size(); ++i) {
        const auto &chip = evalBoard->get_chips()[i];
        headstages[i].setFirstChannel(channels_enabled);
        if (chip.id == IntanChip::ChipID::NA) {
            headstages[i].setNumStreams(0);
        } else {
            headstages[i].setNumStreams((chip.id == IntanChip::ChipID::RHD2164) ? 2 : 1);
            evalBoard->enableDataStream(i * evalBoard->ports.max_streams_per_chip, true);
            channels_enabled += CHANNELS_PER_STREAM;
            if (chip.id == IntanChip::ChipID::RHD2164) {
                evalBoard->enableDataStream(i * evalBoard->ports.max_streams_per_chip + 1, true);
                channels_enabled += CHANNELS_PER_STREAM;
            }
        }
    }
    sourceBuffers[0]->resize(getNumChannels(), 10000);

    LOGD("Number of enabled data streams: ", evalBoard->getNumEnabledDataStreams());

    using SPIPort = Rhd2000EvalBoard::SPIPort;
    settings.cableLength.portA = evalBoard->estimate_cable_length_meters(SPIPort::PortA);
    settings.cableLength.portB = evalBoard->estimate_cable_length_meters(SPIPort::PortB);
    settings.cableLength.portC = evalBoard->estimate_cable_length_meters(SPIPort::PortC);
    settings.cableLength.portD = evalBoard->estimate_cable_length_meters(SPIPort::PortD);
    settings.cableLength.portE = evalBoard->estimate_cable_length_meters(SPIPort::PortE);
    settings.cableLength.portF = evalBoard->estimate_cable_length_meters(SPIPort::PortF);
    settings.cableLength.portG = evalBoard->estimate_cable_length_meters(SPIPort::PortG);
    settings.cableLength.portH = evalBoard->estimate_cable_length_meters(SPIPort::PortH);

    setSampleRate(settings.savedSampleRate);  // restore saved sample rate
}

void DeviceThread::updateSettings(OwnedArray<ContinuousChannel> *continuousChannels,
                                  OwnedArray<EventChannel> *eventChannels,
                                  OwnedArray<SpikeChannel> *spikeChannels,
                                  OwnedArray<DataStream> *sourceStreams,
                                  OwnedArray<DeviceInfo> *devices,
                                  OwnedArray<ConfigurationObject> *configurationObjects)
{
    if (!deviceFound) return;

    continuousChannels->clear();
    eventChannels->clear();
    spikeChannels->clear();
    sourceStreams->clear();
    devices->clear();
    configurationObjects->clear();

    channelNames.clear();

    // create device
    // CODE GOES HERE


    DataStream *stream = new DataStream({
        "Rhythm Data",
        "Continuous and event data from a device running Rhythm FPGA firmware",
        "rhythm-fpga-device.data",
        static_cast<float>(evalBoard->getSampleRate()),
        use_xdaq_timestamp,
    });
    sourceStreams->add(stream);

    int hsIndex = -1;
    int cc = 0;
    for (auto &headstage : headstages) {
        hsIndex++;

        if (headstage.isConnected()) {
            for (int ch = 0; ch < headstage.getNumChannels(); ch++) {
                // if (headstage.getHalfChannels() && ch >= 16) continue;

                ContinuousChannel::Settings channelSettings{
                    ContinuousChannel::ELECTRODE,
                    // headstage.getChannelName(ch),
                    "C" + std::to_string(cc++), "Headstage channel from a Rhythm FPGA device",
                    "rhythm-fpga-device.continuous.headstage",

                    0.195,

                    stream};

                continuousChannels->add(new ContinuousChannel(channelSettings));
                continuousChannels->getLast()->setUnits("uV");

                if (impedances) {
                    continuousChannels->getLast()->impedance.magnitude =
                        headstage.getImpedanceMagnitude(ch);
                    continuousChannels->getLast()->impedance.phase =
                        headstage.getImpedancePhase(ch);
                }
            }

            if (settings.acquireAux) {
                for (int ch = 0; ch < 3; ch++) {
                    ContinuousChannel::Settings channelSettings{
                        ContinuousChannel::AUX,
                        String(headstage.prefix) + "_A" + String(ch + 1),
                        "Aux input channel from a Rhythm FPGA device",
                        "rhythm-fpga-device.continuous.aux",
                        0.0000374,
                        stream};

                    continuousChannels->add(new ContinuousChannel(channelSettings));
                    continuousChannels->getLast()->setUnits("mV");
                }
            }
        }
    }

    if (settings.acquireAdc) {
        for (int ch = 0; ch < 8; ch++) {
            String name = "ADC" + String(ch + 1);

            ContinuousChannel::Settings channelSettings{
                ContinuousChannel::ADC,
                name,
                "ADC input channel from a Rhythm FPGA device",
                "rhythm-fpga-device.continuous.adc",

                getAdcBitVolts(ch),

                stream};

            continuousChannels->add(new ContinuousChannel(channelSettings));
            continuousChannels->getLast()->setUnits("V");
        }
    }

    EventChannel::Settings settings{EventChannel::Type::TTL,
                                    "Rhythm FPGA TTL Input",
                                    "Events on digital input lines of a Rhythm FPGA device",
                                    "rhythm-fpga-device.events",
                                    stream,
                                    32};

    eventChannels->add(new EventChannel(settings));
}

void DeviceThread::update_impedances(std::optional<Impedances> impedances)
{
    this->impedances = impedances;
    if (!impedances) return;
    LOGD("Updating headstage impedance values");
    for (int i = 0; i < impedances->stream_indices.size(); ++i) {
        const int chipidx = impedances->stream_indices[i] / evalBoard->ports.max_streams_per_chip;
        auto chip = evalBoard->get_chips()[chipidx];
        auto &hs = headstages[chipidx];
        auto mag = impedances->magnitudes_by_stream[i];
        auto phase = impedances->phases_by_stream[i];
        if (evalBoard->ports.is_ddr(impedances->stream_indices[i])) continue;
        if (chip.id == IntanChip::ChipID::RHD2164) {
            const auto &mag2 = impedances->magnitudes_by_stream[i + 1];
            mag.insert(mag.end(), mag2.begin(), mag2.end());
            const auto &phase2 = impedances->phases_by_stream[i + 1];
            phase.insert(phase.end(), phase2.begin(), phase2.end());
        }
        hs.setImpedances(std::move(mag), std::move(phase));
    }
}

void DeviceThread::saveImpedances(File &file)
{
    if (!impedances) return;
    auto xml = std::unique_ptr<XmlElement>(new XmlElement("IMPEDANCES"));

    int globalChannelNumber = -1;

    for (auto &hs : headstages) {
        XmlElement *headstageXml = new XmlElement("HEADSTAGE");
        headstageXml->setAttribute("name", hs.prefix);

        for (int ch = 0; ch < hs.getNumActiveChannels(); ch++) {
            globalChannelNumber++;

            XmlElement *channelXml = new XmlElement("CHANNEL");
            channelXml->setAttribute("name", hs.getChannelName(ch));
            channelXml->setAttribute("number", globalChannelNumber);
            channelXml->setAttribute("magnitude", hs.getImpedanceMagnitude(ch));
            channelXml->setAttribute("phase", hs.getImpedancePhase(ch));
            headstageXml->addChildElement(channelXml);
        }

        xml->addChildElement(headstageXml);
    }

    xml->writeTo(file);
}

String DeviceThread::getChannelName(int i) const { return channelNames[i]; }

bool DeviceThread::isAcquisitionActive() const { return isTransmitting; }

void DeviceThread::setNamingScheme(ChannelNamingScheme scheme)
{
    channelNamingScheme = scheme;

    for (auto &hs : headstages) {
        hs.setNamingScheme(scheme);
    }
}

ChannelNamingScheme DeviceThread::getNamingScheme() { return channelNamingScheme; }


int DeviceThread::getNumChannels()
{
    int totalChannels = getNumDataOutputs(ContinuousChannel::ELECTRODE) +
                        getNumDataOutputs(ContinuousChannel::AUX) +
                        getNumDataOutputs(ContinuousChannel::ADC);

    return totalChannels;
}

int DeviceThread::getNumDataOutputs(ContinuousChannel::Type type)
{
    if (type == ContinuousChannel::ELECTRODE) {
        int totalChannels = 0;

        for (auto &headstage : headstages) {
            if (headstage.isConnected()) {
                totalChannels += headstage.getNumActiveChannels();
            }
        }

        return totalChannels;
    }
    if (type == ContinuousChannel::AUX) {
        if (settings.acquireAux) {
            int numAuxOutputs = 0;

            for (auto &headstage : headstages) {
                if (headstage.isConnected()) {
                    numAuxOutputs += 3;
                }
            }
            return numAuxOutputs;
        } else {
            return 0;
        }
    }
    if (type == ContinuousChannel::ADC) {
        if (settings.acquireAdc) {
            return 8;
        } else {
            return 0;
        }
    }

    return 0;
}

float DeviceThread::getAdcBitVolts(int chan) const
{
    if (chan < adcBitVolts.size()) {
        return adcBitVolts[chan];
    } else {
        return 0.0000503540039;  // 3.3V / pow(2,16)
        return -1;
    }
}

double DeviceThread::setUpperBandwidth(double upper)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.upperBandwidth = upper;

    updateRegisters();

    return settings.dsp.upperBandwidth;
}

double DeviceThread::setLowerBandwidth(double lower)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.lowerBandwidth = lower;

    updateRegisters();

    return settings.dsp.lowerBandwidth;
}

double DeviceThread::setDspCutoffFreq(double freq)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.cutoffFreq = freq;

    updateRegisters();

    return settings.dsp.cutoffFreq;
}

double DeviceThread::getDspCutoffFreq() const { return settings.dsp.cutoffFreq; }

void DeviceThread::setDSPOffset(bool state)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.enabled = state;

    updateRegisters();
}

void DeviceThread::setTTLoutputMode(bool state)
{
    settings.ttlMode = state;

    updateSettingsDuringAcquisition = true;
}

void DeviceThread::setDAChpf(float cutoff, bool enabled)
{
    settings.desiredDAChpf = cutoff;

    settings.desiredDAChpfState = enabled;

    updateSettingsDuringAcquisition = true;
}

void DeviceThread::setFastTTLSettle(bool state, int channel)
{
    settings.fastTTLSettleEnabled = state;

    settings.fastSettleTTLChannel = channel;

    updateSettingsDuringAcquisition = true;
}

int DeviceThread::setNoiseSlicerLevel(int level)
{
    settings.noiseSlicerLevel = level;

    if (deviceFound) evalBoard->setAudioNoiseSuppress(settings.noiseSlicerLevel);

    // Level has been checked once before this and then is checked again in setAudioNoiseSuppress.
    // This may be overkill - maybe API should change so that the final function returns the value?

    return settings.noiseSlicerLevel;
}


bool DeviceThread::foundInputSource() { return deviceFound; }

bool DeviceThread::isHeadstageEnabled(int hsNum) const { return headstages[hsNum].isConnected(); }


int DeviceThread::getActiveChannelsInHeadstage(int hsNum) const
{
    return headstages[hsNum].getNumActiveChannels();
}

int DeviceThread::getChannelsInHeadstage(int hsNum) const
{
    return headstages[hsNum].getNumChannels();
}

/*void DeviceThread::assignAudioOut(int dacChannel, int dataChannel)
{
    if (deviceFound)
    {
        if (dacChannel == 0)
        {
            audioOutputR = dataChannel;
            dacChannels[0] = dataChannel;
        }
        else if (dacChannel == 1)
        {
            audioOutputL = dataChannel;
            dacChannels[1] = dataChannel;
        }

        updateSettingsDuringAcquisition = true; // set a flag and take care of setting wires
        // during the updateBuffer() method
        // to avoid problems
    }

}*/

void DeviceThread::enableAuxs(bool t)
{
    settings.acquireAux = t;
    sourceBuffers[0]->resize(getNumChannels(), 10000);
    updateRegisters();
}

void DeviceThread::enableAdcs(bool t)
{
    settings.acquireAdc = t;
    sourceBuffers[0]->resize(getNumChannels(), 10000);
}

bool DeviceThread::isAuxEnabled() { return settings.acquireAux; }

void DeviceThread::setSampleRate(Rhd2000EvalBoard::SampleRate sampleRate, bool isTemporary)
{
    impedanceThread->stopThreadSafely();
    if (!isTemporary) {
        settings.savedSampleRate = sampleRate;
    }

    int numUsbBlocksToRead = -1;  // placeholder - make this change the number of blocks that are
                                  // read in DeviceThread::updateBuffer()

    switch (sampleRate) {
    case Rhd2000EvalBoard::SampleRate::s1000Hz:
        numUsbBlocksToRead = 1;
        settings.boardSampleRate = 1000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s1250Hz:
        numUsbBlocksToRead = 1;
        settings.boardSampleRate = 1250.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s1500Hz:
        numUsbBlocksToRead = 1;
        settings.boardSampleRate = 1500.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s2000Hz:
        numUsbBlocksToRead = 1;
        settings.boardSampleRate = 2000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s2500Hz:
        numUsbBlocksToRead = 1;
        settings.boardSampleRate = 2500.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s3000Hz:
        numUsbBlocksToRead = 2;
        settings.boardSampleRate = 3000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s3333Hz:
        numUsbBlocksToRead = 2;
        settings.boardSampleRate = 3333.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s4000Hz:
        numUsbBlocksToRead = 2;
        settings.boardSampleRate = 4000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s5000Hz:
        numUsbBlocksToRead = 3;
        settings.boardSampleRate = 5000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s6250Hz:
        numUsbBlocksToRead = 3;
        settings.boardSampleRate = 6250.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s8000Hz:
        numUsbBlocksToRead = 4;
        settings.boardSampleRate = 8000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s10000Hz:
        numUsbBlocksToRead = 6;
        settings.boardSampleRate = 10000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s12500Hz:
        numUsbBlocksToRead = 7;
        settings.boardSampleRate = 12500.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s15000Hz:
        numUsbBlocksToRead = 8;
        settings.boardSampleRate = 15000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s20000Hz:
        numUsbBlocksToRead = 12;
        settings.boardSampleRate = 20000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s25000Hz:
        numUsbBlocksToRead = 14;
        settings.boardSampleRate = 25000.0f;
        break;
    case Rhd2000EvalBoard::SampleRate::s30000Hz:
        numUsbBlocksToRead = 16;
        settings.boardSampleRate = 30000.0f;
        break;
    }


    // Select per-channel amplifier sampling rate.
    evalBoard->setSampleRate(sampleRate);

    LOGD("Sample rate set to ", evalBoard->getSampleRate());

    // Now that we have set our sampling rate, we can set the MISO sampling delay
    // which is dependent on the sample rate.
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortA, settings.cableLength.portA);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortB, settings.cableLength.portB);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortC, settings.cableLength.portC);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortD, settings.cableLength.portD);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortE, settings.cableLength.portE);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortF, settings.cableLength.portF);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortG, settings.cableLength.portG);
    evalBoard->setCableLengthMeters(Rhd2000EvalBoard::SPIPort::PortH, settings.cableLength.portH);

    updateRegisters();
}

void DeviceThread::updateRegisters()
{
    if (!deviceFound)  // Safety to avoid crashes loading a chain with Rythm node withouth a board
    {
        return;
    }
    // Set up an RHD2000 register object using this sample rate to
    // optimize MUX-related register settings.
    chipRegisters.defineSampleRate(settings.boardSampleRate);

    // Create a command list for the AuxCmd1 slot.  This command sequence will continuously
    // update Register 3, which controls the auxiliary digital output pin on each RHD2000 chip.
    // In concert with the v1.4 Rhythm FPGA code, this permits real-time control of the digital
    // output pin on chips on each SPI port.
    chipRegisters.setDigOutLow();  // Take auxiliary output out of HiZ mode.
    {
        const auto commands = chipRegisters.createCommandListUpdateDigOut();
        evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 0);
        evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 0,
                                          commands.size() - 1);
        evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                        Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 0);
    }

    // Next, we'll create a command list for the AuxCmd2 slot.  This command sequence
    // will sample the temperature sensor and other auxiliary ADC inputs.
    {
        const auto commands = chipRegisters.createCommandListTempSensor();
        evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd2, 0);
        evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd2, 0,
                                          commands.size() - 1);
        evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                        Rhd2000EvalBoard::AuxCmdSlot::AuxCmd2, 0);
    }

    // Before generating register configuration command sequences, set amplifier
    // bandwidth paramters.
    settings.dsp.cutoffFreq = chipRegisters.setDspCutoffFreq(settings.dsp.cutoffFreq);
    settings.dsp.lowerBandwidth = chipRegisters.setLowerBandwidth(settings.dsp.lowerBandwidth);
    settings.dsp.upperBandwidth = chipRegisters.setUpperBandwidth(settings.dsp.upperBandwidth);
    chipRegisters.enableDsp(settings.dsp.enabled);

    // enable/disable aux inputs:
    chipRegisters.enableAux1(settings.acquireAux);
    chipRegisters.enableAux2(settings.acquireAux);
    chipRegisters.enableAux3(settings.acquireAux);

    // Upload version with ADC calibration to AuxCmd3 RAM Bank 0.
    {
        const auto commands = chipRegisters.createCommandListRegisterConfig(true);
        evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 0);
        evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 0,
                                          commands.size() - 1);
    }
    // Upload version with no ADC calibration to AuxCmd3 RAM Bank 1.
    {
        const auto commands = chipRegisters.createCommandListRegisterConfig(false);
        evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 1);
        evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 0,
                                          commands.size() - 1);
    }

    // Upload version with fast settle enabled to AuxCmd3 RAM Bank 2.
    chipRegisters.setFastSettle(true);
    {
        const auto commands = chipRegisters.createCommandListRegisterConfig(false);
        evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 2);
        evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 0,
                                          commands.size() - 1);
    }
    chipRegisters.setFastSettle(false);

    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                    Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3,
                                    settings.fastSettleEnabled ? 2 : 1);
}

// void DeviceThread::setCableLength(int hsNum, float length)
// {
//     // Set the MISO sampling delay, which is dependent on the sample rate.
//
//     switch (hsNum)
//     {
//         case 0:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortA, length);
//             break;
//         case 1:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortB, length);
//             break;
//         case 2:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortC, length);
//             break;
//         case 3:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortD, length);
//             break;
//         case 4:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortE, length);
//             break;
//         case 5:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortF, length);
//             break;
//         case 6:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortG, length);
//             break;
//         case 7:
//             evalBoard->setCableLengthFeet(Rhd2000EvalBoard::PortH, length);
//             break;
//         default:
//             break;
//     }
//
// }

bool DeviceThread::startAcquisition()
{
    if (!deviceFound || (getNumChannels() == 0)) return false;

    impedanceThread->waitSafely();

    LOGD("Expecting ", getNumChannels(), " channels.");

    // reset TTL output state
    for (int k = 0; k < 16; k++) {
        TTL_OUTPUT_STATE[k] = 0;
    }

    evalBoard->flush();
    evalBoard->setContinuousRunMode(true);


    startThread();

    isTransmitting = true;
    const auto sample_size = evalBoard->get_sample_size<char>();

    constexpr int hw_events_per_sec = 100;
    const auto expected_data_rate = sample_size * evalBoard->getSampleRate();
    const int chunk_size = expected_data_rate / hw_events_per_sec;

    std::vector<bool> isddrstream;
    int nonddr = 0;
    for (int i = 0; i < evalBoard->ports.max_streams; ++i) {
        if (!evalBoard->isStreamEnabled(i)) continue;
        bool ddr = evalBoard->ports.is_ddr(i);
        isddrstream.push_back(ddr);
        nonddr += !ddr;
    }
    isddrstream.push_back(false);

    const int current_aquisition_channels =
        (32 * evalBoard->getNumEnabledDataStreams() + nonddr * 3 * settings.acquireAux +
         +settings.acquireAdc * evalBoard->ports.num_of_adc);

    using namespace xdaq::DataStream;
    using namespace utils::endian;
    fmt::print("222 Sample size: {}, {}\n", sample_size, use_xdaq_timestamp);


    auto aligned_cb = xdaq::DataStream::aligned_read_stream(
        [output_buffer = std::vector<float>(current_aquisition_channels * 1), isddrstream,
         sample_size, streams = evalBoard->getNumEnabledDataStreams(),
         aux_buffer = std::array<float, 32 * 3>(), this,
         use_xdaq_timestamp = this->use_xdaq_timestamp](auto &&event) mutable {
            std::visit(
                [&](auto &&event) {
                    using T = std::decay_t<decltype(event)>;
                    using namespace xdaq::DataStream::Events;
                    if constexpr (std::is_same_v<T, Stop>) {
                    } else if constexpr (std::is_same_v<T, Error>) {
                        LOGE(fmt::format("Datastream Error {}", event.error));
                    } else if constexpr (std::is_same_v<T, DataView>) {
                        for (int off = 0; off < event.data.size(); off += sample_size) {
                            auto data = event.data.subspan(off, sample_size);
                            if (little2host64(data.data()) != RHD2000_HEADER_MAGIC_NUMBER) {
                                LOGE("Failed to parse data");
                                return;
                            }
                            juce::int64 ts = little2host32(data.data() + 8);
                            auto target = output_buffer.begin();
                            const auto amp = data.begin() + 12;
                            for (int s = 0; s < streams; ++s) {
                                for (int c = 3; c < 35; ++c) {
                                    *(target++) = IntanChip::amp2uV(
                                        little2host16(&*amp + (s + c * streams) * 2));
                                }
                                if (!settings.acquireAux | (!isddrstream[s] && isddrstream[s + 1]))
                                    continue;
                                for (int c = 0; c < 3; ++c) {
                                    if (((ts + 3) % 4) == c)
                                        aux_buffer[s * 3 + c] = IntanChip::aux2V(
                                            little2host16(&*amp + (s + c * streams) * 2));
                                    *(target++) = aux_buffer[s * 3 + c];
                                }
                            }
                            const auto io = data.end() - 8 * 2 - 4 - 4;
                            if (settings.acquireAdc) {
                                for (int c = 0; c < 8; ++c) {
                                    *(target++) = IntanChip::adc2V(little2host16(&*io + 2 * c));
                                }
                            }

                            juce::uint64 ttl = little2host32(&*io + 16);
                            double _ts =
                                use_xdaq_timestamp ? (little2host64(&*io - 8) / 1.0e6) : 0.0;
                            const int chunk_size = 1;
                            sourceBuffers[0]->addToBuffer(&output_buffer[0], &ts, &_ts, &ttl,
                                                          chunk_size);
                        }
                    } else if constexpr (std::is_same_v<T, OwnedData>) {
                    } else {
                        static_assert(xdaq::always_false_v<T>, "non-exhaustive visitor");
                    }
                },
                std::move(event));
        },
        sample_size);

    auto new_stream = evalBoard->dev->start_read_stream(
        0xa0,
        xdaq::DataStream::queue(
            [this, aligned_cb = std::move(aligned_cb)](auto &&event) mutable {
                if (std::holds_alternative<Events::Error>(event)) {
                    LOGE(fmt::format("Getting data stream error : {}",
                                     std::get<Events::Error>(event).error));
                    return;
                } else if (std::holds_alternative<Events::OwnedData>(event)) {
                    aligned_cb(std::move(event));
                } else if (std::holds_alternative<Events::Stop>(event)) {
                    LOGD("Data stream stop");
                }

                if (updateSettingsDuringAcquisition) {
                    LOGD("DAC");
                    for (int k = 0; k < 8; k++) {
                        if (dacChannelsToUpdate[k]) {
                            dacChannelsToUpdate[k] = false;
                            if (dacChannels[k] >= 0) {
                                evalBoard->config_dac(k, true, dacStream[k], dacChannels[k]);
                                evalBoard->setDacThreshold(
                                    k, (int) abs((dacThresholds[k] / 0.195) + 32768),
                                    dacThresholds[k] >= 0);
                                // evalBoard->setDacThresholdVoltage(k, (int)
                                // dacThresholds[k]);
                            } else {
                                evalBoard->enableDac(k, false);
                            }
                        }
                    }

                    evalBoard->setTtlMode(settings.ttlMode ? 1 : 0);
                    evalBoard->enableExternalFastSettle(settings.fastTTLSettleEnabled);
                    evalBoard->setExternalFastSettleChannel(settings.fastSettleTTLChannel);
                    evalBoard->setDacHighpassFilter(settings.desiredDAChpf);
                    evalBoard->enableDacHighpassFilter(settings.desiredDAChpfState);

                    updateSettingsDuringAcquisition = false;
                }

                if (!digitalOutputCommands.empty()) {
                    while (!digitalOutputCommands.empty()) {
                        DigitalOutputCommand command = digitalOutputCommands.front();
                        TTL_OUTPUT_STATE[command.ttlLine] = command.state;
                        digitalOutputCommands.pop();
                    }

                    evalBoard->setTtlOut(TTL_OUTPUT_STATE);

                    LOGB("TTL OUTPUT STATE: ", TTL_OUTPUT_STATE[0], TTL_OUTPUT_STATE[1],
                         TTL_OUTPUT_STATE[2], TTL_OUTPUT_STATE[3], TTL_OUTPUT_STATE[4],
                         TTL_OUTPUT_STATE[5], TTL_OUTPUT_STATE[6], TTL_OUTPUT_STATE[7]);
                }
            },
            128, 4096, std::chrono::microseconds(10)),
        chunk_size);

    if (!new_stream) {
        LOGD("Failed to start stream");
        return false;
    }

    evalBoard->run();

    stream = {std::unique_ptr<xdaq::DeviceManager::OwnedDevice::element_type::DataStream>{
        std::move(new_stream.value())}};


    return true;
}

bool DeviceThread::stopAcquisition()
{
    LOGD("RHD2000 data thread stopping acquisition.");
    if (isThreadRunning()) {
        signalThreadShouldExit();
    }

    evalBoard->setMaxTimeStep(0);
    evalBoard->setContinuousRunMode(false);

    if (stream) {
        stream.reset();
        evalBoard->flush();
    }

    isTransmitting = false;

    if (waitForThreadToExit(500)) {
        LOGD("RHD2000 data thread exited.");
    } else {
        LOGD("RHD2000 data thread failed to exit, continuing anyway...");
    }


    sourceBuffers[0]->clear();

    updateSettingsDuringAcquisition = false;

    // remove timers
    digitalOutputTimers.clear();

    // remove commands
    while (!digitalOutputCommands.empty()) digitalOutputCommands.pop();

    return true;
}

bool DeviceThread::updateBuffer()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

Array<const Headstage *> DeviceThread::getConnectedHeadstages()
{
    Array<const Headstage *> headstageArray;

    for (auto &hs : headstages) {
        if (hs.isConnected()) headstageArray.add(&hs);
    }

    return headstageArray;
}

void DeviceThread::setAdcRange(int channel, short range) { adcRangeSettings[channel] = range; }

short DeviceThread::getAdcRange(int channel) const { return adcRangeSettings[channel]; }

void DeviceThread::runImpedanceTest()
{
    impedanceThread->stopThreadSafely();

    impedanceThread->runThread();
}
