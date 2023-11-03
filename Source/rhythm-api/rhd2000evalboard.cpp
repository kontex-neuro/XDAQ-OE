//----------------------------------------------------------------------------------
// rhd2000evalboardusb3.cpp
//
// Intan Technoloies RHD2000 USB3 Rhythm Interface API
// Rhd2000EvalBoard Class
// Version 2.05 (24 July 2017)
//
// Copyright (c) 2013-2017 Intan Technologies LLC
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the
// use of this software.
//
// Permission is granted to anyone to use this software for any applications that
// use Intan Technologies integrated circuits, and to alter it and redistribute it
// freely.
//
// See http://www.intantech.com for documentation and product information.
//----------------------------------------------------------------------------------

#include "rhd2000evalboard.h"

#include <fmt/format.h>
#include <xdaq/xdaqprotocol.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <ranges>
#include <thread>
#include <vector>


// #include "mock_okCFrontPanel.h"
#include "intan_chip.h"
// #include "okFrontPanelDLL.h"
#include "rhd2000datablock.h"

using namespace std;
using std::vector;
using namespace xdaq;

template <typename Protocol>
    requires protocol::Protocol<Protocol> && protocol::FixSizedMessage<typename Protocol::Request>
auto make_request(typename Protocol::Request request)
{
    constexpr auto data_len = protocol::get_serialized_len<typename Protocol::Request>();
    zmq::message_t msg{protocol::header_length + data_len};
    auto data = std::span<std::byte>(msg.data<std::byte>(), protocol::header_length + data_len);
    auto out = zpp::bits::out(data);
    auto id = Protocol::id;
    out(id).or_throw();
    out(request).or_throw();
    return msg;
}

template <typename Protocol>
    requires protocol::Protocol<Protocol>
auto parse_response(std::span<const std::byte> data)
{
    auto in = zpp::bits::in(data);
    typename Protocol::Response request;
    in(request).or_throw();
    return request;
}

template <typename Protocol>
    requires protocol::Protocol<Protocol> && protocol::FixSizedMessage<typename Protocol::Request>
// TODO: return std::optional
auto make_rpc(const typename Protocol::Request &request, zmq::socket_t &socket)
{
    auto msg = make_request<Protocol>(request);
    socket.send(msg, zmq::send_flags::none);
    zmq::message_t res;
    auto r = socket.recv(res, zmq::recv_flags::none);
    if (!r) throw std::runtime_error("Error in Rhd2000EvalBoard::make_rpc");
    auto data = std::span<const std::byte>(res.data<std::byte>(), res.size());
    return parse_response<Protocol>(data);
}

#ifdef Q_OS_WIN
#include <windows.h>  // for Sleep
#endif

bool Rhd2000EvalBoard::set_dio32(bool dio32)
{
    throw std::runtime_error("Not implemented set_dio32");
    return dio32;
}
template <std::size_t L>
std::size_t length(const char a[L])
{
    return L;
}

// Constructor.  Set sampling rate variable to 30.0 kS/s/channel (FPGA default).
Rhd2000EvalBoard::Rhd2000EvalBoard()
{
    cableDelay.resize(ports.num_of_spi);
    chips.resize(ports.max_chips);
}

Rhd2000EvalBoard::~Rhd2000EvalBoard()
{
    if (is_open) {
    }
    running = false;
    if (receiving_thread.joinable()) receiving_thread.join();
}

// Find an Opal Kelly XEM6310-LX45 board attached to a USB port and open it.
// Returns 1 if successful, -1 if FrontPanel cannot be loaded, and -2 if XEM6310 can't be found.
int Rhd2000EvalBoard::open(const char *libname)
{
    socket = zmq::socket_t(context, zmq::socket_type::req);
    socket.connect("tcp://localhost:5555");
    data_socket = zmq::socket_t(context, zmq::socket_type::sub);
    data_socket.connect("tcp://localhost:5565");
    data_socket.set(zmq::sockopt::subscribe, "");
    // throw std::runtime_error("Not implemented");
    return 1;
}

// Set the per-channel sampling rate of the RHD2000 chips connected to the FPGA.
bool Rhd2000EvalBoard::setSampleRate(SampleRate newSampleRate)
{
    sampleRate = newSampleRate;
    // throw std::runtime_error("Not implemented");
    return true;
}

// Returns the current per-channel sampling rate (in Hz) as a floating-point number.
double Rhd2000EvalBoard::getSampleRate() const
{
    switch (sampleRate) {
    case SampleRate::s1000Hz: return 1000.0; break;
    case SampleRate::s1250Hz: return 1250.0; break;
    case SampleRate::s1500Hz: return 1500.0; break;
    case SampleRate::s2000Hz: return 2000.0; break;
    case SampleRate::s2500Hz: return 2500.0; break;
    case SampleRate::s3000Hz: return 3000.0; break;
    case SampleRate::s3333Hz: return (10000.0 / 3.0); break;
    case SampleRate::s4000Hz: return 4000.0; break;
    case SampleRate::s5000Hz: return 5000.0; break;
    case SampleRate::s6250Hz: return 6250.0; break;
    case SampleRate::s8000Hz: return 8000.0; break;
    case SampleRate::s10000Hz: return 10000.0; break;
    case SampleRate::s12500Hz: return 12500.0; break;
    case SampleRate::s15000Hz: return 15000.0; break;
    case SampleRate::s20000Hz: return 20000.0; break;
    case SampleRate::s25000Hz: return 25000.0; break;
    case SampleRate::s30000Hz: return 30000.0; break;
    default: return -1.0;
    }
}

Rhd2000EvalBoard::SampleRate Rhd2000EvalBoard::getSampleRateEnum() const { return sampleRate; }

// Upload an auxiliary command list to a particular command slot (AuxCmd1, AuxCmd2, or AuxCmd3) and
// RAM bank (0-15) on the FPGA.
void Rhd2000EvalBoard::uploadCommandList(const std::vector<uint32_t> &commandList,
                                         AuxCmdSlot auxCommandSlot, int bank)
{
    throw std::runtime_error("Not implemented uploadCommandList");
    if (auxCommandSlot == AuxCmdSlot::All) {
        cerr << "Error in Rhd2000EvalBoard::uploadCommandList: auxCommandSlot cannot be All.\n";
        return;
    }

    if (bank < 0 || bank > 15) {
        cerr << "Error in Rhd2000EvalBoard::uploadCommandList: bank out of range.\n";
        return;
    }
    //                    01234567890123456789
    constexpr auto cmd = "uploadCommandList   ";
    zmq::message_t req(20 + sizeof(uint32_t) * commandList.size() + sizeof(std::int32_t) * 2);
    unsigned char *data = (unsigned char *) req.data();
    std::memcpy(data, cmd, 20);
    data += 20;
    std::memcpy(data, commandList.data(), sizeof(uint32_t) * commandList.size());
    data += sizeof(uint32_t) * commandList.size();
    std::memcpy(data, &auxCommandSlot, sizeof(std::int32_t));
    data += sizeof(std::int32_t);
    std::memcpy(data, &bank, sizeof(std::int32_t));
    data += sizeof(std::int32_t);
    socket.send(req, zmq::send_flags::none);
    zmq::message_t res;
    auto r = socket.recv(res, zmq::recv_flags::none);
    if (!r) throw std::runtime_error("Error in Rhd2000EvalBoard::uploadCommandList");
}

// Select an auxiliary command slot (AuxCmd1, AuxCmd2, or AuxCmd3) and bank (0-15) for a particular
// SPI port (PortA - PortH) on the FPGA.
void Rhd2000EvalBoard::selectAuxCommandBank(SPIPort port, AuxCmdSlot auxCommandSlot, int bank)
{
    throw std::runtime_error("Not implemented selectAuxCommandBank");
}

// Specify a command sequence length (endIndex = 0-1023) and command loop index (0-1023) for a
// particular auxiliary command slot (AuxCmd1, AuxCmd2, or AuxCmd3).
void Rhd2000EvalBoard::selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex,
                                              int endIndex)
{
    throw std::runtime_error("Not implemented selectAuxCommandLength");
}



// Set maxTimeStep for cases where continuousMode == false.
void Rhd2000EvalBoard::setMaxTimeStep(unsigned int maxTimeStep)
{
    fmt::print(stderr, "Not implemented setMaxTimeStep\n");
    return;
}


// Initiate SPI data acquisition.
void Rhd2000EvalBoard::run(std::function<void(std::span<const std::byte>)> callback)
{
    if (!isRunning()) make_rpc<protocol::start_run>({}, socket);
    running = false;
    if (receiving_thread.joinable()) {
        receiving_thread.join();
        throw std::runtime_error("Error in Rhd2000EvalBoard::run");
    }

    running = true;
    receiving_thread = std::thread([&, callback]() {
        while (running) {
            zmq::message_t msg;
            auto r = data_socket.recv(msg, zmq::recv_flags::dontwait);
            if (r) {
                callback({msg.data<std::byte>(), msg.size()});
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });
}

void Rhd2000EvalBoard::stop()
{
    running = false;
    if (receiving_thread.joinable()) {
        receiving_thread.join();
        receiving_thread = std::thread();
    }
    make_rpc<protocol::stop_run>({}, socket);
}

// Is the FPGA currently running?
bool Rhd2000EvalBoard::isRunning()
{
    auto res = make_rpc<protocol::is_running>({}, socket);
    return res.running;
}

// Returns the number of 16-bit words in the USB FIFO.  The user should never attempt to read
// more data than the FIFO currently contains, as it is not protected against underflow.
// (Private method.)
unsigned int Rhd2000EvalBoard::numWordsInFifo()
{
    throw std::runtime_error("Not implemented numWordsInFifo");
}


// Enable or disable one of the 32 available USB data streams (0-31).
void Rhd2000EvalBoard::enableDataStream(int stream, bool enabled)
{
    if (stream < 0 || stream > (MAX_NUM_DATA_STREAMS - 1)) {
        cerr << "Error in Rhd2000EvalBoard::enableDataStream: stream out of range." << endl;
        return;
    }
    make_rpc<protocol::set_stream_enabled>(
        {.stream = static_cast<std::uint32_t>(stream), .enabled = enabled}, socket);

    if (enabled) {
        if (dataStreamEnabled[stream] == 0) {
            dataStreamEnabled[stream] = 1;
            numDataStreams++;
        }
    } else {
        if (dataStreamEnabled[stream] == 1) {
            dataStreamEnabled[stream] = 0;
            numDataStreams--;
        }
    }
}


// Set the 16 bits of the digital TTL output lines on the FPGA high or low according to integer
// array.
void Rhd2000EvalBoard::setTtlOut(int ttlOutArray[])
{
    throw std::runtime_error("Not implemented setTtlOut");
}


// Set the eight red LEDs on the front panel SPI ports according to integer array.
void Rhd2000EvalBoard::setSpiLedDisplay(int ledArray[])
{
    throw std::runtime_error("Not implemented setSpiLedDisplay");
}



// Enable or disable DAC channel (0-7)
void Rhd2000EvalBoard::enableDac(int dacChannel, bool enabled)
{
    throw std::runtime_error("Not implemented enableDac");
}

// Set the gain level of all eight DAC channels to 2^gain (gain = 0-7).
void Rhd2000EvalBoard::setDacGain(int gain)
{
    throw std::runtime_error("Not implemented setDacGain");
    if (gain < 0 || gain > 7) {
        cerr << "Error in Rhd2000EvalBoard::setDacGain: gain setting out of range." << endl;
        return;
    }
}


// Suppress the noise on DAC channels 0 and 1 (the audio channels) between
// +16*noiseSuppress and -16*noiseSuppress LSBs.  (noiseSuppress = 0-127).
void Rhd2000EvalBoard::setAudioNoiseSuppress(int noiseSuppress)
{
    fmt::print(stderr, "Not implemented setAudioNoiseSuppress\n");
    return;
    throw std::runtime_error("Not implemented setAudioNoiseSuppress");
    if (noiseSuppress < 0 || noiseSuppress > 127) {
        cerr << "Error in Rhd2000EvalBoard::setAudioNoiseSuppress: noiseSuppress out of range."
             << endl;
        return;
    }
}


void Rhd2000EvalBoard::config_dac(int channel, bool enable, int source_stream, int source_channel)
{
    throw std::runtime_error("Not implemented config_dac");
    const bool rhs = false;
    if (channel < 0 || channel > ports.num_of_dac) {
        std::cerr << "channel out of range" << std::endl;
        return;
    }
    if (source_stream < 0 || source_stream > (ports.max_streams + 1)) {
        std::cerr << "stream out of range" << std::endl;
        return;
    }
    if (source_channel < 0 || source_channel > 31) {
        std::cerr << "channel out of range" << std::endl;
        return;
    }
    // RHD [enable:1bit, stream:6bit, channel:5bit]
    // RHS [enable:1bit, stream:4bit, channel:5bit]
}

// Enable external triggering of amplifier hardware 'fast settle' function (blanking).
// If external triggering is enabled, the fast settling of amplifiers on all connected
// chips will be controlled in real time via one of the 16 TTL inputs.
void Rhd2000EvalBoard::enableExternalFastSettle(bool enable)
{
    throw std::runtime_error("Not implemented enableExternalFastSettle");
}

// Select which of the TTL inputs 0-15 is used to perform a hardware 'fast settle' (blanking)
// of the amplifiers if external triggering of fast settling is enabled.
void Rhd2000EvalBoard::setExternalFastSettleChannel(int channel)
{
    throw std::runtime_error("Not implemented setExternalFastSettleChannel");

    if (channel < 0 || channel > 15) {
        cerr << "Error in Rhd2000EvalBoard::setExternalFastSettleChannel: channel out of range."
             << endl;
        return;
    }
}

// Enable external control of RHD2000 auxiliary digital output pin (auxout).
// If external control is enabled, the digital output of all chips connected to a
// selected SPI port will be controlled in real time via one of the 16 TTL inputs.
void Rhd2000EvalBoard::enableExternalDigOut(SPIPort port, bool enable)
{
    throw std::runtime_error("Not implemented enableExternalDigOut");
}

// Select which of the TTL inputs 0-15 is used to control the auxiliary digital output
// pin of the chips connected to a particular SPI port, if external control of auxout is enabled.
void Rhd2000EvalBoard::setExternalDigOutChannel(SPIPort port, int channel)
{
    throw std::runtime_error("Not implemented setExternalDigOutChannel");
}

// Enable optional FPGA-implemented digital high-pass filters associated with DAC outputs
// on USB interface board.. These one-pole filters can be used to record wideband neural data
// while viewing only spikes without LFPs on the DAC outputs, for example.  This is useful when
// using the low-latency FPGA thresholds to detect spikes and produce digital pulses on the TTL
// outputs, for example.
void Rhd2000EvalBoard::enableDacHighpassFilter(bool enable)
{
    throw std::runtime_error("Not implemented enableDacHighpassFilter");
}

// Set cutoff frequency (in Hz) for optional FPGA-implemented digital high-pass filters
// associated with DAC outputs on USB interface board.  These one-pole filters can be used
// to record wideband neural data while viewing only spikes without LFPs on the DAC outputs,
// for example.  This is useful when using the low-latency FPGA thresholds to detect spikes
// and produce digital pulses on the TTL outputs, for example.
void Rhd2000EvalBoard::setDacHighpassFilter(double cutoff)
{
    throw std::runtime_error("Not implemented setDacHighpassFilter");
}

// Set thresholds for DAC channels; threshold output signals appear on TTL outputs 0-7.
// The parameter 'threshold' corresponds to the RHD2000 chip ADC output value, and must fall
// in the range of 0 to 65535, where the 'zero' level is 32768.
// If trigPolarity is true, voltages equaling or rising above the threshold produce a high TTL
// output. If trigPolarity is false, voltages equaling or falling below the threshold produce a high
// TTL output.
void Rhd2000EvalBoard::setDacThreshold(int dacChannel, int threshold, bool trigPolarity)
{
    throw std::runtime_error("Not implemented setDacThreshold");
}

// Set the TTL output mode of the board.
// mode = 0: All 16 TTL outputs are under manual control
// mode = 1: Top 8 TTL outputs are under manual control;
//           Bottom 8 TTL outputs are outputs of DAC comparators
void Rhd2000EvalBoard::setTtlMode(int mode)
{
    throw std::runtime_error("Not implemented setTtlMode");
}

// std::optional<Rhd2000DataBlock> Rhd2000EvalBoard::run_and_read_samples(
//     int samples, std::optional<std::chrono::milliseconds> timeout)
// {
//     setMaxTimeStep(samples);
//     setContinuousRunMode(false);
//     run();
//     if (timeout) {
//         auto start = std::chrono::high_resolution_clock::now();
//         while (isRunning()) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//             auto now = std::chrono::high_resolution_clock::now();
//             if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start) > *timeout) {
//                 std::cerr << "Timeout waiting for data block" << std::endl;
//                 return std::nullopt;
//             }
//         }
//     } else {
//         while (isRunning())
//             ;
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
//
//     auto available = get_num_samples_available(true);
//     if (available != samples) {
//         std::cerr << "Error: expected " << samples << " samples, got " << available << std::endl;
//         return std::nullopt;
//     }
//
//     return read_samples(samples);
// }
// Read data block from the USB interface, if one is available.  Returns true if data block
// was available.
// std::optional<Rhd2000DataBlock> Rhd2000EvalBoard::read_samples(int samples)
// {
//     lock_guard<mutex> lockOk(okMutex);
//     auto result = read_raw_samples(samples, &usbBuffer[0]);
//     if (result < 0) {
//         std::cerr << "Error reading data block " << result << std::endl;
//         return std::nullopt;
//     }
//     return std::make_optional<Rhd2000DataBlock>(numDataStreams, samples, dio32, &usbBuffer[0]);
// }

// Return FPGA cable delay for selected SPI port.
int Rhd2000EvalBoard::getCableDelay(SPIPort port) const
{
    if (port == SPIPort::All) return -1;
    return cableDelay[static_cast<int>(port)];
}

// Return FPGA cable delays for all SPI ports.
void Rhd2000EvalBoard::getCableDelay(std::vector<int> &delays) const
{
    if (delays.size() != MAX_NUM_SPI_PORTS) {
        delays.resize(MAX_NUM_SPI_PORTS);
    }
    for (int i = 0; i < MAX_NUM_SPI_PORTS; ++i) {
        delays[i] = cableDelay[i];
    }
}

const std::vector<IntanChip::Chip> &Rhd2000EvalBoard::scan_chips()
{
    auto res = make_rpc<protocol::scan_chips>({}, socket);
    for (int i = 0; i < ports.max_chips; ++i) {
        chips[i].id = (IntanChip::ChipID) std::get<0>(res.chips[i]);
        chips[i].miso = (IntanChip::ChipMISO) std::get<1>(res.chips[i]);
    }
    return chips;
}