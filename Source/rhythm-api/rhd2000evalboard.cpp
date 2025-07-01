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
#include <xdaq/data_streams.h>
#include <xdaq/device_manager.h>

#include <array>
#include <chrono>
#include <cmath>
#include <expected>
#include <future>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
#include <thread>
#include <vector>

#include "intan_chip.h"
#include "rhd2000datablock.h"



using namespace std;
using std::vector;

using json = nlohmann::json;

// Opal Kelly module USB interface endpoint addresses
enum OkEndPoint {
    WireInResetRun = 0x00,
    WireInMaxTimeStep = 0x01,
    WireInSerialDigitalInCntl = 0x02,
    WireInDataFreqPll = 0x03,
    WireInMisoDelay = 0x04,
    WireInCmdRamAddr = 0x05,
    WireInCmdRamBank = 0x06,
    WireInCmdRamData = 0x07,
    WireInAuxCmdBank1 = 0x08,
    WireInAuxCmdBank2 = 0x09,
    WireInAuxCmdBank3 = 0x0a,
    WireInAuxCmdLength = 0x0b,
    WireInAuxCmdLoop = 0x0c,
    WireInLedDisplay = 0x0d,
    WireInDacReref = 0x0e,
    // Note: room for extra WireIns here
    WireInDataStreamEn = 0x14,
    WireInTtlOut = 0x15,
    WireInDacSource1 = 0x16,
    WireInDacSource2 = 0x17,
    WireInDacSource3 = 0x18,
    WireInDacSource4 = 0x19,
    WireInDacSource5 = 0x1a,
    WireInDacSource6 = 0x1b,
    WireInDacSource7 = 0x1c,
    WireInDacSource8 = 0x1d,
    WireInDacManual = 0x1e,
    WireInMultiUse = 0x1f,

    TrigInConfig = 0x40,
    TrigInSpiStart = 0x41,
    TrigInDacConfig = 0x42,

    WireOutNumWords = 0x20,
    WireOutSerialDigitalIn = 0x21,
    WireOutSpiRunning = 0x22,
    WireOutTtlIn = 0x23,
    WireOutDataClkLocked = 0x24,
    WireOutBoardMode = 0x25,
    // Note: room for extra WireOuts here
    WireOutBoardId = 0x3e,
    WireOutBoardVersion = 0x3f,

    PipeOutData = 0xa0,


    // kontexdev
    TrigVStim = 0x40,
    WireInVStimMultiUse = 0x1f,
    TrigMCU = 0x48,
    WireOutXDAQStatus = 0x22,
    // Write Serial Numbers b1=1 (default=0) 0x02, b0=1 (defualt = 0) -> FNEB_ReadB
    WireInMCUControl = 0x02,
    PipeInFirmware = 0x88,
    PipeOutFirmware = 0xb0,
    WireInSetMode = 0x00,
    Enable32bitDIO = 0x12,
    PipeInDAC1 = 0x90,
    PipeInDAC2 = 0x91,
    PipeInDAC3 = 0x92,
    PipeInDAC4 = 0x93,
    PipeInDAC5 = 0x94,
    PipeInDAC6 = 0x95,
    PipeInDAC7 = 0x96,
    PipeInDAC8 = 0x97
};

constexpr int WireInAuxCmdBank[] = {OkEndPoint::WireInAuxCmdBank1, OkEndPoint::WireInAuxCmdBank2,
                                    OkEndPoint::WireInAuxCmdBank3};

constexpr int WireInDacSource[] = {OkEndPoint::WireInDacSource1, OkEndPoint::WireInDacSource2,
                                   OkEndPoint::WireInDacSource3, OkEndPoint::WireInDacSource4,
                                   OkEndPoint::WireInDacSource5, OkEndPoint::WireInDacSource6,
                                   OkEndPoint::WireInDacSource7, OkEndPoint::WireInDacSource8};

bool Rhd2000EvalBoard::UploadDACData(const vector<uint16_t> &commandList, int dacChannel,
                                     int length)
{
    lock_guard<mutex> lockOk(okMutex);

    const int words = commandList.size();
    vector<uint8_t> dacBuffer(2 * words);
    for (int i = 0; i < words; i++) {
        dacBuffer[2 * i] = commandList[i] & 0xFF;
        dacBuffer[2 * i + 1] = (commandList[i] & 0xFF00) >> 8;
    }

    dev->trigger(0x41, 2);  // reset DAC write counter, set bit2 = 1
    auto result = dev->write(PipeInDAC1 + dacChannel, 2 * words, &dacBuffer[0]);
    if (result < 0) return false;

    dev->set_register_sync(0x1F, length - 1);  // set loop index
    dev->trigger(0x41, 8 + dacChannel);        // latch DAC loop index channel 8 => 8+7
    return true;
}


Rhd2000EvalBoard::XDAQStatus Rhd2000EvalBoard::getXDAQStatus()
{
    auto value = dev->get_register_sync(WireOutXDAQStatus).value();
    auto is_stat = [](auto v, XDAQStatus s) {
        return (v & static_cast<uint8_t>(s)) == static_cast<uint8_t>(s);
    };

    if (is_stat(value, XDAQStatus::MCU_BUSY)) {
        return XDAQStatus::MCU_BUSY;
    } else if (is_stat(value, XDAQStatus::MCU_ERROR)) {
        return XDAQStatus::MCU_ERROR;
    } else if (is_stat(value, XDAQStatus::MCU_DONE)) {
        return XDAQStatus::MCU_DONE;
    } else if (value == 0) {
        return XDAQStatus::MCU_IDLE;
    }
    return XDAQStatus::MCU_BUSY;
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
        std::array<int, 8> led = {0, 0, 0, 0, 0, 0, 0, 0};
        setSpiLedDisplay(&led[0]);
        resetFpga();
    }
}

// Find an Opal Kelly XEM6310-LX45 board attached to a USB port and open it.
// Returns 1 if successful, -1 if FrontPanel cannot be loaded, and -2 if XEM6310 can't be found.
int Rhd2000EvalBoard::open(xdaq::DeviceManager::OwnedDevice dev)
{
    this->dev = std::move(dev);
    expander = true;
    is_open = true;
    return 1;
}

// Initialize Rhythm FPGA to default starting values.
void Rhd2000EvalBoard::initialize()
{
    resetBoard();
    setSampleRate(SampleRate::s30000Hz);
    selectAuxCommandBank(SPIPort::All, AuxCmdSlot::All, 0);
    selectAuxCommandLength(AuxCmdSlot::All, 0, 0);

    setContinuousRunMode(true);
    setMaxTimeStep(4294967295);  // 4294967295 == (2^32 - 1)

    setCableLengthFeet(SPIPort::All, 3.0);  // assume 3 ft cables

    setDspSettle(false);

    // Must first force all data streams off
    dev->set_register_sync(WireInDataStreamEn, 0);

    enableDataStream(0, true);  // start with only one data stream enabled
    for (int i = 1; i < MAX_NUM_DATA_STREAMS; i++) {
        enableDataStream(i, false);
    }

    clearTtlOut();

    for (int i = 0; i < ports.num_of_dac; ++i) {
        config_dac(i, false, ports.max_streams, 0);
    }

    setDacManual(32768);  // midrange value = 0 V
    setDacGain(0);
    setAudioNoiseSuppress(0);

    for (int i = 0; i < ports.num_of_dac; ++i) setDacThreshold(i, 32768, true);

    setTtlMode(1);  // Digital outputs 0-7 are DAC comparators; 8-15 under manual control

    enableExternalFastSettle(false);
    setExternalFastSettleChannel(0);

    enableExternalDigOut(SPIPort::All, false);
    setExternalDigOutChannel(SPIPort::All, 0);

    enableDacReref(false);
}

// Set the per-channel sampling rate of the RHD2000 chips connected to the FPGA.
bool Rhd2000EvalBoard::setSampleRate(SampleRate newSampleRate)
{
    lock_guard<mutex> lockOk(okMutex);

    // Assuming a 100 MHz reference clock is provided to the FPGA, the programmable FPGA clock
    // frequency is given by:
    //
    //       FPGA internal clock frequency = 100 MHz * (M/D) / 2
    //
    // M and D are "multiply" and "divide" integers used in the FPGA's digital clock manager (DCM)
    // phase- locked loop (PLL) frequency synthesizer, and are subject to the following
    // restrictions:
    //
    //                M must have a value in the range of 2 - 256
    //                D must have a value in the range of 1 - 256
    //                M/D must fall in the range of 0.05 - 3.33
    //
    // (See pages 85-86 of Xilinx document UG382 "Spartan-6 FPGA Clocking Resources" for more
    // details.)
    //
    // This variable-frequency clock drives the state machine that controls all SPI communication
    // with the RHD2000 chips.  A complete SPI cycle (consisting of one CS pulse and 16 SCLK pulses)
    // takes 80 clock cycles.  The SCLK period is 4 clock cycles; the CS pulse is high for 14 clock
    // cycles between commands.
    //
    // Rhythm samples all 32 channels and then executes 3 "auxiliary" commands that can be used to
    // read and write from other registers on the chip, or to sample from the temperature sensor or
    // auxiliary ADC inputs, for example.  Therefore, a complete cycle that samples from each
    // amplifier channel takes 80 * (32 + 3) = 80 * 35 = 2800 clock cycles.
    //
    // So the per-channel sampling rate of each amplifier is 2800 times slower than the clock
    // frequency.
    //
    // Based on these design choices, we can use the following values of M and D to generate the
    // following useful amplifier sampling rates for electrophsyiological applications:
    //
    //   M    D     clkout frequency    per-channel sample rate     per-channel sample period
    //  ---  ---    ----------------    -----------------------     -------------------------
    //    7  125          2.80 MHz               1.00 kS/s                 1000.0 usec = 1.0 msec
    //    7  100          3.50 MHz               1.25 kS/s                  800.0 usec
    //   21  250          4.20 MHz               1.50 kS/s                  666.7 usec
    //   14  125          5.60 MHz               2.00 kS/s                  500.0 usec
    //   35  250          7.00 MHz               2.50 kS/s                  400.0 usec
    //   21  125          8.40 MHz               3.00 kS/s                  333.3 usec
    //   14   75          9.33 MHz               3.33 kS/s                  300.0 usec
    //   28  125         11.20 MHz               4.00 kS/s                  250.0 usec
    //    7   25         14.00 MHz               5.00 kS/s                  200.0 usec
    //    7   20         17.50 MHz               6.25 kS/s                  160.0 usec
    //  112  250         22.40 MHz               8.00 kS/s                  125.0 usec
    //   14   25         28.00 MHz              10.00 kS/s                  100.0 usec
    //    7   10         35.00 MHz              12.50 kS/s                   80.0 usec
    //   21   25         42.00 MHz              15.00 kS/s                   66.7 usec
    //   28   25         56.00 MHz              20.00 kS/s                   50.0 usec
    //   35   25         70.00 MHz              25.00 kS/s                   40.0 usec
    //   42   25         84.00 MHz              30.00 kS/s                   33.3 usec
    //
    // To set a new clock frequency, assert new values for M and D (e.g., using okWireIn modules)
    // and pulse DCM_prog_trigger high (e.g., using an okTriggerIn module).  If this module is
    // reset, it reverts to a per-channel sampling rate of 30.0 kS/s.

    unsigned long M, D;

    switch (newSampleRate) {
    case SampleRate::s1000Hz:
        M = 7;
        D = 125;
        break;
    case SampleRate::s1250Hz:
        M = 7;
        D = 100;
        break;
    case SampleRate::s1500Hz:
        M = 21;
        D = 250;
        break;
    case SampleRate::s2000Hz:
        M = 14;
        D = 125;
        break;
    case SampleRate::s2500Hz:
        M = 35;
        D = 250;
        break;
    case SampleRate::s3000Hz:
        M = 21;
        D = 125;
        break;
    case SampleRate::s3333Hz:
        M = 14;
        D = 75;
        break;
    case SampleRate::s4000Hz:
        M = 28;
        D = 125;
        break;
    case SampleRate::s5000Hz:
        M = 7;
        D = 25;
        break;
    case SampleRate::s6250Hz:
        M = 7;
        D = 20;
        break;
    case SampleRate::s8000Hz:
        M = 112;
        D = 250;
        break;
    case SampleRate::s10000Hz:
        M = 14;
        D = 25;
        break;
    case SampleRate::s12500Hz:
        M = 7;
        D = 10;
        break;
    case SampleRate::s15000Hz:
        M = 21;
        D = 25;
        break;
    case SampleRate::s20000Hz:
        M = 28;
        D = 25;
        break;
    case SampleRate::s25000Hz:
        M = 35;
        D = 25;
        break;
    case SampleRate::s30000Hz:
        M = 42;
        D = 25;
        break;
    default: return (false);
    }

    sampleRate = newSampleRate;

    // Wait for DcmProgDone = 1 before reprogramming clock synthesizer
    while (isDcmProgDone() == false);

    // Reprogram clock synthesizer
    dev->set_register_sync(WireInDataFreqPll, (256 * M + D));
    dev->trigger(TrigInConfig, 0);

    // Wait for DataClkLocked = 1 before allowing data acquisition to continue
    while (isDataClockLocked() == false);

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
    lock_guard<mutex> lockOk(okMutex);
    if (auxCommandSlot == AuxCmdSlot::All) {
        cerr << "Error in Rhd2000EvalBoard::uploadCommandList: auxCommandSlot cannot be All.\n";
        return;
    }

    if (bank < 0 || bank > 15) {
        cerr << "Error in Rhd2000EvalBoard::uploadCommandList: bank out of range.\n";
        return;
    }

    dev->set_register(WireInCmdRamBank, bank);
    const auto aux = static_cast<int>(auxCommandSlot);
    for (size_t i = 0; i < commandList.size(); ++i) {
        dev->set_register(WireInCmdRamData, commandList[i]);
        dev->set_register_sync(WireInCmdRamAddr, i);
        dev->trigger(TrigInConfig, aux + 1);
    }
}

// Select an auxiliary command slot (AuxCmd1, AuxCmd2, or AuxCmd3) and bank (0-15) for a particular
// SPI port (PortA - PortH) on the FPGA.
void Rhd2000EvalBoard::selectAuxCommandBank(SPIPort port, AuxCmdSlot auxCommandSlot, int bank)
{
    lock_guard<mutex> lockOk(okMutex);
    if (bank < 0 || bank > 15) {
        cerr << "Error in Rhd2000EvalBoard::selectAuxCommandBank: bank out of range." << endl;
        return;
    }

    uint32_t value, mask;
    if (port == SPIPort::All) {
        value = bank | (bank << 4);
        value |= value << 8;
        value |= value << 16;
        mask = 0xffffffff;
    } else {
        value = bank << static_cast<int>(port) * 4;
        mask = 0xf << static_cast<int>(port) * 4;
    }
    if (auxCommandSlot == AuxCmdSlot::All) {
        for (int i = 0; i < 3; ++i) dev->set_register_sync(WireInAuxCmdBank[i], value, mask);
    } else {
        dev->set_register_sync(WireInAuxCmdBank[static_cast<int>(auxCommandSlot)], value, mask);
    }
}

// Specify a command sequence length (endIndex = 0-1023) and command loop index (0-1023) for a
// particular auxiliary command slot (AuxCmd1, AuxCmd2, or AuxCmd3).
void Rhd2000EvalBoard::selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex,
                                              int endIndex)
{
    lock_guard<mutex> lockOk(okMutex);

    if (loopIndex < 0 || loopIndex > 1023) {
        cerr << "Error in Rhd2000EvalBoard::selectAuxCommandLength: loopIndex out of range.\n";
        return;
    }

    if (endIndex < 0 || endIndex > 1023) {
        cerr << "Error in Rhd2000EvalBoard::selectAuxCommandLength: endIndex out of range.\n";
        return;
    }

    if (auxCommandSlot == AuxCmdSlot::All) {
        dev->set_register(WireInAuxCmdLoop, loopIndex | (loopIndex << 10) | (loopIndex << 20));
        dev->set_register_sync(WireInAuxCmdLength, endIndex | (endIndex << 10) | (endIndex << 20));
    } else {
        const auto aux = static_cast<int>(auxCommandSlot);
        dev->set_register(WireInAuxCmdLoop, loopIndex << aux * 10, 0x3ff << aux * 10);
        dev->set_register_sync(WireInAuxCmdLength, endIndex << aux * 10, 0x3ff << aux * 10);
    }
}

// Reset FPGA.  This clears all auxiliary command RAM banks, clears the USB FIFO, and resets the
// per-channel sampling rate to 30.0 kS/s/ch.
void Rhd2000EvalBoard::resetBoard() {}

// Low-level FPGA reset.  Call when closing application to make sure everything has stopped.
void Rhd2000EvalBoard::resetFpga()
{
    lock_guard<mutex> lockOk(okMutex);

    // dev->ResetFPGA();
    is_open = false;
}

// Set the FPGA to run continuously once started (if continuousMode == true) or to run until
// maxTimeStep is reached (if continuousMode == false).
void Rhd2000EvalBoard::setContinuousRunMode(bool continuousMode)
{
    lock_guard<mutex> lockOk(okMutex);

    if (continuousMode) {
        dev->set_register_sync(WireInResetRun, 0x02, 0x02);
    } else {
        dev->set_register_sync(WireInResetRun, 0x00, 0x02);
    }
}

// Set maxTimeStep for cases where continuousMode == false.
void Rhd2000EvalBoard::setMaxTimeStep(unsigned int maxTimeStep)
{
    lock_guard<mutex> lockOk(okMutex);

    dev->set_register_sync(WireInMaxTimeStep, maxTimeStep);
}

// Initiate SPI data acquisition.
void Rhd2000EvalBoard::run()
{
    lock_guard<mutex> lockOk(okMutex);

    dev->trigger(TrigInSpiStart, 0);
}

// Is the FPGA currently running?
bool Rhd2000EvalBoard::isRunning()
{
    lock_guard<mutex> lockOk(okMutex);
    int value;

    value = dev->get_register_sync(WireOutSpiRunning).value();

    if ((value & 0x01) == 0) {
        return false;
    } else {
        return true;
    }
}


// Set the delay for sampling the MISO line on a particular SPI port (PortA - PortH), in integer
// clock steps, where each clock step is 1/2800 of a per-channel sampling period. Note: Cable delay
// must be updated after sampleRate is changed, since cable delay calculations are based on the
// clock frequency!
void Rhd2000EvalBoard::setCableDelay(SPIPort port, int delay)
{
    lock_guard<mutex> lockOk(okMutex);
    if (delay < -1 || delay > 15) {
        cerr << "Warning in Rhd2000EvalBoard::setCableDelay: delay out of range: " << delay << endl;
        return;
    }

    if (port == SPIPort::All) {
        if (delay >= 0) dev->set_register_sync(WireInMisoDelay, delay * 0x11111111llu);
        std::fill(cableDelay.begin(), cableDelay.end(), delay);
    } else {
        const auto idx = static_cast<int>(port);
        if (delay >= 0) dev->set_register_sync(WireInMisoDelay, delay << 4 * idx, 0xf << 4 * idx);
        cableDelay[idx] = delay;
    }
}

// Set the delay for sampling the MISO line on a particular SPI port (PortA - PortH) based on the
// length of the cable between the FPGA and the RHD2000 chip (in meters). Note: Cable delay must be
// updated after sampleRate is changed, since cable delay calculations are based on the clock
// frequency!
void Rhd2000EvalBoard::setCableLengthMeters(SPIPort port, double lengthInMeters)
{
    double tStep, cableVelocity, distance, timeDelay;
    const double speedOfLight = 299792458.0;      // units = meters per second
    const double xilinxLvdsOutputDelay = 1.9e-9;  // 1.9 ns Xilinx LVDS output pin delay
    const double xilinxLvdsInputDelay = 1.4e-9;   // 1.4 ns Xilinx LVDS input pin delay
    const double rhd2000Delay = 9.0e-9;           // 9.0 ns RHD2000 SCLK-to-MISO delay
    const double misoSettleTime = 6.7e-9;  // 6.7 ns delay after MISO changes, before we sample it

    tStep = 1.0 / (2800.0 * getSampleRate());  // data clock that samples MISO has a rate 35 x 80 =
                                               // 2800x higher than the sampling rate
    // cableVelocity = 0.67 * speedOfLight;  // propogation velocity on cable: version 1.3 and
    // earlier
    cableVelocity = 0.555 * speedOfLight;  // propogation velocity on cable: version 1.4 improvement
                                           // based on cable measurements
    distance = 2.0 * lengthInMeters;       // round trip distance data must travel on cable
    timeDelay = (distance / cableVelocity) + xilinxLvdsOutputDelay + rhd2000Delay +
                xilinxLvdsInputDelay + misoSettleTime;

    int delay = (int) floor(((timeDelay / tStep) + 1.0) + 0.5);

    if (delay < 1)
        delay = 1;  // delay of zero is too short (due to I/O delays), even for zero-length cables

    setCableDelay(port, delay);
}

// Same function as above, but accepts lengths in feet instead of meters
void Rhd2000EvalBoard::setCableLengthFeet(SPIPort port, double lengthInFeet)
{
    setCableLengthMeters(port, 0.3048 * lengthInFeet);  // convert feet to meters
}

// Estimate cable length based on a particular delay used in setCableDelay.
// (Note: Depends on sample rate.)
double Rhd2000EvalBoard::estimateCableLengthMeters(int delay) const
{
    const double speedOfLight = 299792458.0;      // units = meters per second
    const double xilinxLvdsOutputDelay = 1.9e-9;  // 1.9 ns Xilinx LVDS output pin delay
    const double xilinxLvdsInputDelay = 1.4e-9;   // 1.4 ns Xilinx LVDS input pin delay
    const double rhd2000Delay = 9.0e-9;           // 9.0 ns RHD2000 SCLK-to-MISO delay
    const double misoSettleTime = 6.7e-9;  // 6.7 ns delay after MISO changes, before we sample it

    const double tStep =
        1.0 / (2800.0 * getSampleRate());  // data clock that samples MISO has a rate 35 x 80 =
                                           // 2800x higher than the sampling rate
    // cableVelocity = 0.67 * speedOfLight;  // propogation velocity on cable: version 1.3 and
    // earlier
    const double cableVelocity =
        0.555 * speedOfLight;  // propogation velocity on cable: version 1.4 improvement
                               // based on cable measurements

    // distance = cableVelocity * (delay * tStep - (xilinxLvdsOutputDelay + rhd2000Delay +
    // xilinxLvdsInputDelay));  // version 1.3 and earlier
    double distance =
        cableVelocity * ((((double) delay) - 1.0) * tStep -
                         (xilinxLvdsOutputDelay + rhd2000Delay + xilinxLvdsInputDelay +
                          misoSettleTime));  // version 1.4 improvement
    if (distance < 0.0) distance = 0.0;

    return (distance / 2.0);
}

// Same function as above, but returns length in feet instead of meters
double Rhd2000EvalBoard::estimateCableLengthFeet(int delay) const
{
    return 3.2808 * estimateCableLengthMeters(delay);
}

// Turn on or off DSP settle function in the FPGA.  (Executes only when CONVERT commands are sent.)
void Rhd2000EvalBoard::setDspSettle(bool enabled)
{
    lock_guard<mutex> lockOk(okMutex);

    dev->set_register_sync(WireInResetRun, (enabled ? 0x04 : 0x00), 0x04);
}

// Enable or disable one of the 32 available USB data streams (0-31).
void Rhd2000EvalBoard::enableDataStream(int stream, bool enabled)
{
    lock_guard<mutex> lockOk(okMutex);

    if (stream < 0 || stream > (MAX_NUM_DATA_STREAMS - 1)) {
        cerr << "Error in Rhd2000EvalBoard::enableDataStream: stream out of range." << endl;
        return;
    }

    if (enabled) {
        if (dataStreamEnabled[stream] == 0) {
            dev->set_register_sync(WireInDataStreamEn, 1 << stream, 1 << stream);
            dataStreamEnabled[stream] = 1;
            numDataStreams++;
        }
    } else {
        if (dataStreamEnabled[stream] == 1) {
            dev->set_register_sync(WireInDataStreamEn, 0, 1 << stream);
            dataStreamEnabled[stream] = 0;
            numDataStreams--;
        }
    }
}

// Set all 16 bits of the digital TTL output lines on the FPGA to zero.
void Rhd2000EvalBoard::clearTtlOut()
{
    lock_guard<mutex> lockOk(okMutex);

    dev->set_register_sync(WireInTtlOut, 0x0000);
}

// Set the 16 bits of the digital TTL output lines on the FPGA high or low according to integer
// array.
void Rhd2000EvalBoard::setTtlOut(int ttlOutArray[])
{
    lock_guard<mutex> lockOk(okMutex);
    int i, ttlOut;

    ttlOut = 0;
    for (i = 0; i < 16; ++i) {
        if (ttlOutArray[i] > 0) ttlOut += 1 << i;
    }
    dev->set_register_sync(WireInTtlOut, ttlOut);
}

// Read the 16 bits of the digital TTL input lines on the FPGA into an integer array.
void Rhd2000EvalBoard::getTtlIn(int ttlInArray[])
{
    lock_guard<mutex> lockOk(okMutex);
    int i, ttlIn;

    ttlIn = dev->get_register_sync(WireOutTtlIn).value();

    // kontexdev!
    for (i = 0; i < MAX_DIO; ++i) {
        ttlInArray[i] = 0;
        if ((ttlIn & (1 << i)) > 0) ttlInArray[i] = 1;
    }
}

// Set manual value for DACs.  Must run SPI commands for this value to take effect.
void Rhd2000EvalBoard::setDacManual(int value)
{
    lock_guard<mutex> lockOk(okMutex);
    if (value < 0 || value > 65535) {
        cerr << "Error in Rhd2000EvalBoard::setDacManual: value out of range." << endl;
        return;
    }

    dev->set_register_sync(WireInDacManual, value);
}

// Set the eight red LEDs on the Opal Kelly XEM6310 board according to integer array.
void Rhd2000EvalBoard::setLedDisplay(int ledArray[])
{
    lock_guard<mutex> lockOk(okMutex);
    int i, ledOut;

    ledOut = 0;
    for (i = 0; i < 8; ++i) {
        if (ledArray[i] > 0) ledOut += 1 << i;
    }
    dev->set_register_sync(WireInLedDisplay, ledOut);
}

// Set the eight red LEDs on the front panel SPI ports according to integer array.
void Rhd2000EvalBoard::setSpiLedDisplay(int ledArray[])
{
    lock_guard<mutex> lockOk(okMutex);
    int i, ledOut;

    ledOut = 0;
    for (i = 0; i < 8; ++i) {
        if (ledArray[i] > 0) ledOut += 1 << i;
    }
    dev->set_register_sync(WireInMultiUse, ledOut);
    dev->trigger(TrigInConfig, 8);
}



// Enable or disable DAC channel (0-7)
void Rhd2000EvalBoard::enableDac(int dacChannel, bool enabled)
{
    lock_guard<mutex> lockOk(okMutex);
    if (dacChannel < 0 || dacChannel > 7) {
        cerr << "Error in Rhd2000EvalBoard::enableDac: dacChannel out of range." << endl;
        return;
    }
    dev->set_register_sync(WireInDacSource[dacChannel], (enabled ? 0x0800 : 0x0000), 0x0800);
}

// Set the gain level of all eight DAC channels to 2^gain (gain = 0-7).
void Rhd2000EvalBoard::setDacGain(int gain)
{
    lock_guard<mutex> lockOk(okMutex);
    if (gain < 0 || gain > 7) {
        cerr << "Error in Rhd2000EvalBoard::setDacGain: gain setting out of range." << endl;
        return;
    }

    dev->set_register_sync(WireInResetRun, gain << 13, 0xe000);
}


// Suppress the noise on DAC channels 0 and 1 (the audio channels) between
// +16*noiseSuppress and -16*noiseSuppress LSBs.  (noiseSuppress = 0-127).
void Rhd2000EvalBoard::setAudioNoiseSuppress(int noiseSuppress)
{
    lock_guard<mutex> lockOk(okMutex);

    if (noiseSuppress < 0 || noiseSuppress > 127) {
        cerr << "Error in Rhd2000EvalBoard::setAudioNoiseSuppress: noiseSuppress out of range."
             << endl;
        return;
    }

    dev->set_register_sync(WireInResetRun, noiseSuppress << 6, 0x1fc0);
}

// Assign a particular data stream (0-31) to a DAC channel (0-7).  Setting stream
// to 32 selects DacManual value.
void Rhd2000EvalBoard::selectDacDataStream(int dacChannel, int stream)
{
    lock_guard<mutex> lockOk(okMutex);

    if (dacChannel < 0 || dacChannel > 7) {
        cerr << "Error in Rhd2000EvalBoard::selectDacDataStream: dacChannel out of range." << endl;
        return;
    }

    // kontexdev
    if (stream < 0 || stream > 33) {
        cerr << "Error in Rhd2000EvalBoard::selectDacDataStream: stream out of range." << endl;
        return;
    }
    dev->set_register_sync(WireInDacSource[dacChannel], stream << 5, 0x07e0);
}

// Assign a particular amplifier channel (0-31) to a DAC channel (0-7).
void Rhd2000EvalBoard::selectDacDataChannel(int dacChannel, int dataChannel)
{
    lock_guard<mutex> lockOk(okMutex);

    if (dacChannel < 0 || dacChannel > 7) {
        cerr << "Error in Rhd2000EvalBoard::selectDacDataChannel: dacChannel out of range." << endl;
        return;
    }

    if (dataChannel < 0 || dataChannel > 31) {
        cerr << "Error in Rhd2000EvalBoard::selectDacDataChannel: dataChannel out of range."
             << endl;
        return;
    }
    dev->set_register_sync(WireInDacSource[dacChannel], dataChannel << 0, 0x001f);
}



void Rhd2000EvalBoard::config_dac(int channel, bool enable, int source_stream, int source_channel)
{
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
    dev->set_register_sync(
        WireInDacSource[channel],
        (enable * (rhs ? 0x0200 : 0x0800)) | (source_stream << 5) | source_channel,
        rhs ? 0x3fff : 0xffff);
}

// Enable external triggering of amplifier hardware 'fast settle' function (blanking).
// If external triggering is enabled, the fast settling of amplifiers on all connected
// chips will be controlled in real time via one of the 16 TTL inputs.
void Rhd2000EvalBoard::enableExternalFastSettle(bool enable)
{
    lock_guard<mutex> lockOk(okMutex);

    dev->set_register_sync(WireInMultiUse, enable ? 1 : 0);
    dev->trigger(TrigInConfig, 6);
}

// Select which of the TTL inputs 0-15 is used to perform a hardware 'fast settle' (blanking)
// of the amplifiers if external triggering of fast settling is enabled.
void Rhd2000EvalBoard::setExternalFastSettleChannel(int channel)
{
    lock_guard<mutex> lockOk(okMutex);

    if (channel < 0 || channel > 15) {
        cerr << "Error in Rhd2000EvalBoard::setExternalFastSettleChannel: channel out of range."
             << endl;
        return;
    }

    dev->set_register_sync(WireInMultiUse, channel);
    dev->trigger(TrigInConfig, 7);
}

// Enable external control of RHD2000 auxiliary digital output pin (auxout).
// If external control is enabled, the digital output of all chips connected to a
// selected SPI port will be controlled in real time via one of the 16 TTL inputs.
void Rhd2000EvalBoard::enableExternalDigOut(SPIPort port, bool enable)
{
    lock_guard<mutex> lockOk(okMutex);
    if (port == SPIPort::All) {
        for (int i = 0; i < ports.num_of_spi; ++i) {
            dev->set_register_sync(WireInMultiUse, (int) enable);
            dev->trigger(TrigInDacConfig, 16 + i);
        }
    } else {
        dev->set_register_sync(WireInMultiUse, (int) enable);
        dev->trigger(TrigInDacConfig, 16 + static_cast<int>(port));
    }
}

// Select which of the TTL inputs 0-15 is used to control the auxiliary digital output
// pin of the chips connected to a particular SPI port, if external control of auxout is enabled.
void Rhd2000EvalBoard::setExternalDigOutChannel(SPIPort port, int channel)
{
    lock_guard<mutex> lockOk(okMutex);

    if (channel < 0 || channel > 15) {
        cerr << "Error in Rhd2000EvalBoard::setExternalDigOutChannel: channel out of range.\n";
        return;
    }
    if (port == SPIPort::All) {
        for (int i = 0; i < ports.num_of_spi; ++i) {
            dev->set_register_sync(WireInMultiUse, channel);
            dev->trigger(TrigInDacConfig, 24 + i);
        }
    } else {
        dev->set_register_sync(WireInMultiUse, channel);
        dev->trigger(TrigInDacConfig, 24 + static_cast<int>(port));
    }
}

// Enable optional FPGA-implemented digital high-pass filters associated with DAC outputs
// on USB interface board.. These one-pole filters can be used to record wideband neural data
// while viewing only spikes without LFPs on the DAC outputs, for example.  This is useful when
// using the low-latency FPGA thresholds to detect spikes and produce digital pulses on the TTL
// outputs, for example.
void Rhd2000EvalBoard::enableDacHighpassFilter(bool enable)
{
    lock_guard<mutex> lockOk(okMutex);

    dev->set_register_sync(WireInMultiUse, enable ? 1 : 0);
    dev->trigger(TrigInConfig, 4);
}

// Set cutoff frequency (in Hz) for optional FPGA-implemented digital high-pass filters
// associated with DAC outputs on USB interface board.  These one-pole filters can be used
// to record wideband neural data while viewing only spikes without LFPs on the DAC outputs,
// for example.  This is useful when using the low-latency FPGA thresholds to detect spikes
// and produce digital pulses on the TTL outputs, for example.
void Rhd2000EvalBoard::setDacHighpassFilter(double cutoff)
{
    lock_guard<mutex> lockOk(okMutex);

    double b;
    int filterCoefficient;
    const double pi = 3.1415926535897;

    // Note that the filter coefficient is a function of the amplifier sample rate, so this
    // function should be called after the sample rate is changed.
    b = 1.0 - exp(-2.0 * pi * cutoff / getSampleRate());

    // In hardware, the filter coefficient is represented as a 16-bit number.
    filterCoefficient = (int) floor(65536.0 * b + 0.5);

    if (filterCoefficient < 1) {
        filterCoefficient = 1;
    } else if (filterCoefficient > 65535) {
        filterCoefficient = 65535;
    }

    dev->set_register_sync(WireInMultiUse, filterCoefficient);
    dev->trigger(TrigInConfig, 5);
}

// Set thresholds for DAC channels; threshold output signals appear on TTL outputs 0-7.
// The parameter 'threshold' corresponds to the RHD2000 chip ADC output value, and must fall
// in the range of 0 to 65535, where the 'zero' level is 32768.
// If trigPolarity is true, voltages equaling or rising above the threshold produce a high TTL
// output. If trigPolarity is false, voltages equaling or falling below the threshold produce a high
// TTL output.
void Rhd2000EvalBoard::setDacThreshold(int dacChannel, int threshold, bool trigPolarity)
{
    lock_guard<mutex> lockOk(okMutex);

    if (dacChannel < 0 || dacChannel > 7) {
        cerr << "Error in Rhd2000EvalBoard::setDacThreshold: dacChannel out of range." << endl;
        return;
    }

    if (threshold < 0 || threshold > 65535) {
        cerr << "Error in Rhd2000EvalBoard::setDacThreshold: threshold out of range." << endl;
        return;
    }

    // Set threshold level.
    dev->set_register_sync(WireInMultiUse, threshold);
    dev->trigger(TrigInDacConfig, dacChannel);

    // Set threshold polarity.
    dev->set_register_sync(WireInMultiUse, (trigPolarity ? 1 : 0));
    dev->trigger(TrigInDacConfig, dacChannel + 8);
}

// Set the TTL output mode of the board.
// mode = 0: All 16 TTL outputs are under manual control
// mode = 1: Top 8 TTL outputs are under manual control;
//           Bottom 8 TTL outputs are outputs of DAC comparators
void Rhd2000EvalBoard::setTtlMode(int mode)
{
    lock_guard<mutex> lockOk(okMutex);

    if (mode < 0 || mode > 1) {
        cerr << "Error in Rhd2000EvalBoard::setTtlMode: mode out of range." << endl;
        return;
    }

    dev->set_register_sync(WireInResetRun, mode << 3, 0x0008);
}

// Is variable-frequency clock DCM programming done?
bool Rhd2000EvalBoard::isDcmProgDone() const
{
    int value;

    value = dev->get_register_sync(WireOutDataClkLocked).value();

    return ((value & 0x0002) > 1);
}

// Is variable-frequency clock PLL locked?
bool Rhd2000EvalBoard::isDataClockLocked() const
{
    int value;

    value = dev->get_register_sync(WireOutDataClkLocked).value();

    return ((value & 0x0001) > 0);
}

// Flush all remaining data out of the FIFO.  (This function should only be called when SPI
// data acquisition has been stopped.)
void Rhd2000EvalBoard::flush()
{
    lock_guard<mutex> lockOk(okMutex);
    dev->read(0xA0, 0, nullptr);
}

std::expected<Rhd2000DataBlock, std::string> Rhd2000EvalBoard::run_and_read_samples(
    int samples, std::optional<std::chrono::milliseconds> timeout)
{
    const auto sample_size = get_sample_size<char>();

    constexpr int hw_events_per_sec = 100;
    const auto expected_data_rate = sample_size * getSampleRate();
    const int chunk_size = expected_data_rate / hw_events_per_sec;

    std::promise<std::expected<Rhd2000DataBlock, std::string>> result_promise;

    auto result = result_promise.get_future();

    auto s = dev->start_read_stream(
        PipeOutData,
        xdaq::DataStream::queue(
            [this, sample_size, samples = samples,
             result_promise = std::make_optional(std::move(result_promise)),
             buffer = std::vector<unsigned char>(),
             streams = numDataStreams](auto &&event) mutable {
                if (!result_promise.has_value()) return;
                std::visit(
                    [&](auto &&event) {
                        using T = std::decay_t<decltype(event)>;
                        using namespace xdaq::DataStream::Events;
                        if constexpr (std::is_same_v<T, Stop>) {
                        } else if constexpr (std::is_same_v<T, Error>) {
                            result_promise->set_value(
                                std::unexpected(fmt::format("Datastream Error {}", event.error)));
                            result_promise.reset();
                        } else if constexpr (std::is_same_v<T, DataView>) {
                        } else if constexpr (std::is_same_v<T, OwnedData>) {
                            std::copy(event.buffer.get(),
                                      event.buffer.get() +
                                          std::min<std::size_t>(
                                              sample_size * samples - buffer.size(), event.length),
                                      std::back_inserter(buffer));
                            if (buffer.size() == sample_size * samples) {
                                result_promise->set_value(
                                    Rhd2000DataBlock(numDataStreams, samples, buffer.data()));
                                result_promise.reset();
                            }
                        } else {
                            static_assert(xdaq::always_false_v<T>, "non-exhaustive visitor");
                        }
                    },
                    std::move(event));
            },
            32, 4096, std::chrono::nanoseconds{0}),
        chunk_size);
    auto bytes_required = sample_size * samples;
    setMaxTimeStep(((bytes_required + chunk_size - 1) / chunk_size * chunk_size + sample_size - 1) /
                   sample_size);
    setContinuousRunMode(false);
    run();
    const auto expected_sample_time =
        std::chrono::milliseconds{(int) (1000 * samples * sample_size / getSampleRate())};
    auto wait_result = result.wait_for(expected_sample_time + 2s);
    s->reset();
    while (isRunning()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    flush();

    if (wait_result == std::future_status::timeout)
        return std::unexpected("Read Timeout");
    else
        return result.get();
}


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

void Rhd2000EvalBoard::setAllDacsToZero()
{
    setDacManual(32768);  // midrange value = 0 V
    for (int i = 0; i < 8; i++) {
        selectDacDataStream(i, 32);
    }
}

// Selects an amplifier channel from a particular data stream to be subtracted from all DAC signals.
void Rhd2000EvalBoard::setDacRerefSource(int stream, int channel)
{
    lock_guard<mutex> lockOk(okMutex);

    if (stream < 0 || stream > (MAX_NUM_DATA_STREAMS - 1)) {
        cerr << "Error in Rhd2000EvalBoard::setDacRerefSource: stream out of range." << endl;
        return;
    }

    if (channel < 0 || channel > 31) {
        cerr << "Error in Rhd2000EvalBoard::setDacRerefSource: channel out of range." << endl;
        return;
    }

    dev->set_register_sync(WireInDacReref, (stream << 5) + channel, 0x0000003ff);
}

// Enables DAC rereferencing, where a selected amplifier channel is subtracted from all DACs in real
// time.
void Rhd2000EvalBoard::enableDacReref(bool enabled)
{
    lock_guard<mutex> lockOk(okMutex);

    dev->set_register_sync(WireInDacReref, (enabled ? 0x00000400 : 0x00000000), 0x00000400);
}

template <typename OS>
void print_delay_results(OS &os, const vector<vector<std::optional<IntanChip::Chip>>> &scan_results,
                         const vector<int> &optimal_delays)
{
    for (int i = 0; i < scan_results.size(); i++) {
        os << fmt::format("Delay {:2d}: ", i);
        for (int j = 0; j < scan_results[i].size(); j++) {
            if (scan_results[i][j]) {
                os << fmt::format("{:2d};", (int) scan_results[i][j]->id);
            } else {
                os << "NA;";
            }
        }
        os << "\n";
    }
    os << "Optimal : ";
    for (auto d : optimal_delays) os << fmt::format("{:2d};", d);
    os << '\n';
}

namespace CabelDelayAlgorithms
{
/**
 * @brief Find the delay by counting the number of valid delays for each stream
 *        and choosing the first valid delay if number of valid delays is 1 or 2.
 *        If number of valid delays is greater than 2, choose the second valid delay.
 * @param scan_results Delays x Streams
 * @return vector<int>  optimal delays for each stream
 */
vector<int> find_optimal_delays(const vector<vector<std::optional<IntanChip::Chip>>> &scan_results)
{
    const int n_delays = scan_results.size();
    const int n_streams = scan_results[0].size();
    vector<int> optimal_delays(scan_results[0].size(), -1);
    vector<int> second_valid_delays(scan_results[0].size(), -1);
    for (int stream = 0; stream < n_streams; ++stream) {
        int valid_delays = 0;
        for (int delay = 0; delay < n_delays; ++delay) {
            if (scan_results[delay][stream]) {
                if (valid_delays == 0)
                    optimal_delays[stream] = delay;
                else if (valid_delays == 1)
                    second_valid_delays[stream] = delay;
                else if (valid_delays == 2)
                    optimal_delays[stream] = second_valid_delays[stream];
                else
                    break;
                ++valid_delays;
            }
        }
    }
    return optimal_delays;
}

/**
 * @brief Find the delay by counting the number of valid delays for each stream
 *        and choosing the first valid delay.
 * @param scan_results Delays x Streams
 * @return vector<int>  optimal delays for each stream
 */
vector<int> find_optimal_delays_greedy(
    const vector<vector<std::optional<IntanChip::Chip>>> &scan_results)
{
    const int n_delays = scan_results.size();
    const int n_streams = scan_results[0].size();
    vector<int> optimal_delays(scan_results[0].size(), -1);
    for (int stream = 0; stream < n_streams; ++stream) {
        for (int delay = 0; delay < n_delays; ++delay) {
            if (scan_results[delay][stream]) {
                optimal_delays[stream] = delay;
                break;
            }
        }
    }
    return optimal_delays;
}
}  // namespace CabelDelayAlgorithms

const std::vector<IntanChip::Chip> &Rhd2000EvalBoard::scan_chips()
{
    if (false) {
        for (int i = 0; i < ports.max_streams; ++i) enableDataStream(i, true);
        for (int i = 0; i < ports.max_chips; ++i) {
            if (i != 8 && i != 9) {
                chips[i].id = IntanChip::ChipID::RHD2164;
                chips[i].miso = i % 2 == 0 ? IntanChip::ChipMISO::A : IntanChip::ChipMISO::B;
            } else {
                chips[i].id = IntanChip::ChipID::RHD2132;
                chips[i].miso = IntanChip::ChipMISO::NA;
            }
        }
        setCableDelay(Rhd2000EvalBoard::SPIPort::All, 1);

        return chips;
    }
    for (int i = 0; i < ports.max_streams; ++i) enableDataStream(i, !ports.is_ddr(i));

    // Read the Intan chip ID number from each RHD2000 chip found.
    // Record delay settings that yield good communication with the chip.
    vector<vector<optional<IntanChip::Chip>>> scan_results(16);
    for (int delay = 0; delay < 16; ++delay) {
        setCableDelay(Rhd2000EvalBoard::SPIPort::All, delay);
        auto db = run_and_read_samples(SAMPLES_PER_DATA_BLOCK);
        if (db) {
            for (int stream = 0; stream < ports.max_non_ddr_streams; ++stream) {
                scan_results[delay].push_back(
                    IntanChip::parse_device_id(&db->aux[2][stream * db->num_samples]));
            }
        } else {
            throw runtime_error("Failed to read data block");
        }
    }

    std::vector<int> optimal_delays =
        CabelDelayAlgorithms::find_optimal_delays_greedy(scan_results);
    int port_idx = 0;
    for (auto s = optimal_delays.begin(); s != optimal_delays.end();
         s += ports.max_non_ddr_streams_per_spi) {
        int max_delay = *std::max_element(s, s + ports.max_non_ddr_streams_per_spi);
        const auto spi = static_cast<Rhd2000EvalBoard::SPIPort>(port_idx++);
        setCableDelay(spi, max_delay);
    }
    std::vector<int> led(8, 0);
    for (int i = 0; i < ports.num_of_ports; ++i) {
        auto s = optimal_delays.begin() + i * ports.max_non_ddr_streams_per_port;
        led[i] = *std::max_element(s, s + ports.max_non_ddr_streams_per_port) > 0;
    }
    setSpiLedDisplay(&led[0]);

    for (int i = 0; i < ports.max_chips; ++i) {
        if (optimal_delays[i] == -1)
            chips[i] = IntanChip::Chip();
        else
            chips[i] = *scan_results[optimal_delays[i]][i];
    }
    for (int i = 0; i < ports.max_streams; ++i) enableDataStream(i, false);
    // print_delay_results(std::cout, scan_results, optimal_delays);
    return chips;
}