//----------------------------------------------------------------------------------
// rhd2000evalboardusb3.h
//
// Intan Technoloies RHD2000 USB3 Rhythm Interface API
// Rhd2000EvalBoard Class Header File
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
#pragma once

#define RHYTHM_BOARD_ID 700
#define MAX_NUM_DATA_STREAMS 32
#define MAX_NUM_SPI_PORTS 8
#define RHD_BOARD_MODE 13

// The maximum number of Rhd2000DataBlockUsb3 objects we will need is set by the need
// to perform electrode impedance measurements at very low frequencies.
// (Maximum command length = 1024 for one period; seven periods required in worst case.)
#define MAX_NUM_BLOCKS 57

#define FIFO_CAPACITY_WORDS 67108864

#define USB3_BLOCK_SIZE 1024
#define RAM_BURST_SIZE 32

#define MAX_DIO 32

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <zmq.hpp>

// #include "okFrontPanelDLL.h"
#include "ports.h"
#include "rhd2000datablock.h"

class Rhd2000EvalBoard
{
public:
    Ports ports = XDAQPortRHD();
    Rhd2000EvalBoard();
    Rhd2000EvalBoard(const Rhd2000EvalBoard &) = delete;
    Rhd2000EvalBoard &operator=(const Rhd2000EvalBoard &) = delete;

    ~Rhd2000EvalBoard();

    template <typename Unit>
    static constexpr std::size_t fifo_capacity()
    {
        return (1u << 27) / sizeof(Unit);
    }


    int open(const char *libname);

    int FPGA_board;
    bool usb3;

    enum class SampleRate {
        s1000Hz,
        s1250Hz,
        s1500Hz,
        s2000Hz,
        s2500Hz,
        s3000Hz,
        s3333Hz,
        s4000Hz,
        s5000Hz,
        s6250Hz,
        s8000Hz,
        s10000Hz,
        s12500Hz,
        s15000Hz,
        s20000Hz,
        s25000Hz,
        s30000Hz
    };

    bool setSampleRate(SampleRate newSampleRate);
    double getSampleRate() const;
    SampleRate getSampleRateEnum() const;

    enum class AuxCmdSlot { All = -1, AuxCmd1 = 0, AuxCmd2 = 1, AuxCmd3 = 2 };

    enum class SPIPort {
        All = -1,
        PortA = 0,
        PortB = 1,
        PortC = 2,
        PortD = 3,
        PortE = 4,
        PortF = 5,
        PortG = 6,
        PortH = 7
    };

    enum HdmiPort {
        PortA,
        PortB,
        PortC,
        PortD,
    };

    void uploadCommandList(const std::vector<uint32_t> &commandList, AuxCmdSlot auxCommandSlot,
                           int bank);

    void selectAuxCommandBank(SPIPort port, AuxCmdSlot auxCommandSlot, int bank);
    void selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex, int endIndex);

    void setMaxTimeStep(unsigned int maxTimeStep);
    void run(std::function<void(std::span<const std::byte>)> callback);
    void stop();
    bool isRunning();

    void enableDataStream(int stream, bool enabled);
    int getNumEnabledDataStreams() const { return numDataStreams; }

    void setTtlOut(int ttlOutArray[]);

    void setSpiLedDisplay(int ledArray[]);

    void enableDac(int dacChannel, bool enabled);
    void setDacGain(int gain);
    void setAudioNoiseSuppress(int noiseSuppress);

    /**
     * @brief
     * M = Max stream, 32 for RHD, 8 for RHS
     * To enable the DAC, the enable bit must be set high.
     * For each DAC, user may select an amplifier channel from a data stream (0 ~ M-1).
     * If source is set to M, the DAC will be controlled directly by the host computer via
     * WireInDacManual; the source_channel parameter is ignored in this case. XDAQ : When source
     * stream is set to M+1, the DAC will controlled by user defined waveform uploaded via
     * UploadWaveform.
     *
     * @param channel : DAC channel
     * @param enable : true to enable DAC
     * @param source_stream : 0 ~ M-1, M for manual, M+1 for user defined waveform
     * @param source_channel : 0 ~ 31 when source_stream is 0 ~ M-1, ignored otherwise
     */
    void config_dac(int channel, bool enable, int source_stream, int source_channel);
    void enableExternalFastSettle(bool enable);
    void setExternalFastSettleChannel(int channel);
    void enableExternalDigOut(SPIPort port, bool enable);
    void setExternalDigOutChannel(SPIPort port, int channel);
    void enableDacHighpassFilter(bool enable);
    void setDacHighpassFilter(double cutoff);
    void setDacThreshold(int dacChannel, int threshold, bool trigPolarity);
    void setTtlMode(int mode);

    template <typename Unit>
    std::size_t get_sample_size()
    {
        return sample_size<Unit>(numDataStreams, CHANNELS_PER_STREAM, dio32);
    }

    template <typename Unit>
    std::size_t get_fifo_data_size(bool update)
    {
        return (update ? numWordsInFifo() : lastNumWordsInFifo) * 2 / sizeof(Unit);
    }

    int get_num_samples_available(bool update)
    {
        return get_fifo_data_size<char>(update) / get_sample_size<char>();
    }

    // std::optional<Rhd2000DataBlock> run_and_read_samples(
    //     int samples, std::optional<std::chrono::milliseconds> timeout = std::nullopt);
    // std::optional<Rhd2000DataBlock> read_samples(int samples);

    // int read_to_buffer(int samples, unsigned char *buffer);


    int getCableDelay(SPIPort port) const;
    void getCableDelay(std::vector<int> &delays) const;

    // kontexdev
    bool set_dio32(bool dio32);
    bool get_dio32() const { return dio32; }
    bool expander_present() const { return expander; }

    enum class XDAQStatus : uint8_t {
        MCU_IDLE = 0x0,
        MCU_BUSY = 0x04,
        MCU_DONE = 0x08,
        MCU_ERROR = 0x10
    };
    bool isStreamEnabled(int stream) const { return dataStreamEnabled[stream]; }
    const std::vector<IntanChip::Chip> &scan_chips();
    const std::vector<IntanChip::Chip> &get_chips() const { return chips; }

private:
    zmq::context_t context;
    std::thread receiving_thread;
    zmq::socket_t socket;
    zmq::socket_t data_socket;
    std::atomic_bool running = false;
    const int samples_per_block = SAMPLES_PER_DATA_BLOCK;
    long read_raw_samples(int samples, unsigned char *buffer);

    bool is_open = false;

    bool expander = false;

    bool dio32 = false;
    SampleRate sampleRate = SampleRate::s30000Hz;
    int numDataStreams = 0;  // total number of data streams currently enabled
    int dataStreamEnabled[MAX_NUM_DATA_STREAMS] = {0};  // 0 (disabled) or 1 (enabled)
    std::vector<int> cableDelay;
    std::vector<IntanChip::Chip> chips;

    // Methods in this class are designed to be thread-safe.  This variable is used to ensure that.
    std::mutex okMutex;

    unsigned int lastNumWordsInFifo = 0;
    unsigned int numWordsInFifo();
};