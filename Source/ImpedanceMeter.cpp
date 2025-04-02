/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2021 Open Ephys

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

#include "ImpedanceMeter.h"

#include "rhythm-api/rhd2000datablock.h"
#include "rhythm-api/rhd2000evalboard.h"

using namespace RhythmNode;

#define PI 3.14159265359
#define TWO_PI 6.28318530718
#define DEGREES_TO_RADIANS 0.0174532925199
#define RADIANS_TO_DEGREES 57.2957795132

// Allocates memory for a 3-D array of doubles.
void allocateDoubleArray3D(std::vector<std::vector<std::vector<double>>> &array3D, int xSize,
                           int ySize, int zSize)
{
    int i, j;

    if (xSize == 0) return;
    array3D.resize(xSize);
    for (i = 0; i < xSize; ++i) {
        array3D[i].resize(ySize);
        for (j = 0; j < ySize; ++j) {
            array3D[i][j].resize(zSize);
        }
    }
}

ImpedanceMeter::ImpedanceMeter(DeviceThread *board_)
    : ThreadWithProgressWindow("RHD2000 Impedance Measurement", true, true), board(board_)
{
}

ImpedanceMeter::~ImpedanceMeter() { stopThreadSafely(); }

void ImpedanceMeter::stopThreadSafely()
{
    if (isThreadRunning()) {
        CoreServices::sendStatusMessage("Impedance measurement in progress. Stopping it.");

        if (!stopThread(3000))  // wait three seconds max for it to exit gracefully
        {
            std::cerr << "ERROR: Impedance measurement did not exit." << std::endl;
        }
    }
}

void ImpedanceMeter::waitSafely()
{
    if (!waitForThreadToExit(120000))  // two minutes should be enough for completing a scan
    {
        CoreServices::sendStatusMessage("Impedance measurement took too much time. Aborting.");

        if (!stopThread(3000))  // wait three seconds max for it to exit gracefully
        {
            std::cerr << "ERROR: Impedance measurement thread did not exit." << std::endl;
        }
    }
}

float ImpedanceMeter::updateImpedanceFrequency(float desiredImpedanceFreq, bool &impedanceFreqValid)
{
    int impedancePeriod;
    double lowerBandwidthLimit, upperBandwidthLimit;
    float actualImpedanceFreq;

    upperBandwidthLimit = board->settings.dsp.upperBandwidth / 1.5;
    lowerBandwidthLimit = board->settings.dsp.lowerBandwidth * 1.5;

    if (board->settings.dsp.enabled) {
        if (board->settings.dsp.cutoffFreq > board->settings.dsp.lowerBandwidth) {
            lowerBandwidthLimit = board->settings.dsp.cutoffFreq * 1.5;
        }
    }

    if (desiredImpedanceFreq > 0.0) {
        impedancePeriod = (board->settings.boardSampleRate / desiredImpedanceFreq);
        if (impedancePeriod >= 4 && impedancePeriod <= 1024 &&
            desiredImpedanceFreq >= lowerBandwidthLimit &&
            desiredImpedanceFreq <= upperBandwidthLimit) {
            actualImpedanceFreq = board->settings.boardSampleRate / impedancePeriod;
            impedanceFreqValid = true;
        } else {
            actualImpedanceFreq = 0.0;
            impedanceFreqValid = false;
        }
    } else {
        actualImpedanceFreq = 0.0;
        impedanceFreqValid = false;
    }

    return actualImpedanceFreq;
}



/** Returns the real and imaginary amplitudes of a selected frequency component in the vector
    data, between a start index and end index. */
template <typename T>
std::complex<double> amplitudeOfFreqComponent(const std::span<T> data, int startIndex, int endIndex,
                                              double sampleRate, double frequency)
{
    int length = endIndex - startIndex + 1;
    const double k = TWO_PI * frequency / sampleRate;  // precalculate for speed

    // Perform correlation with sine and cosine waveforms.
    double meanI = 0.0;
    double meanQ = 0.0;
    for (int t = startIndex; t <= endIndex; ++t) {
        meanI += data[t] * cos(k * t);
        meanQ += data[t] * -1.0 * sin(k * t);
    }
    meanI /= (double) length;
    meanQ /= (double) length;

    return {2.0 * meanI, 2.0 * meanQ};
}



std::complex<double> ImpedanceMeter::measureComplexAmplitude(std::span<float> ampdata,
                                                             int numBlocks, double sampleRate,
                                                             double frequency, int numPeriods)
{
    const int period = (sampleRate / frequency);
    int startIndex = 0;
    int endIndex = startIndex + numPeriods * period - 1;

    // Move the measurement window to the end of the waveform to ignore start-up transient.
    while (endIndex < SAMPLES_PER_DATA_BLOCK * numBlocks - period) {
        startIndex += period;
        endIndex += period;
    }

    // Measure real (iComponent) and imaginary (qComponent) amplitude of frequency component.
    return amplitudeOfFreqComponent(ampdata, startIndex, endIndex, sampleRate, frequency);
}



/** Given a measured complex impedance that is the result of an electrode impedance in parallel
    with a parasitic capacitance (i.e., due to the amplifier input capacitance and other
    capacitances associated with the chip bondpads), this function factors out the effect of the
    parasitic capacitance to return the acutal electrode impedance. */
void factorOutParallelCapacitance(double &impedanceMagnitude, double &impedancePhase,
                                  double frequency, double parasiticCapacitance)
{
    // First, convert from polar coordinates to rectangular coordinates.
    double measuredR = impedanceMagnitude * cos(DEGREES_TO_RADIANS * impedancePhase);
    double measuredX = impedanceMagnitude * sin(DEGREES_TO_RADIANS * impedancePhase);

    double capTerm = TWO_PI * frequency * parasiticCapacitance;
    double xTerm = capTerm * (measuredR * measuredR + measuredX * measuredX);
    double denominator = capTerm * xTerm + 2 * capTerm * measuredX + 1;
    double trueR = measuredR / denominator;
    double trueX = (measuredX + xTerm) / denominator;

    // Now, convert from rectangular coordinates back to polar coordinates.
    impedanceMagnitude = sqrt(trueR * trueR + trueX * trueX);
    impedancePhase = RADIANS_TO_DEGREES * atan2(trueX, trueR);
}


/** This is a purely empirical function to correct observed errors in the real component
    of measured electrode impedances at sampling rates below 15 kS/s.  At low sampling rates,
    it is difficult to approximate a smooth sine wave with the on-chip voltage DAC and 10 kHz
    2-pole lowpass filter.  This function attempts to somewhat correct for this, but a better
    solution is to always run impedance measurements at 20 kS/s, where they seem to be most
    accurate. */
void empiricalResistanceCorrection(double &impedanceMagnitude, double &impedancePhase,
                                   double boardSampleRate)
{
    // First, convert from polar coordinates to rectangular coordinates.
    double impedanceR = impedanceMagnitude * cos(DEGREES_TO_RADIANS * impedancePhase);
    double impedanceX = impedanceMagnitude * sin(DEGREES_TO_RADIANS * impedancePhase);

    // Emprically derived correction factor (i.e., no physical basis for this equation).
    impedanceR /=
        10.0 * exp(-boardSampleRate / 2500.0) * cos(TWO_PI * boardSampleRate / 15000.0) + 1.0;

    // Now, convert from rectangular coordinates back to polar coordinates.
    impedanceMagnitude = sqrt(impedanceR * impedanceR + impedanceX * impedanceX);
    impedancePhase = RADIANS_TO_DEGREES * atan2(impedanceX, impedanceR);
}


void ImpedanceMeter::run()
{
    auto res = runImpedanceMeasurement();

    restoreBoardSettings();

    board->update_impedances(std::move(res));

    setProgress(1.0f);
}

#define CHECK_EXIT \
    if (threadShouldExit()) return std::nullopt;

std::optional<Impedances> ImpedanceMeter::runImpedanceMeasurement()
{
    // Impedances impedances;
    // int numdataStreams = board->evalBoard->getNumEnabledDataStreams();
    // for (int s = 0; s < board->evalBoard->ports.max_streams; ++s) {
    //     if (!board->evalBoard->isStreamEnabled(s)) continue;
    //     auto &chip = board->evalBoard->get_chips()[s / 2];
    //     if (chip.id != IntanChip::ChipID::RHD2164) {
    //         impedances.stream_indices.push_back(s);
    //         impedances.magnitudes_by_stream.emplace_back(32, s * 1000 + 0.1);
    //         impedances.phases_by_stream.emplace_back();
    //         for (int c = 0; c < 32; ++c)
    //             impedances.phases_by_stream.back().push_back(-0.01 * c - 1);
    //     } else {
    //         impedances.stream_indices.push_back(s);
    //         impedances.magnitudes_by_stream.emplace_back(32, s * 1000 + 0.2);
    //         impedances.phases_by_stream.emplace_back();
    //         for (int c = 0; c < 32; ++c)
    //             impedances.phases_by_stream.back().push_back(-0.01 * c - 2);
    //     }
    // }
    // return impedances;
    setProgress(0.0f);

    int numdataStreams = board->evalBoard->getNumEnabledDataStreams();

    bool rhd2164ChipPresent = false;

    std::vector<int> enabledStreams;

    for (int stream = 0; stream < MAX_NUM_DATA_STREAMS; ++stream) {
        if (board->evalBoard->isStreamEnabled(stream)) {
            enabledStreams.push_back(stream);
        }

        if (board->chipId[stream] == CHIP_ID_RHD2164_B) {
            rhd2164ChipPresent = true;
        }
    }

    bool validImpedanceFreq;
    float actualImpedanceFreq = updateImpedanceFrequency(1000.0, validImpedanceFreq);

    if (!validImpedanceFreq) {
        return std::nullopt;
    }

    // Create a command list for the AuxCmd1 slot.
    {
        auto commands = board->chipRegisters.createCommandListZcheckDac(actualImpedanceFreq, 128.0);
        CHECK_EXIT;
        board->evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 1);
        board->evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 0,
                                                 commands.size() - 1);
    }

    if (board->settings.fastTTLSettleEnabled) {
        board->evalBoard->enableExternalFastSettle(false);
    }

    CHECK_EXIT;
    board->evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                           Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 1);


    const int numPeriods = std::max<int>(  // Select number of periods to measure impedance over
        0.020 * actualImpedanceFreq,       // Test each channel for at least 20 msec...
        5  // ...but always measure across no fewer than 5 complete periods
    );

    double period = board->settings.boardSampleRate / actualImpedanceFreq;
    const int numBlocks = std::max<int>(
        ceil((numPeriods + 2.0) * period / 60.0),  // + 2 periods to give time to settle initially
        2  // need first block for command to switch channels to take effect.
    );

    CHECK_EXIT;
    board->settings.dsp.cutoffFreq =
        board->chipRegisters.setDspCutoffFreq(board->settings.dsp.cutoffFreq);
    board->settings.dsp.lowerBandwidth =
        board->chipRegisters.setLowerBandwidth(board->settings.dsp.lowerBandwidth);
    board->settings.dsp.upperBandwidth =
        board->chipRegisters.setUpperBandwidth(board->settings.dsp.upperBandwidth);
    board->chipRegisters.enableDsp(board->settings.dsp.enabled);
    board->chipRegisters.enableZcheck(true);

    {
        const auto commands = board->chipRegisters.createCommandListRegisterConfig(false);
        CHECK_EXIT;
        // Upload version with no ADC calibration to AuxCmd3 RAM Bank 1.
        board->evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 3);
        board->evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 0,
                                                 commands.size() - 1);
        board->evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                               Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 3);
    }

    CHECK_EXIT;
    board->evalBoard->setContinuousRunMode(false);
    board->evalBoard->setMaxTimeStep(SAMPLES_PER_DATA_BLOCK * numBlocks);
    std::vector<unsigned char> buffer(SAMPLES_PER_DATA_BLOCK * numBlocks *
                                      board->evalBoard->get_sample_size<char>());

    // Create matrices of doubles of size (numStreams x 32 x 3) to store complex amplitudes
    // of all amplifier channels (32 on each data stream) at three different Cseries values.
    std::vector<std::vector<std::vector<double>>> measuredMagnitude;
    std::vector<std::vector<std::vector<double>>> measuredPhase;

    measuredMagnitude.resize(board->evalBoard->getNumEnabledDataStreams());
    measuredPhase.resize(board->evalBoard->getNumEnabledDataStreams());

    for (int i = 0; i < board->evalBoard->getNumEnabledDataStreams(); ++i) {
        measuredMagnitude[i].resize(32);
        measuredPhase[i].resize(32);

        for (int j = 0; j < 32; ++j) {
            measuredMagnitude[i][j].resize(3);
            measuredPhase[i][j].resize(3);
        }
    }

    double distance, current, Cseries;
    double impedanceMagnitude, impedancePhase;

    double relativeFreq = actualImpedanceFreq / board->settings.boardSampleRate;


    // We execute three complete electrode impedance measurements: one each with
    // Cseries set to 0.1 pF, 1 pF, and 10 pF.  Then we select the best measurement
    // for each channel so that we achieve a wide impedance measurement range.
    for (int capRange = 0; capRange < 3; ++capRange) {
        switch (capRange) {
        case 0: board->chipRegisters.setZcheckScale(Rhd2000Registers::ZcheckCs100fF); break;
        case 1: board->chipRegisters.setZcheckScale(Rhd2000Registers::ZcheckCs1pF); break;
        case 2: board->chipRegisters.setZcheckScale(Rhd2000Registers::ZcheckCs10pF); break;
        }

        // Check all 32 channels across all active data streams.
        for (int channel = 0; channel < 32 + (32 * rhd2164ChipPresent); ++channel) {
            CHECK_EXIT;

            board->chipRegisters.setZcheckChannel(channel);

            const auto commands = board->chipRegisters.createCommandListRegisterConfig(false);
            // Upload version with no ADC calibration to AuxCmd3 RAM Bank 1.
            board->evalBoard->uploadCommandList(commands, Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3, 3);
            auto res = board->evalBoard->run_and_read_samples(SAMPLES_PER_DATA_BLOCK * numBlocks);
            if (!res.has_value()) {
                LOGE(fmt::format("Read error {}", res.error()));
                return std::nullopt;
            }

            for (int stream = 0; stream < numdataStreams; ++stream) {
                setProgress(float(capRange) / 3.0f +
                            (float(channel) / (32 + 32 * rhd2164ChipPresent) / 3.0f) +
                            (float(stream) / float(numdataStreams) / 32.0f / 3.0f));

                const auto r = measureComplexAmplitude(
                    {&res.value().amp[(stream * 32 + channel) * SAMPLES_PER_DATA_BLOCK * numBlocks],
                     static_cast<std::size_t>(SAMPLES_PER_DATA_BLOCK * numBlocks)},
                    numBlocks, board->settings.boardSampleRate, actualImpedanceFreq, numPeriods);
                measuredMagnitude[stream][channel % 32][capRange] = std::abs(r);
                measuredPhase[stream][channel % 32][capRange] = std::arg(r) * RADIANS_TO_DEGREES;
            }
        }
    }

    Impedances impedances;

    // we favor voltage readings that are closest to 250 uV: not too large, and not too small.
    const double bestAmplitude = 250.0;

    const double dacVoltageAmplitude =
        128 * (1.225 / 256);  // this assumes the DAC amplitude was set to 128
    const double parasiticCapacitance =
        14.0e-12;  // 14 pF: an estimate of on-chip parasitic capacitance,
    // including 10 pF of amplifier input capacitance.
    for (int stream = 0; stream < board->evalBoard->getNumEnabledDataStreams(); ++stream) {
        impedances.stream_indices.push_back(enabledStreams[stream]);
        impedances.magnitudes_by_stream.push_back(std::vector<float>());
        impedances.phases_by_stream.push_back(std::vector<float>());
        for (int channel = 0; channel < 32; ++channel) {
            int bestAmplitudeIndex;
            double minDistance = 9.9e99;  // ridiculously large number
            for (int capRange = 0; capRange < 3; ++capRange) {
                // Find the measured amplitude that is closest to bestAmplitude on a logarithmic
                // scale
                distance = abs(log(measuredMagnitude[stream][channel][capRange] / bestAmplitude));
                if (distance < minDistance) {
                    bestAmplitudeIndex = capRange;
                    minDistance = distance;
                }
            }
            switch (bestAmplitudeIndex) {
            case 0: Cseries = 0.1e-12; break;
            case 1: Cseries = 1.0e-12; break;
            case 2: Cseries = 10.0e-12; break;
            }

            // Calculate current amplitude produced by on-chip voltage DAC
            current = TWO_PI * actualImpedanceFreq * dacVoltageAmplitude * Cseries;

            // Calculate impedance magnitude from calculated current and measured voltage.
            impedanceMagnitude =
                1.0e-6 * (measuredMagnitude[stream][channel][bestAmplitudeIndex] / current) *
                (18.0 * relativeFreq * relativeFreq + 1.0);

            // Calculate impedance phase, with small correction factor accounting for the
            // 3-command SPI pipeline delay.
            impedancePhase =
                measuredPhase[stream][channel][bestAmplitudeIndex] + (360.0 * (3.0 / period));

            // Factor out on-chip parasitic capacitance from impedance measurement.
            factorOutParallelCapacitance(impedanceMagnitude, impedancePhase, actualImpedanceFreq,
                                         parasiticCapacitance);

            // Perform empirical resistance correction to improve accuarcy at sample rates below
            // 15 kS/s.
            empiricalResistanceCorrection(impedanceMagnitude, impedancePhase,
                                          board->settings.boardSampleRate);
            impedances.magnitudes_by_stream[stream].push_back(impedanceMagnitude);
            impedances.phases_by_stream[stream].push_back(impedancePhase);
        }
    }

    return impedances;
}

void ImpedanceMeter::restoreBoardSettings()
{
    board->evalBoard->setContinuousRunMode(false);
    board->evalBoard->setMaxTimeStep(0);
    board->evalBoard->flush();
    // Switch back to flatline
    board->evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                           Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 0);
    board->evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot::AuxCmd1, 0, 1);

    board->evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::SPIPort::All,
                                           Rhd2000EvalBoard::AuxCmdSlot::AuxCmd3,
                                           board->settings.fastSettleEnabled ? 2 : 1);

    if (board->settings.fastTTLSettleEnabled) {
        board->evalBoard->enableExternalFastSettle(true);
    }
}
