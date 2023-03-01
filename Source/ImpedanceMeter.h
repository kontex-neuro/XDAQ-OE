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
#include <array>
#include <atomic>
#include <complex>
#include <span>
#include <optional>

#include "DeviceThread.h"
#include "rhythm-api/rhd2000registers.h"

namespace RhythmNode
{

class ImpedanceMeter : public ThreadWithProgressWindow
{
public:
    /** Constructor*/
    ImpedanceMeter(DeviceThread *b);

    /** Destructor*/
    ~ImpedanceMeter();

    /** Runs the impedance measurement*/
    void run() override;

    /** Interrupt impedance measurement thread*/
    void stopThreadSafely();

    /** Wait for thread to finish*/
    void waitSafely();

    /** Save values to a file (XML format)*/
    void saveValues(File &file);

private:
    /** Calculates impedance values for all channels*/
    std::optional<Impedances> runImpedanceMeasurement();

    /** Restores settings of device*/
    void restoreBoardSettings();

    /** Returns the magnitude and phase (in degrees) of a selected frequency component (in Hz)
        for a selected amplifier channel on the selected USB data stream.*/
    std::complex<double> measureComplexAmplitude(std::span<float> ampdata, int numBlocks,
                                                 double sampleRate, double frequency,
                                                 int numPeriods);



    /** Updates electrode impedance measurement frequency, after checking that
        requested test frequency lies within acceptable ranges based on the
        amplifier bandwidth and the sampling rate.  See impedancefreqdialog.cpp
        for more information.*/
    float updateImpedanceFrequency(float desiredImpedanceFreq, bool &impedanceFreqValid);


    DeviceThread *board;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ImpedanceMeter);
};

}  // namespace RhythmNode
