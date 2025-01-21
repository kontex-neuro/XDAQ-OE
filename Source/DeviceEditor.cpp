/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2014 Open Ephys

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

#include "DeviceEditor.h"

#include <cmath>

#include "DeviceThread.h"
#include "Headstage.h"
#include "UI/ChannelCanvas.h"

using namespace RhythmNode;

#ifdef WIN32
#if (_MSC_VER < 1800)  // round doesn't exist on MSVC prior to 2013 version
inline double round(double x) { return floor(x + 0.5); }
#endif
#endif

DeviceEditor::DeviceEditor(GenericProcessor *parentNode, DeviceThread *board_)
    : VisualizerEditor(parentNode, "tabText", 340), board(board_)
{
    canvas = nullptr;

    tabText = "XDAQ";

    measureWhenRecording = false;
    saveImpedances = false;

    // add headstage-specific controls (currently just a toggle button)
    const auto port_layout_width = 30 + board->get_ports().max_chips_per_port * 20;
    setDesiredWidth(260 + port_layout_width);
    for (int i = 0; i < board->get_ports().num_of_ports; i++) {
        HeadstageOptionsInterface *hsOptions = new HeadstageOptionsInterface(board, this, i);
        headstageOptionsInterfaces.add(hsOptions);
        addAndMakeVisible(hsOptions);
        hsOptions->setBounds(3, 28 + i * 20, port_layout_width, 18);
    }

    // add rescan button
    rescanButton = new UtilityButton("RESCAN", Font("Small Text", 13, Font::plain));
    rescanButton->setRadius(3.0f);
    rescanButton->setBounds(6, 108, 65, 18);
    rescanButton->addListener(this);
    rescanButton->setTooltip("Check for connected headstages");
    addAndMakeVisible(rescanButton);

    const auto grid_col1 = 10 + port_layout_width;
    // add sample rate selection
    sampleRateInterface = new SampleRateInterface(board, this);
    addAndMakeVisible(sampleRateInterface);
    sampleRateInterface->setBounds(grid_col1, 20, 80, 50);

    // add Bandwidth selection
    bandwidthInterface = new BandwidthInterface(board, this);
    addAndMakeVisible(bandwidthInterface);
    bandwidthInterface->setBounds(grid_col1, 55, 80, 50);

    // add AUX channel enable/disable button
    auxButton = new UtilityButton("AUX", Font("Small Text", 13, Font::plain));
    auxButton->setRadius(3.0f);
    auxButton->setBounds(grid_col1, 108, 32, 18);
    auxButton->addListener(this);
    auxButton->setClickingTogglesState(true);
    auxButton->setTooltip("Toggle AUX channels (3 per headstage)");
    addAndMakeVisible(auxButton);

    // add ADC channel enable/disable button
    adcButton = new UtilityButton("ADC", Font("Small Text", 13, Font::plain));
    adcButton->setRadius(3.0f);
    adcButton->setBounds(grid_col1 + 32 + 1, 108, 32, 18);
    adcButton->addListener(this);
    adcButton->setClickingTogglesState(true);
    adcButton->setTooltip("Toggle 8 external HDMI ADC channels");
    addAndMakeVisible(adcButton);

    const auto grid_col2 = 104 + port_layout_width;
    // add audio output config interface
    audioLabel = new Label("audio label", "Audio out");
    audioLabel->setBounds(grid_col2, 20, 75, 15);
    audioLabel->setFont(Font("Small Text", 10, Font::plain));
    audioLabel->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(audioLabel);

    for (int i = 0; i < 2; i++) {
        ElectrodeButton *button = new ElectrodeButton(-1);
        electrodeButtons.add(button);

        button->setBounds(grid_col2 + i * 30, 35, 30, 15);
        button->setChannelNum(-1);
        button->setClickingTogglesState(false);
        button->setToggleState(false, dontSendNotification);

        addAndMakeVisible(button);
        button->addListener(this);

        if (i == 0) {
            button->setTooltip("Audio monitor left channel");
        } else {
            button->setTooltip("Audio monitor right channel");
        }
    }

    // add HW audio parameter selection
    audioInterface = new AudioInterface(board, this);
    addAndMakeVisible(audioInterface);
    audioInterface->setBounds(grid_col2, 55, 70, 50);

    clockInterface = new ClockDivideInterface(board, this);
    // addAndMakeVisible(clockInterface);
    // clockInterface->setBounds(grid_col2, 80, 70, 50);

    // add DSP Offset Button
    dspoffsetButton = new UtilityButton("DSP:", Font("Small Text", 13, Font::plain));
    dspoffsetButton->setRadius(3.0f);  // sets the radius of the button's corners
    // sets the x position, y position, width, and height of the button
    dspoffsetButton->setBounds(grid_col2, 108, 32, 18);
    dspoffsetButton->addListener(this);
    dspoffsetButton->setClickingTogglesState(
        true);  // makes the button toggle its state when clicked
    dspoffsetButton->setTooltip("Toggle DSP offset removal");
    addAndMakeVisible(
        dspoffsetButton);  // makes the button a child component of the editor and makes it visible
    dspoffsetButton->setToggleState(true, dontSendNotification);

    // add DSP Frequency Selection field
    dspInterface = new DSPInterface(board, this);
    addAndMakeVisible(dspInterface);
    dspInterface->setBounds(grid_col2 + 32, 108, 40, 50);

    const auto grid_col3 = 180 + port_layout_width;
    dacTTLButton = new UtilityButton("DAC TTL", Font("Small Text", 13, Font::plain));
    dacTTLButton->setRadius(3.0f);
    dacTTLButton->setBounds(grid_col3, 25, 60, 18);
    dacTTLButton->addListener(this);
    dacTTLButton->setClickingTogglesState(true);
    dacTTLButton->setTooltip("Toggle DAC Threshold TTL Output");
    addAndMakeVisible(dacTTLButton);

    dacHPFlabel = new Label("DAC HPF", "DAC HPF");
    dacHPFlabel->setFont(Font("Small Text", 10, Font::plain));
    dacHPFlabel->setBounds(grid_col3, 40, 60, 20);
    dacHPFlabel->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(dacHPFlabel);

    dacHPFcombo = new ComboBox("dacHPFCombo");
    dacHPFcombo->setBounds(grid_col3, 55, 60, 18);
    dacHPFcombo->addListener(this);
    dacHPFcombo->addItem("OFF", 1);
    int HPFvalues[10] = {50, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    for (int k = 0; k < 10; k++) {
        dacHPFcombo->addItem(String(HPFvalues[k]) + " Hz", 2 + k);
    }
    dacHPFcombo->setSelectedId(1, sendNotification);
    addAndMakeVisible(dacHPFcombo);

    ttlSettleLabel = new Label("TTL Settle", "TTL Settle");
    ttlSettleLabel->setFont(Font("Small Text", 10, Font::plain));
    ttlSettleLabel->setBounds(grid_col3, 70, 70, 20);
    ttlSettleLabel->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(ttlSettleLabel);

    ttlSettleCombo = new ComboBox("FastSettleComboBox");
    ttlSettleCombo->setBounds(grid_col3, 85, 60, 18);
    ttlSettleCombo->addListener(this);
    ttlSettleCombo->addItem("-", 1);
    for (int k = 0; k < 8; k++) {
        ttlSettleCombo->addItem("TTL" + String(1 + k), 2 + k);
    }
    ttlSettleCombo->setSelectedId(1, sendNotification);
    addAndMakeVisible(ttlSettleCombo);
}


void DeviceEditor::measureImpedance()
{
    board->runImpedanceTest();

    CoreServices::updateSignalChain(this);
}

void DeviceEditor::saveImpedance(File &file)
{
    LOGD("Saving impedances to ", file.getFullPathName());

    board->saveImpedances(file);
}

void DeviceEditor::updateSettings()
{
    if (canvas != nullptr) {
        canvas->update();
    }
}

void DeviceEditor::comboBoxChanged(ComboBox *comboBox)
{
    if (comboBox == ttlSettleCombo) {
        int selectedChannel = ttlSettleCombo->getSelectedId();
        if (selectedChannel == 1) {
            board->setFastTTLSettle(false, 0);
        } else {
            board->setFastTTLSettle(true, selectedChannel - 2);
        }
    } else if (comboBox == dacHPFcombo) {
        int selection = dacHPFcombo->getSelectedId();
        if (selection == 1) {
            board->setDAChpf(100, false);
        } else {
            int HPFvalues[10] = {50, 100, 200, 300, 400, 500, 600, 700, 800, 900};
            board->setDAChpf(HPFvalues[selection - 2], true);
        }
    }
}

void DeviceEditor::channelStateChanged(Array<int> newChannels)
{
    int selectedChannel = -1;

    if (newChannels.size() > 0) {
        selectedChannel = newChannels[0];
    }


    board->setDACchannel(int(activeAudioChannel), selectedChannel);

    if (selectedChannel > -1) {
        electrodeButtons[int(activeAudioChannel)]->setToggleState(true, dontSendNotification);
        electrodeButtons[int(activeAudioChannel)]->setChannelNum(selectedChannel + 1);
    } else {
        electrodeButtons[int(activeAudioChannel)]->setChannelNum(selectedChannel);
        electrodeButtons[int(activeAudioChannel)]->setToggleState(false, dontSendNotification);
    }
}


void DeviceEditor::buttonClicked(Button *button)
{
    if (button == rescanButton && !acquisitionIsActive) {
        board->scanPorts();

        for (int i = 0; i < 4; i++) {
            headstageOptionsInterfaces[i]->checkEnabledState();
        }
        CoreServices::updateSignalChain(this);
    } else if (button == electrodeButtons[0] || button == electrodeButtons[1]) {
        std::vector<bool> channelStates;

        if (button == electrodeButtons[0])
            activeAudioChannel = LEFT;
        else
            activeAudioChannel = RIGHT;

        for (int i = 0; i < board->getNumDataOutputs(ContinuousChannel::ELECTRODE); i++) {
            if (electrodeButtons[int(activeAudioChannel)]->getChannelNum() - 1 == i)
                channelStates.push_back(true);
            else
                channelStates.push_back(false);
        }

        auto *channelSelector = new PopupChannelSelector(this, channelStates);

        channelSelector->setChannelButtonColour(Colour(0, 174, 239));
        channelSelector->setMaximumSelectableChannels(1);

        CallOutBox &myBox = CallOutBox::launchAsynchronously(
            std::unique_ptr<Component>(channelSelector), button->getScreenBounds(), nullptr);
    } else if (button == auxButton && !acquisitionIsActive) {
        board->enableAuxs(button->getToggleState());
        LOGD("AUX Button toggled");
        CoreServices::updateSignalChain(this);
    } else if (button == adcButton && !acquisitionIsActive) {
        board->enableAdcs(button->getToggleState());
        LOGD("ADC Button toggled");
        CoreServices::updateSignalChain(this);

    } else if (button == dacTTLButton) {
        board->setTTLoutputMode(dacTTLButton->getToggleState());
    } else if (button == dspoffsetButton && !acquisitionIsActive) {
        LOGD("DSP offset ", button->getToggleState());
        board->setDSPOffset(button->getToggleState());
    }
}

void DeviceEditor::startAcquisition()
{
    sampleRateInterface->setEnabled(false);
    rescanButton->setEnabledState(false);
    auxButton->setEnabledState(false);
    adcButton->setEnabledState(false);
    dspoffsetButton->setEnabledState(false);
    if (dio32_button) {
        dio32_button->setEnabledState(false);
    }

    acquisitionIsActive = true;
}

void DeviceEditor::stopAcquisition()
{
    sampleRateInterface->setEnabled(true);
    rescanButton->setEnabledState(true);
    auxButton->setEnabledState(true);
    adcButton->setEnabledState(true);
    dspoffsetButton->setEnabledState(true);
    if (dio32_button) {
        dio32_button->setEnabledState(true);
    }

    acquisitionIsActive = false;
}

void DeviceEditor::saveVisualizerEditorParameters(XmlElement *xml)
{
    xml->setAttribute("SampleRate", sampleRateInterface->getSelectedId());
    xml->setAttribute("SampleRateString", sampleRateInterface->getText());
    xml->setAttribute("LowCut", bandwidthInterface->getLowerBandwidth());
    xml->setAttribute("HighCut", bandwidthInterface->getUpperBandwidth());
    xml->setAttribute("AUXsOn", auxButton->getToggleState());
    xml->setAttribute("ADCsOn", adcButton->getToggleState());
    xml->setAttribute("AudioOutputL", electrodeButtons[0]->getChannelNum());
    xml->setAttribute("AudioOutputR", electrodeButtons[1]->getChannelNum());
    xml->setAttribute("NoiseSlicer", audioInterface->getNoiseSlicerLevel());
    xml->setAttribute("TTLFastSettle", ttlSettleCombo->getSelectedId());
    xml->setAttribute("DAC_TTL", dacTTLButton->getToggleState());
    xml->setAttribute("DAC_HPF", dacHPFcombo->getSelectedId());
    xml->setAttribute("DSPOffset", dspoffsetButton->getToggleState());
    xml->setAttribute("DSPCutoffFreq", dspInterface->getDspCutoffFreq());
    xml->setAttribute("save_impedance_measurements", saveImpedances);
    xml->setAttribute("auto_measure_impedances", measureWhenRecording);
    xml->setAttribute("ClockDivideRatio", clockInterface->getClockDivideRatio());
    if (dio32_button) xml->setAttribute("DIO32", dio32_button->getToggleState());

    // loop through all headstage options interfaces and save their parameters
    for (int i = 0; i < 4; i++) {
        XmlElement *hsOptions = xml->createNewChildElement("HSOPTIONS");
        hsOptions->setAttribute("index", i);
    }

    for (int i = 0; i < 8; i++) {
        XmlElement *adc = xml->createNewChildElement("ADCRANGE");
        adc->setAttribute("Channel", i);
        adc->setAttribute("Range", board->getAdcRange(i));
    }

    // save channel naming scheme
    xml->setAttribute("Channel_Naming_Scheme", board->getNamingScheme());
}

void DeviceEditor::loadVisualizerEditorParameters(XmlElement *xml)
{
    sampleRateInterface->setSelectedId(xml->getIntAttribute("SampleRate"));
    bandwidthInterface->setLowerBandwidth(xml->getDoubleAttribute("LowCut"));
    bandwidthInterface->setUpperBandwidth(xml->getDoubleAttribute("HighCut"));
    auxButton->setToggleState(xml->getBoolAttribute("AUXsOn"), sendNotification);
    adcButton->setToggleState(xml->getBoolAttribute("ADCsOn"), sendNotification);

    audioInterface->setNoiseSlicerLevel(xml->getIntAttribute("NoiseSlicer"));
    ttlSettleCombo->setSelectedId(xml->getIntAttribute("TTLFastSettle"));
    dacTTLButton->setToggleState(xml->getBoolAttribute("DAC_TTL"), sendNotification);
    dacHPFcombo->setSelectedId(xml->getIntAttribute("DAC_HPF"));
    dspoffsetButton->setToggleState(xml->getBoolAttribute("DSPOffset"), sendNotification);
    dspInterface->setDspCutoffFreq(xml->getDoubleAttribute("DSPCutoffFreq"));
    saveImpedances = xml->getBoolAttribute("save_impedance_measurements");
    measureWhenRecording = xml->getBoolAttribute("auto_measure_impedances");
    clockInterface->setClockDivideRatio(xml->getIntAttribute("ClockDivideRatio"));
    if (dio32_button)
        dio32_button->setToggleState(xml->getBoolAttribute("DIO32"), sendNotification);

    int AudioOutputL = xml->getIntAttribute("AudioOutputL", -1);
    int AudioOutputR = xml->getIntAttribute("AudioOutputR", -1);

    electrodeButtons[0]->setChannelNum(AudioOutputL);
    board->setDACchannel(0, AudioOutputL);
    if (AudioOutputL > -1) electrodeButtons[0]->setToggleState(true, dontSendNotification);

    electrodeButtons[1]->setChannelNum(AudioOutputR);
    board->setDACchannel(1, AudioOutputR);
    if (AudioOutputR > -1) electrodeButtons[1]->setToggleState(true, dontSendNotification);

    forEachXmlChildElementWithTagName(*xml, adc, "ADCRANGE")
    {
        int channel = adc->getIntAttribute("Channel", -1);
        int range = adc->getIntAttribute("Range", -1);
        if (channel >= 0 && range >= 0) board->setAdcRange(channel, range);
    }

    forEachXmlChildElementWithTagName(*xml, hsOptions, "HSOPTIONS")
    {
        int index = hsOptions->getIntAttribute("index", -1);

        if (index > -1) {
            // headstageOptionsInterfaces[index]->set32Channel(0,
            // hsOptions->getBoolAttribute("hs1_full_channels", true));
            // headstageOptionsInterfaces[index]->set32Channel(1,
            // hsOptions->getBoolAttribute("hs2_full_channels", true));
        }
    }


    // load channel naming scheme
    board->setNamingScheme((ChannelNamingScheme) xml->getIntAttribute("Channel_Naming_Scheme", 0));
}


Visualizer *DeviceEditor::createNewCanvas()
{
    GenericProcessor *processor = (GenericProcessor *) getProcessor();

    canvas = new ChannelCanvas(board, this);

    return canvas;
}

// Bandwidth Options --------------------------------------------------------------------

BandwidthInterface::BandwidthInterface(DeviceThread *board_, DeviceEditor *editor_)
    : board(board_), editor(editor_)
{
    name = "Bandwidth";

    lastHighCutString = "7500";
    lastLowCutString = "1";

    actualUpperBandwidth = 7500.0f;
    actualLowerBandwidth = 1.0f;

    upperBandwidthSelection = new Label(
        "UpperBandwidth", lastHighCutString);  // this is currently set in DeviceThread, the cleaner
                                               // way would be to set it here again
    upperBandwidthSelection->setEditable(true, false, false);
    upperBandwidthSelection->addListener(this);
    upperBandwidthSelection->setBounds(30, 25, 60, 20);
    upperBandwidthSelection->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(upperBandwidthSelection);

    lowerBandwidthSelection = new Label("LowerBandwidth", lastLowCutString);
    lowerBandwidthSelection->setEditable(true, false, false);
    lowerBandwidthSelection->addListener(this);
    lowerBandwidthSelection->setBounds(30, 10, 60, 20);
    lowerBandwidthSelection->setColour(Label::textColourId, Colours::darkgrey);

    addAndMakeVisible(lowerBandwidthSelection);
}


void BandwidthInterface::labelTextChanged(Label *label)
{
    if (!(editor->acquisitionIsActive) && board->foundInputSource()) {
        if (label == upperBandwidthSelection) {
            Value val = label->getTextValue();
            double requestedValue = double(val.getValue());

            if (requestedValue < 100.0 || requestedValue > 20000.0 ||
                requestedValue < lastLowCutString.getFloatValue()) {
                CoreServices::sendStatusMessage("Value out of range.");

                label->setText(lastHighCutString, dontSendNotification);

                return;
            }

            actualUpperBandwidth = board->setUpperBandwidth(requestedValue);

            LOGD("Setting Upper Bandwidth to ", requestedValue);
            LOGD("Actual Upper Bandwidth:  ", actualUpperBandwidth);
            label->setText(String(round(actualUpperBandwidth * 10.f) / 10.f), dontSendNotification);

        } else {
            Value val = label->getTextValue();
            double requestedValue = double(val.getValue());

            if (requestedValue < 0.1 || requestedValue > 500.0 ||
                requestedValue > lastHighCutString.getFloatValue()) {
                CoreServices::sendStatusMessage("Value out of range.");

                label->setText(lastLowCutString, dontSendNotification);

                return;
            }

            actualLowerBandwidth = board->setLowerBandwidth(requestedValue);

            LOGD("Setting Lower Bandwidth to ", requestedValue);
            LOGD("Actual Lower Bandwidth:  ", actualLowerBandwidth);

            label->setText(String(round(actualLowerBandwidth * 10.f) / 10.f), dontSendNotification);
        }
    } else if (editor->acquisitionIsActive) {
        CoreServices::sendStatusMessage("Can't change bandwidth while acquisition is active!");
        if (label == upperBandwidthSelection)
            label->setText(lastHighCutString, dontSendNotification);
        else
            label->setText(lastLowCutString, dontSendNotification);
        return;
    }
}

void BandwidthInterface::setLowerBandwidth(double value)
{
    actualLowerBandwidth = board->setLowerBandwidth(value);
    lowerBandwidthSelection->setText(String(round(actualLowerBandwidth * 10.f) / 10.f),
                                     dontSendNotification);
}

void BandwidthInterface::setUpperBandwidth(double value)
{
    actualUpperBandwidth = board->setUpperBandwidth(value);
    upperBandwidthSelection->setText(String(round(actualUpperBandwidth * 10.f) / 10.f),
                                     dontSendNotification);
}

double BandwidthInterface::getLowerBandwidth() { return actualLowerBandwidth; }

double BandwidthInterface::getUpperBandwidth() { return actualUpperBandwidth; }


void BandwidthInterface::paint(Graphics &g)
{
    g.setColour(Colours::darkgrey);

    g.setFont(Font("Small Text", 10, Font::plain));

    g.drawText(name, 0, 0, 200, 15, Justification::left, false);

    g.drawText("Low:", 0, 11, 200, 20, Justification::left, false);

    g.drawText("High:", 0, 26, 200, 20, Justification::left, false);
}

// Sample rate Options --------------------------------------------------------------------

SampleRateInterface::SampleRateInterface(DeviceThread *board_, DeviceEditor *editor_)
    : board(board_), editor(editor_)
{
    name = "Sample Rate";

    sampleRateOptions.add("1.00 kS/s");
    sampleRateOptions.add("1.25 kS/s");
    sampleRateOptions.add("1.50 kS/s");
    sampleRateOptions.add("2.00 kS/s");
    sampleRateOptions.add("2.50 kS/s");
    sampleRateOptions.add("3.00 kS/s");
    sampleRateOptions.add("3.33 kS/s");
    sampleRateOptions.add("4.00 kS/s");
    sampleRateOptions.add("5.00 kS/s");
    sampleRateOptions.add("6.25 kS/s");
    sampleRateOptions.add("8.00 kS/s");
    sampleRateOptions.add("10.0 kS/s");
    sampleRateOptions.add("12.5 kS/s");
    sampleRateOptions.add("15.0 kS/s");
    sampleRateOptions.add("20.0 kS/s");
    sampleRateOptions.add("25.0 kS/s");
    sampleRateOptions.add("30.0 kS/s");


    rateSelection = new ComboBox("Sample Rate");
    rateSelection->addItemList(sampleRateOptions, 1);
    rateSelection->setSelectedId(17, dontSendNotification);
    rateSelection->addListener(this);
    rateSelection->setBounds(0, 12, 80, 20);
    addAndMakeVisible(rateSelection);
}

void SampleRateInterface::comboBoxChanged(ComboBox *cb)
{
    if (!(editor->acquisitionIsActive) && board->foundInputSource()) {
        if (cb == rateSelection) {
            board->setSampleRate(
                static_cast<Rhd2000EvalBoard::SampleRate>(cb->getSelectedId() - 1));

            LOGD("Setting sample rate to index ", cb->getSelectedId() - 1);

            CoreServices::updateSignalChain(editor);
        }
    }
}

int SampleRateInterface::getSelectedId() { return rateSelection->getSelectedId(); }

void SampleRateInterface::setSelectedId(int id) { rateSelection->setSelectedId(id); }

String SampleRateInterface::getText() { return rateSelection->getText(); }


void SampleRateInterface::paint(Graphics &g)
{
    g.setColour(Colours::darkgrey);

    g.setFont(Font("Small Text", 10, Font::plain));

    g.drawText(name, 0, 0, 80, 15, Justification::left, false);
}


// Headstage Options --------------------------------------------------------------------

HeadstageOptionsInterface::HeadstageOptionsInterface(DeviceThread *board_, DeviceEditor *editor_,
                                                     int port_idx)
    : board(board_), editor(editor_), name(std::to_string(port_idx + 1))
{
    const auto max_chips_per_port = board_->get_ports().max_chips_per_port;
    for (int chip = 0; chip < max_chips_per_port; ++chip) {
        headstage_ids.push_back(port_idx * max_chips_per_port + chip);
        hsButtons.push_back(
            std::make_unique<UtilityButton>(" ", Font("Small Text", 13, Font::plain)));
        hsButtons.back()->setRadius(3.0f);
        hsButtons.back()->setBounds(23 + 20 * chip, 1, 20, 17);
        hsButtons.back()->setEnabledState(false);
        hsButtons.back()->setCorners(true, false, true, false);
        hsButtons.back()->addListener(this);
        addAndMakeVisible(hsButtons.back().get());
    }

    checkEnabledState();
}

void HeadstageOptionsInterface::checkEnabledState()
{
    for (int i = 0; i < hsButtons.size(); ++i) {
        bool enabled = board->isHeadstageEnabled(headstage_ids[i]);
        hsButtons[i]->setEnabledState(enabled);
        if (enabled)
            hsButtons[i]->setLabel(
                String(board->get_headstages()[headstage_ids[i]].getNumActiveChannels()));
        else
            hsButtons[i]->setLabel(" ");
    }
    repaint();
}

void HeadstageOptionsInterface::buttonClicked(Button *button)
{
    if (!(editor->acquisitionIsActive) && board->foundInputSource()) {
        CoreServices::updateSignalChain(editor);
    }
}

bool HeadstageOptionsInterface::is32Channel(int hsIndex)
{
    return board->get_headstages()[headstage_ids[hsIndex]].getNumActiveChannels() == 32;
}

void HeadstageOptionsInterface::set32Channel(int hsIndex, bool is32Channel) {}


void HeadstageOptionsInterface::paint(Graphics &g)
{
    g.setColour(Colours::lightgrey);

    g.fillRoundedRectangle(5, 0, getWidth() - 10, getHeight(), 4.0f);

    if (isEnabled())
        g.setColour(Colours::black);
    else
        g.setColour(Colours::grey);

    g.setFont(Font("Small Text", 15, Font::plain));

    g.drawText(name, 8, 2, 200, 15, Justification::left, false);
}


// (Direct OpalKelly) Audio Options
// --------------------------------------------------------------------

AudioInterface::AudioInterface(DeviceThread *board_, DeviceEditor *editor_)
    : board(board_), editor(editor_)
{
    name = "Noise";

    lastNoiseSlicerString = "0";

    actualNoiseSlicerLevel = 0.0f;

    noiseSlicerLevelSelection = new Label(
        "Noise Slicer", lastNoiseSlicerString);  // this is currently set in DeviceThread, the
                                                 // cleaner would be to set it here again
    noiseSlicerLevelSelection->setEditable(true, false, false);
    noiseSlicerLevelSelection->addListener(this);
    noiseSlicerLevelSelection->setBounds(45, 6, 35, 20);
    noiseSlicerLevelSelection->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(noiseSlicerLevelSelection);
}

AudioInterface::~AudioInterface() {}


void AudioInterface::labelTextChanged(Label *label)
{
    if (board->foundInputSource()) {
        if (label == noiseSlicerLevelSelection) {
            Value val = label->getTextValue();
            int requestedValue = int(val.getValue());  // Note that it might be nice to translate to
                                                       // actual uV levels (16*value)

            if (requestedValue < 0 || requestedValue > 127) {
                CoreServices::sendStatusMessage("Value out of range.");

                label->setText(lastNoiseSlicerString, dontSendNotification);

                return;
            }

            actualNoiseSlicerLevel = board->setNoiseSlicerLevel(requestedValue);

            LOGD("Setting Noise Slicer Level to ", requestedValue);
            label->setText(String((roundFloatToInt) (actualNoiseSlicerLevel)),
                           dontSendNotification);
        }
    } else {
        Value val = label->getTextValue();
        int requestedValue = int(val.getValue());  // Note that it might be nice to translate to
                                                   // actual uV levels (16*value)
        if (requestedValue < 0 || requestedValue > 127) {
            CoreServices::sendStatusMessage("Value out of range.");
            label->setText(lastNoiseSlicerString, dontSendNotification);
            return;
        }
    }
}

void AudioInterface::setNoiseSlicerLevel(int value)
{
    actualNoiseSlicerLevel = board->setNoiseSlicerLevel(value);
    noiseSlicerLevelSelection->setText(String(roundFloatToInt(actualNoiseSlicerLevel)),
                                       dontSendNotification);
}

int AudioInterface::getNoiseSlicerLevel() { return actualNoiseSlicerLevel; }


void AudioInterface::paint(Graphics &g)
{
    g.setColour(Colours::darkgrey);
    g.setFont(Font("Small Text", 10, Font::plain));
    g.drawText(name, 0, 0, 200, 15, Justification::left, false);
    g.drawText("Slicer:", 0, 10, 200, 15, Justification::left, false);
}


// Clock Divider options
ClockDivideInterface::ClockDivideInterface(DeviceThread *board_, DeviceEditor *editor_)
    : name("Clock"),
      lastDivideRatioString("1"),
      board(board_),
      editor(editor_),
      actualDivideRatio(1)

{
    divideRatioSelection = new Label("Clock Divider", lastDivideRatioString);
    divideRatioSelection->setEditable(true, false, false);
    divideRatioSelection->addListener(this);
    divideRatioSelection->setBounds(45, 6, 35, 20);
    divideRatioSelection->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(divideRatioSelection);
}

void ClockDivideInterface::labelTextChanged(Label *label)
{
    if (board->foundInputSource()) {
        if (label == divideRatioSelection) {
            Value val = label->getTextValue();
            int requestedValue = int(val.getValue());

            if (requestedValue < 1 || requestedValue > 65534) {
                CoreServices::sendStatusMessage("Value must be between 1 and 65534.");
                label->setText(lastDivideRatioString, dontSendNotification);
                return;
            }

            // actualDivideRatio = board->setClockDivider(requestedValue);
            lastDivideRatioString = String(actualDivideRatio);

            LOGD("Setting clock divide ratio to ", actualDivideRatio);
            label->setText(lastDivideRatioString, dontSendNotification);
        }
    }
}

void ClockDivideInterface::setClockDivideRatio(int value)
{
    // actualDivideRatio = board->setClockDivider(value);
    divideRatioSelection->setText(String(actualDivideRatio), dontSendNotification);
}

void ClockDivideInterface::paint(Graphics &g)
{
    g.setColour(Colours::darkgrey);
    g.setFont(Font("Small Text", 10, Font::plain));
    g.drawText(name, 0, 0, 200, 15, Justification::left, false);
    g.drawText("Divider: ", 0, 10, 200, 15, Justification::left, false);
}

// DSP Options --------------------------------------------------------------------

DSPInterface::DSPInterface(DeviceThread *board_, DeviceEditor *editor_)
    : board(board_), editor(editor_)
{
    name = "DSP";

    dspOffsetSelection =
        new Label("DspOffsetSelection", String(round(board->getDspCutoffFreq() * 10.f) / 10.f));
    dspOffsetSelection->setEditable(true, false, false);
    dspOffsetSelection->addListener(this);
    dspOffsetSelection->setBounds(0, 0, 35, 20);
    dspOffsetSelection->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(dspOffsetSelection);
}

DSPInterface::~DSPInterface() {}

void DSPInterface::labelTextChanged(Label *label)
{
    if (!(editor->acquisitionIsActive) && board->foundInputSource()) {
        if (label == dspOffsetSelection) {
            Value val = label->getTextValue();
            double requestedValue = double(val.getValue());

            actualDspCutoffFreq = board->setDspCutoffFreq(requestedValue);

            LOGD("Setting DSP Cutoff Freq to ", requestedValue);
            LOGD("Actual DSP Cutoff Freq:  ", actualDspCutoffFreq);
            label->setText(String(round(actualDspCutoffFreq * 10.f) / 10.f), dontSendNotification);
        }
    } else if (editor->acquisitionIsActive) {
        CoreServices::sendStatusMessage("Can't change DSP cutoff while acquisition is active!");
    }
}

void DSPInterface::setDspCutoffFreq(double value)
{
    actualDspCutoffFreq = board->setDspCutoffFreq(value);
    dspOffsetSelection->setText(String(round(actualDspCutoffFreq * 10.f) / 10.f),
                                dontSendNotification);
}

double DSPInterface::getDspCutoffFreq() { return actualDspCutoffFreq; }

void DSPInterface::paint(Graphics &g)
{
    g.setColour(Colours::darkgrey);
    g.setFont(Font("Small Text", 10, Font::plain));
}
