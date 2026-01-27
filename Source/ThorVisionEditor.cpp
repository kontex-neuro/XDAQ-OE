/*
------------------------------------------------------------------

This file is part of the Open Ephys GUI
Copyright (C) 2022 Open Ephys

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

#include "ThorVisionEditor.h"

#include "ThorVision.h"

AppStatusInterface::AppStatusInterface()
{
    _appStatusLabel = std::make_unique<Label>("App Status", "Disconnected");
    _appStatusLabel->setBounds(-5, 25, 150, 20);
    _appStatusLabel->setEditable(false, false, false);
    _appStatusLabel->addListener(this);
    _appStatusLabel->setTooltip("ThorVision app connection status: [Disconnected, Connected]");
    addAndMakeVisible(_appStatusLabel.get());

    _recordingStatusLabel = std::make_unique<Label>("Recording Status", "Unknown");
    _recordingStatusLabel->setBounds(-5, 65, 150, 20);
    _recordingStatusLabel->setEditable(false, false, false);
    _recordingStatusLabel->addListener(this);
    _recordingStatusLabel->setTooltip(
        "Recording status of the ThorVision app: [Not Ready, Ready, Recording]");
    addAndMakeVisible(_recordingStatusLabel.get());
}

void AppStatusInterface::setAppStatus(const String &status)
{
    _appStatusLabel->setText(status, dontSendNotification);
}

void AppStatusInterface::setRecordingStatus(const String &status)
{
    _recordingStatusLabel->setText(status, sendNotification);
}

void AppStatusInterface::paint(Graphics &g)
{
    g.setFont(Font("Small Text", 12, Font::plain));
    g.setColour(findColour(ThemeColours::defaultText));
    g.drawText("App Connection:", 0, 10, 150, 20, Justification::left, false);
    g.drawText("Recording Status:", 0, 50, 150, 20, Justification::left, false);
}

ThorVisionEditor::ThorVisionEditor(GenericProcessor *parentNode) : GenericEditor(parentNode)
{
    desiredWidth = 150;

    _board = (ThorVision *) parentNode;
    _board->addChangeListener(this);

    _appStatusInterface = std::make_unique<AppStatusInterface>();
    addAndMakeVisible(_appStatusInterface.get());
    _appStatusInterface->setBounds(10, 20, 150, 150);
}

ThorVisionEditor::~ThorVisionEditor() { _board->removeChangeListener(this); }

void ThorVisionEditor::changeListenerCallback(ChangeBroadcaster *)
{
    _appStatusInterface->setAppStatus(_board->isConnected() ? "Connected" : "Disconnected");

    switch (_board->getRecordingState()) {
    case ThorVisionHttpClient::RecordingState::NotReady:
        _appStatusInterface->setRecordingStatus("Not Ready");
        break;
    case ThorVisionHttpClient::RecordingState::Ready:
        _appStatusInterface->setRecordingStatus("Ready");
        break;
    case ThorVisionHttpClient::RecordingState::Recording:
        _appStatusInterface->setRecordingStatus("Recording");
        break;
    case ThorVisionHttpClient::RecordingState::Unknown:
        _appStatusInterface->setRecordingStatus("Unknown");
        break;
    default: fmt::println("Unknown recording state"); break;
    }
}