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

#include "ThorVisionTriggerEditor.h"

#include "ThorVisionTrigger.h"

ThorVisionTriggerEditor::ThorVisionTriggerEditor(GenericProcessor *parentNode)
    : GenericEditor(parentNode)
{
    desiredWidth = 150;

    board = (ThorVisionTrigger *) parentNode;

    // event frequency editor
    // addBoundedValueParameterEditor(Parameter::PROCESSOR_SCOPE,  // parameter scope
    //                                "interval",                  // parameter name
    //                                15,                          // x pos
    //                                35);                         // y pos

    // const auto grid_col1 = 10;

    // triggerButton = std::make_unique<UtilityButton>("Trigger");
    // triggerButton->setRadius(3.0f);
    // triggerButton->setBounds(55, 105, 80, 20);
    // triggerButton->setFont(FontOptions(12.0f));
    // triggerButton->addListener(this);
    // triggerButton->setClickingTogglesState(true);
    // triggerButton->setTooltip("Trigger record in ThorVision");
    // triggerButton->setToggleState(true, dontSendNotification);
    // addAndMakeVisible(triggerButton.get());
}

void ThorVisionTriggerEditor::buttonClicked(Button *button)
{
    // if (button == triggerButton.get()) {
    //     fmt::println("ThorVision trigger button clicked");
    //     ThorVisionTrigger *processor = (ThorVisionTrigger *) getProcessor();
    // }
}