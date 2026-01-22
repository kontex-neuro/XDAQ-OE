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

#pragma once

#ifndef THORVISIONEDITOR_H_DEFINED
#define THORVISIONEDITOR_H_DEFINED

#include <EditorHeaders.h>

class ThorVision;

class ThorVisionEditor : public GenericEditor, public Button::Listener
{
public:
    /** Constructor */
    ThorVisionEditor(GenericProcessor *parentNode);

    /** Destructor */
    ~ThorVisionEditor() {}

    void buttonClicked(Button *button) override;

private:
    // ScopedPointer<UtilityButton> thorvisionButton;
    // std::unique_ptr<UtilityButton> triggerButton;

    ThorVision *board;

    /** Generates an assertion if this class leaks */
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ThorVisionEditor);
};

#endif  // THORVISIONEDITOR_H_DEFINED