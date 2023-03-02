#include "DeviceEditor.h"

#include <cmath>

#include "DeviceThread.h"
#include "UI/ChannelCanvas.h"

using namespace RhythmNode;

DeviceEditor::DeviceEditor(GenericProcessor *parentNode, DeviceThread *board_)
    : VisualizerEditor(parentNode, "tabText", 340), board(board_)
{
    chunk_size_label = std::make_unique<Label>("Chunk Size Label", "Chunk Size");
    chunk_size_label->setFont(Font("Small Text", 10, Font::plain));
    chunk_size_label->setBounds(0, 20, 60, 20);
    chunk_size_label->setColour(Label::textColourId, Colours::darkgrey);
    addAndMakeVisible(chunk_size_label.get());

    chunk_size_combo = std::make_unique<ComboBox>("Chunk Size");
    chunk_size_combo->setBounds(0, 35, 60, 18);
    chunk_size_combo->addListener(this);
    std::vector<int> vals{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
    for (int i = 0; i < vals.size(); ++i) {
        chunk_size_combo->addItem(std::to_string(vals[i]), i + 1);
    }
    chunk_size_combo->setSelectedId(1, sendNotification);
    addAndMakeVisible(chunk_size_combo.get());

    new_layout_button =
        std::make_unique<UtilityButton>("Use New Memory Layout", Font("Small Text", 13, Font::plain));
    new_layout_button->setBounds(0, 60, 180, 18);
    new_layout_button->addListener(this);
    new_layout_button->setClickingTogglesState(true);
    new_layout_button->setToggleState(false, dontSendNotification);
    addAndMakeVisible(new_layout_button.get());
}



void DeviceEditor::updateSettings()
{
    if (canvas != nullptr) {
        canvas->update();
    }
}

void DeviceEditor::comboBoxChanged(ComboBox *comboBox)
{
    if (comboBox == chunk_size_combo.get()) {
        board->chunk_size = std::stoi(comboBox->getText().toStdString());
    }
}


void DeviceEditor::buttonClicked(Button *button)
{
    if (button == new_layout_button.get()) {
        board->modified_layout = new_layout_button->getToggleState();
    }
}


Visualizer *DeviceEditor::createNewCanvas()
{
    GenericProcessor *processor = (GenericProcessor *) getProcessor();
    return new ChannelCanvas(board);
}