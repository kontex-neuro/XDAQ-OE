#pragma once

#include <VisualizerEditorHeaders.h>

#include <memory>

#include "DeviceThread.h"
#include "UI/ChannelCanvas.h"

namespace RhythmNode
{
class DeviceEditor : public VisualizerEditor,
                     public ComboBox::Listener,
                     public Button::Listener,
                     public PopupChannelSelector::Listener

{
public:
    /** Constructor */
    DeviceEditor(GenericProcessor *parentNode, DeviceThread *thread);

    /** Destructor*/
    ~DeviceEditor() {}

    /** Respond to combo box changes (e.g. sample rate)*/
    void comboBoxChanged(ComboBox *comboBox) override;

    /** Respond to button clicks*/
    void buttonClicked(Button *button) override;

    /** Disable UI during acquisition*/
    void startAcquisition() override{};

    /** Enable UI after acquisition is finished*/
    void stopAcquisition() override{};

    /** Updates channel canvas*/
    void updateSettings() override;

    /** Saves custom parameters */
    void saveVisualizerEditorParameters(XmlElement *xml) override{};

    /** Loads custom parameters*/
    void loadVisualizerEditorParameters(XmlElement *xml) override{};

    /** Creates an interface with additional channel settings*/
    Visualizer *createNewCanvas(void) override;

    /** Called by PopupChannelSelector */
    void channelStateChanged(Array<int> newChannels) override{};

private:
    std::unique_ptr<UtilityButton> new_layout_button;
    std::unique_ptr<ComboBox> chunk_size_combo;
    std::unique_ptr<ComboBox> num_sample_combo;
    std::unique_ptr<ComboBox> num_channel_combo;

    std::unique_ptr<Label> chunk_size_label;
    std::unique_ptr<Label> num_sample_label;
    std::unique_ptr<Label> num_channel_label;

    DeviceThread *board;
    ChannelCanvas *canvas = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DeviceEditor);
};

}  // namespace RhythmNode
