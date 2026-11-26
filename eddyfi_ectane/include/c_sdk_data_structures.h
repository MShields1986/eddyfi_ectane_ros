#pragma once

// In line with Euan Foster's additions to InstDefsEctIris.h
// https://github.com/UoS-EEE-Automation/EctaneLabviewIntegration/blob/3bd3d2f49d6a7a6e077b98b8bf7a1289c4204995/C%20Programs/InstDefsEctIris.h#L1443

enum State {
    do_nothing,
    socket_initialise,
    connect_to_device,
    set_up_device,
    balance_device,
    acquire_data,
    stop_data,
    disconnect_device
};


// Frequency and voltage pairs carried by the command packet
constexpr int kCommandPacketFrequencies = 8;

// The Ectane supports up to 5 frequencies per time slot (inst_MAX_NUM_OF_FREQUENCY_PER_TS in InstDefsEctIris.h)
constexpr int kMaxFrequencies = 5;

struct CommandPacket {
    enum State state;
    char ip_address[22];
    int configuration;
    int32_t acquisition_rate;
    int32_t gain;
    // Sent as frequency_1,voltage_1,frequency_2,voltage_2,...
    int32_t frequencies[kCommandPacketFrequencies];
    float voltages[kCommandPacketFrequencies];
};


// In line with Euan Foster's Configurations.h
// https://github.com/UoS-EEE-Automation/EctaneLabviewIntegration/blob/3bd3d2f49d6a7a6e077b98b8bf7a1289c4204995/C%20Programs/Configurations.h
// case 0: //example
// case 1: //HF Probe Axial - 1 Channel
// case 2: //HF Probe Transversal - 1 Channel
// case 3: //HF Probe Axial & Transversal - 1 Channel
// case 4: //HF Probe Axial & Transversal - 4 Channels
// case 5: //LF Probe Axial - 1 Channel
// case 6: //LF Probe Transversal - 1 Channel
// case 7: //LF Probe Axial & Transversal - 1 Channel
// case 8: //LF Probe Axial & Transversal - 4 Channels

enum Configurations {
    example,
    hf_probe_axial_1_channel,
    hf_probe_transversal_1_channel,
    hf_probe_axial_and_transversal_1_channel,
    hf_probe_axial_and_transversal_4_channels,
    lf_probe_axial_1_channel,
    lf_probe_transversal_1_channel,
    lf_probe_axial_and_transversal_1_channel,
    lf_probe_axial_and_transversal_4_channels
};
