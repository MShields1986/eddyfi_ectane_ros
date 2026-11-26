#include "ectane_handler.h"



EctaneHandler::EctaneHandler()
    :  // C SDK Data Sender
       sender_address_("127.0.0.1"),
       receive_port_(24011),
       receiver_(sender_address_, receive_port_),

       // C SDK State Command Listener
       recipient_address_("127.0.0.1"), 
       send_port_(24010),
       sender_(recipient_address_, send_port_)
{
}


void EctaneHandler::setup(
    std::string ectane_address,
    Configurations configuration,
    int32_t acquisition_rate,
    int32_t gain,
    const std::vector<int32_t>& frequencies,
    const std::vector<float>& voltages
    )

{
    // Ectane
    ectane_address_ = ectane_address;

    // Internal Representation of Command State
    state_ = State::do_nothing;
    configuration_ = configuration;
    acquisition_rate_ = acquisition_rate;
    gain_ = gain;
    frequencies_ = frequencies;
    voltages_ = voltages;

    updateCommandPacket();
}


std::vector<int> EctaneHandler::readData() {
    return receiver_.receive();
}


void EctaneHandler::interruptReadData() {
    receiver_.interrupt();
}


void EctaneHandler::updateCommandPacket() {
    command_packet_.state = state_;
    strncpy(command_packet_.ip_address, ectane_address_.c_str(), sizeof(command_packet_.ip_address) - 1);
    command_packet_.ip_address[sizeof(command_packet_.ip_address) - 1] = '\0';
    command_packet_.configuration = configuration_;
    command_packet_.acquisition_rate = acquisition_rate_;
    command_packet_.gain = gain_;

    // Slots beyond the configuration's frequencies are not read by the C SDK
    for (std::size_t i = 0; i < kCommandPacketFrequencies; ++i) {
        command_packet_.frequencies[i] = i < frequencies_.size() ? frequencies_[i] : 0;
        command_packet_.voltages[i] = i < voltages_.size() ? voltages_[i] : 0.0f;
    }

    updateCommandString();
}


void EctaneHandler::updateCommandString() {
    command_string_ = //std::to_string((int32_t)command_packet_.state) + "," +
                        std::to_string(command_packet_.state) + "," +
                        command_packet_.ip_address + "," +
                        std::to_string(command_packet_.configuration) + "," +
                        std::to_string(command_packet_.acquisition_rate) + "," +
                        std::to_string(command_packet_.gain);

    for (int i = 0; i < kCommandPacketFrequencies; ++i) {
        command_string_ += "," + std::to_string(command_packet_.frequencies[i]) +
                           "," + std::to_string(command_packet_.voltages[i]);
    }
}


void EctaneHandler::changeState(State state) {
    state_ = state;
    updateCommandPacket();
    //std::cout << "Outgoing command string: " << command_string_ << std::endl;
    sender_.send(command_string_);
}


void EctaneHandler::doNothing() {
    changeState(State::do_nothing);
}


void EctaneHandler::commandSocketInitialise() {
    changeState(State::socket_initialise);
}


void EctaneHandler::connecttoDevice() {
    changeState(State::connect_to_device);
}


void EctaneHandler::setUpDevice() {
    changeState(State::set_up_device);
}

void EctaneHandler::balanceDevice() {
    changeState(State::balance_device);
}


void EctaneHandler::acquireData() {
    changeState(State::acquire_data);
}


void EctaneHandler::stopData() {
    changeState(State::stop_data);
}


void EctaneHandler::disconnectDevice() {
    changeState(State::disconnect_device);
}
