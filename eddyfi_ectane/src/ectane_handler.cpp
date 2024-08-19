#include "ectane_handler.h"



EctaneHandler::EctaneHandler(
    std::string ectane_address,
    Configurations configuration,
    int32_t acquisition_rate,
    int32_t gain,
    int32_t frequency,
    float voltage
    )
     : // Ectane
       ectane_address_(ectane_address),
       ec_data_(), ec_data_ptr_(&ec_data_),

       // C SDK Data Sender
       sender_address_("127.0.0.1"),
       receive_port_(24011),
       receiver_(sender_address_, receive_port_),

       // C SDK State Command Listener
       recipient_address_("127.0.0.1"), 
       send_port_(24010),
       sender_(recipient_address_, send_port_),

       // Internal Representation of Command State
       state_(State::do_nothing),
       configuration_(configuration),
       acquisition_rate_(acquisition_rate),
       gain_(gain),
       frequency_(frequency),
       voltage_(voltage)
{
    updateCommandPacket();
}


void EctaneHandler::readData() {
    ec_data_ = receiver_.receive();
}


void EctaneHandler::updateCommandPacket() {
    command_packet_.state = state_;
    strcpy(command_packet_.ip_address, ectane_address_.c_str());
    command_packet_.configuration = configuration_;
    command_packet_.acquisition_rate = acquisition_rate_;
    command_packet_.gain = gain_;
    command_packet_.frequency_1 = frequency_;
    command_packet_.voltage_1 = voltage_;
    command_packet_.frequency_2 = frequency_;
    command_packet_.voltage_2 = voltage_;
    command_packet_.frequency_3 = frequency_;
    command_packet_.voltage_3 = voltage_;
    command_packet_.frequency_4 = frequency_;
    command_packet_.voltage_4 = voltage_;
    command_packet_.frequency_5 = frequency_;
    command_packet_.voltage_5 = voltage_;
    command_packet_.frequency_6 = frequency_;
    command_packet_.voltage_6 = voltage_;
    command_packet_.frequency_7 = frequency_;
    command_packet_.voltage_7 = voltage_;
    command_packet_.frequency_8 = frequency_;
    command_packet_.voltage_8 = voltage_;

    updateCommandString();
}


void EctaneHandler::updateCommandString() {
    command_string_ = //std::to_string((int32_t)command_packet_.state) + "," +
                        std::to_string(command_packet_.state) + "," +
                        command_packet_.ip_address + "," +
                        std::to_string(command_packet_.configuration) + "," +
                        std::to_string(command_packet_.acquisition_rate) + "," +
                        std::to_string(command_packet_.gain) + "," +
                        std::to_string(command_packet_.frequency_1) + "," +
                        std::to_string((int32_t)command_packet_.voltage_1) + "," +
                        std::to_string(command_packet_.frequency_2) + "," +
                        std::to_string((int32_t)command_packet_.voltage_2) + "," +
                        std::to_string(command_packet_.frequency_3) + "," +
                        std::to_string((int32_t)command_packet_.voltage_3) + "," +
                        std::to_string(command_packet_.frequency_4) + "," +
                        std::to_string((int32_t)command_packet_.voltage_4) + "," +
                        std::to_string(command_packet_.frequency_5) + "," +
                        std::to_string((int32_t)command_packet_.voltage_5) + "," +
                        std::to_string(command_packet_.frequency_6) + "," +
                        std::to_string((int32_t)command_packet_.voltage_6) + "," +
                        std::to_string(command_packet_.frequency_7) + "," +
                        std::to_string((int32_t)command_packet_.voltage_7) + "," +
                        std::to_string(command_packet_.frequency_8) + "," +
                        std::to_string((int32_t)command_packet_.voltage_8);

    command_string_ += "\0";
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
