#include "ectane_handler.h"


EctaneHandler::EctaneHandler()
     : // Ectane
       ectane_address_("192.168.002.035"),
       //ectane_address_("127.000.000.001"),
       ec_data_(""), ec_data_ptr_(&ec_data_),

       // C SDK State Command Listener
       recipient_address_("127.0.0.1"), 
       send_port_(24010),
       sender_(recipient_address_, send_port_),

       // C SDK Data Sender
       sender_address_("127.0.0.1"),
       receive_port_(24011),
       receiver_(sender_address_, receive_port_),

       // Internal Representation of Command State
       state_(State::do_nothing),
       configuration_(Configurations::lf_probe_axial_1_channel),
       acquisition_rate_(40),
       gain_(1),
       frequency_(1000),
       voltage_(1.0)
{
    updateCommandPacket();
}


void EctaneHandler::readData() {
    std::string test("test_data"); // TODO: Remove in the final version
    receiver_.send(test); // TODO: Remove in the final version

    ec_data_ = receiver_.receive();
    std::cout << "Received EC data: " << *ec_data_ptr_ << std::endl; // TODO: Remove in the final version
}


void EctaneHandler::updateCommandPacket() {
    std::cout << "Ectane address: " << ectane_address_ << std::endl;
    command_packet_.state = state_;
    // oarsome's post here:
    //      https://stackoverflow.com/questions/1076714/max-length-for-client-ip-address
    // suggests IP address string encoding ::ffff:000.000.000.000
    //strcpy(command_packet_.ip_address, ("::ffff:" + ectane_address_ + "\0").c_str()); // Buffer overflow
    //strcpy(command_packet_.ip_address, (":ffff:" + ectane_address_ + "\0").c_str()); // OK
    strcpy(command_packet_.ip_address, ("ffff:" + ectane_address_ + "\0").c_str()); // OK
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
    // TODO: I think I am getting UB when going from a char to a string which shows as a buffer overflow 
    //       when compiling in Release mode
    // https://stackoverflow.com/questions/66074911/undefined-behavior-when-converting-char-to-stdstring-c
    // https://stackoverflow.com/questions/20794832/pointers-and-strings-c

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
}


void EctaneHandler::changeState(State state) {
    state_ = state;
    updateCommandPacket();
    std::cout << "Outgoing command string: " << command_string_ << std::endl;
    sender_.send(command_string_);
}


void EctaneHandler::doNothing() {
    changeState(State::do_nothing);
}


void EctaneHandler::socketInitialise() {
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
