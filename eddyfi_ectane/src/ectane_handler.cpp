#include "ectane_handler.h"


EctaneHandler::EctaneHandler()
     : ectane_address_("192.168.2.39"),

       recipient_address_("127.0.0.1"), 
       send_port_(24010),
       sender_(recipient_address_, send_port_),

       sender_address_("127.0.0.1"),
       receive_port_(24011),
       receiver_(sender_address_, receive_port_),

       state_(State::do_nothing),
       configuration_(Configurations::lf_probe_axial_1_channel),
       acquisition_rate_(40),
       gain_(1),
       frequency_(1000),
       voltage_(1.0)
{
       populateCommandPacket();
}


void EctaneHandler::spinSender()
{
    std::string message_1("Hi!");

    sender_.send(message_1);
    std::string received_message_1 = sender_.receive();
    std::cout << "Received message: " << received_message_1 << std::endl;
}


void EctaneHandler::spinReceiver()
{
    std::string message_2(commandToString(command_packet_));

    receiver_.send(message_2);
    std::string received_message_2 = receiver_.receive();
    std::cout << "Received message: " << received_message_2 << std::endl;
}


void EctaneHandler::run()
{
    std::thread sender_thread(&EctaneHandler::spinSender, this);
    std::thread receiver_thread(&EctaneHandler::spinReceiver, this);
    sender_thread.join();
    receiver_thread.join();
}


void EctaneHandler::populateCommandPacket()
{
    command_packet_.state = state_;
    // oarsome's post here:
    //      https://stackoverflow.com/questions/1076714/max-length-for-client-ip-address
    // suggests IP address string encoding ::ffff:000.000.000.000
    strcpy(command_packet_.ip_address, ("ffff::" + ectane_address_).c_str());
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
}


std::string EctaneHandler::commandToString(CommandPacket command)
{
    std::string str_command = //std::to_string((int32_t)command.state) + "," +
                              std::to_string(command.state) + "," +
                              command.ip_address + "," +
                              std::to_string(command.configuration) + "," +
                              std::to_string(command.acquisition_rate) + "," +
                              std::to_string(command.gain) + "," +
                              std::to_string(command.frequency_1) + "," +
                              std::to_string((int32_t)command.voltage_1) + "," +
                              std::to_string(command.frequency_2) + "," +
                              std::to_string((int32_t)command.voltage_2) + "," +
                              std::to_string(command.frequency_3) + "," +
                              std::to_string((int32_t)command.voltage_3) + "," +
                              std::to_string(command.frequency_4) + "," +
                              std::to_string((int32_t)command.voltage_4) + "," +
                              std::to_string(command.frequency_5) + "," +
                              std::to_string((int32_t)command.voltage_5) + "," +
                              std::to_string(command.frequency_6) + "," +
                              std::to_string((int32_t)command.voltage_6) + "," +
                              std::to_string(command.frequency_7) + "," +
                              std::to_string((int32_t)command.voltage_7) + "," +
                              std::to_string(command.frequency_8) + "," +
                              std::to_string((int32_t)command.voltage_8);

    return str_command;
}


void EctaneHandler::changeState(State state)
{
    state_ = state;
    populateCommandPacket();
    commandToString(command_packet_);
}


void EctaneHandler::doNothing()
{
    changeState(State::do_nothing);
}


void EctaneHandler::socketInitialise()
{
    changeState(State::socket_initialise);
}

void EctaneHandler::connecttoDevice()
{
    changeState(State::connect_to_device);
}


void EctaneHandler::setUpDevice()
{
    changeState(State::set_up_device);
}

void EctaneHandler::balanceDevice()
{
    changeState(State::balance_device);
}


void EctaneHandler::acquireData()
{
    changeState(State::acquire_data);
}


void EctaneHandler::stopData()
{
    changeState(State::stop_data);
}


void EctaneHandler::disconnectDevice()
{
    changeState(State::disconnect_device);
}