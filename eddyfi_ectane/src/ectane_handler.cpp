#include "ectane_handler.h"


using namespace std::chrono_literals;

EctaneHandler::EctaneHandler()
     : recipient_address_("127.0.0.1"), 
       send_port_(24010),
       sender_(recipient_address_, send_port_),
       sender_address_("127.0.0.1"),
       receive_port_(24011),
       receiver_(sender_address_, receive_port_),
       state_(State::do_nothing)
{
}


void EctaneHandler::run()
{
    std::thread sender_thread(&EctaneHandler::spinSender, this);
    std::thread receiver_thread(&EctaneHandler::spinReceiver, this);
    sender_thread.join();
    receiver_thread.join();
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
    std::string message_2("Hello!");

    receiver_.send(message_2);
    std::string received_message_2 = receiver_.receive();
    std::cout << "Received message: " << received_message_2 << std::endl;
}


std::string EctaneHandler::commandToString(CommandPacket command)
{
    // oarsome's post here:
    //      https://stackoverflow.com/questions/1076714/max-length-for-client-ip-address
    // suggests IP address string encoding ::ffff:000.000.000.000

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