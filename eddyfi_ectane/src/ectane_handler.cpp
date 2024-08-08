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
       ec_data_("null"), ec_data_ptr_(&ec_data_),

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
       voltage_(voltage),
       flag(false), count(0)
{
    updateCommandPacket();
}


void EctaneHandler::readData() {
    ec_data_ = receiver_.receive();

    ///////////////////////////////////////////////
    // Parsing debugging
    ///////////////////////////////////////////////

    int max_buff_size = (32 * 8 * 5 ) * 32 / 8;

    size_t n_bytes = ec_data_.size();

    std::cout << "#################################" << std::endl;
    std::cout << "Read a string of" << std::endl;
    std::cout << "n Bytes: " << n_bytes << std::endl;

    //char   c_buffer[n_bytes];       //  8 bits per element
    //short  buffer[n_bytes / 2];   // 16 bits per element
    //int    buffer[n_bytes / 4];   // 32 bits per element
    uint32_t buffer[max_buff_size / 4];   // 32 bits per element

    //bzero(&c_buffer, n_bytes);
    bzero(&buffer, max_buff_size);

    //strcpy(c_buffer, ec_data_.c_str());
    //std::memcpy(&buffer, &c_buffer, n_bytes);

    //std::memcpy(&c_buffer, ec_data_.c_str(), n_bytes);

    std::memcpy(&buffer, ec_data_.c_str(), n_bytes);

    std::cout << "--------------------" << std::endl;
    std::cout << "      New Cycle     " << std::endl;
    std::cout << "--------------------" << std::endl;

    /*
    int others = (
        31                          // Timeslots per observation
        * 8                         // Channels per timeslot
        * 5                         // Fields per channel (5 Freqs + 1 Voltage)
        //+ 3                         // Encoders
        //+ 32 / 4                    // Saturations
        //+ 1                         // Line 583 of hostdemo.c says ten | Seems wrong to me looks like it ought to be one
        )
        * 32
        / CHAR_BIT;

    //int n = sizeof(buffer) / sizeof(buffer[0]);
    int n = (sizeof(buffer) - others) / sizeof(buffer[0]);

    std::cout << "n: " << n << std::endl;
    std::cout << "others:" << others << std::endl;
    */

    //for (int i = 0; i < n; i++) {
    for (int i = 0; i < 1; i++) {
        if (i % 5 == 0) {
            std::cout << "Input ===---" << std::endl;
        }
        //std::cout << "Buffer value uint32_t: " << buffer[i] << std::endl;
        std::cout << "Buffer value      int: " << (int)buffer[i] << std::endl;
        if ((int)buffer[i] == 0){
            std::cout << "Skipping empty buffer..." << std::endl;
        } else if (i % 5 < 5) {
            std::cout << "Frequency " << (i % 5) + 1 << " ---" << std::endl;
            // MSB
            std::cout << "Imaginary Value: " << (int)(short)(((buffer[i]) >> 16) & 0x0000FFFF) << std::endl;
            // LSB
            std::cout << "     Real Value: " << (int)(short)(((buffer[i]) >> 0) & 0x0000FFFF) << std::endl;
        } else {
            std::cout << "Voltage ---" << std::endl;
            // MSB
            std::cout << " Should be zero: " << (int32_t)(int16_t)(((buffer[i]) >> 16) & 0x0000FFFF) << std::endl;
            // LSB
            std::cout << "     DC Voltage: " << (int32_t)(int16_t)(((buffer[i]) >> 0) & 0x0000FFFF) << std::endl;
        }
    }
    count++;
}


void EctaneHandler::updateCommandPacket() {
    command_packet_.state = state_;
    // oarsome's post here:
    //      https://stackoverflow.com/questions/1076714/max-length-for-client-ip-address
    // suggests IP address string encoding ::ffff:000.000.000.000
    //strcpy(command_packet_.ip_address, ("::ffff:" + ectane_address_ + "\0").c_str()); // Buffer overflow
    //strcpy(command_packet_.ip_address, (":ffff:" + ectane_address_ + "\0").c_str()); // OK
    //strcpy(command_packet_.ip_address, ("ffff:" + ectane_address_ + "\0").c_str()); // OK
    //strcpy(command_packet_.ip_address, (":ffff:" + ectane_address_).c_str());
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
    std::cout << "Outgoing command string: " << command_string_ << std::endl;
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
