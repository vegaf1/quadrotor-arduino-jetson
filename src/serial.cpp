//#include <zmq.hpp>
#include <iostream>
#include <boost/asio.hpp>
#include <sstream>
char* message;

int main(int arcg, char *argv[])
{
    //zmq::context_t context (1);
    boost::asio::io_service ioservice; //set io service
    boost::asio::serial_port serial(ioservice, "/dev/ttyACM0"); //link to serial port

    serial.set_option(boost::asio::serial_port_base::baud_rate(115200)); //establish teh baud rate
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8))); //read/write 8 bits

    std::string s = "u";
    boost::asio::streambuf b;
    std::ostream os(&b);
    os << s;
    while(1)
{
    boost::asio::write(serial, b.data());
}
    //if (serial.is_open()) {
        //serial.close();
    //}

    return 0;
}
