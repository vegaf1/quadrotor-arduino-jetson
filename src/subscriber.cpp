#include <zmq.hpp>
#include <iostream>
#include <sstream>
#include <pwm.pb.h>
#include<pwm.pb.h>
#include<string>
#include <boost/asio.hpp>
#include <string.h>

using namespace std;



int main (int argc, char *argv [])
{
    boost::asio::io_service ioservice; //set io service
    boost::asio::serial_port serial(ioservice, "/dev/ttyACM0"); //link to serial port
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200)); //establish teh baud rate
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8))); //read/write 8 bits

    zmq::context_t context (1);

    //  Socket to talk to server
    std::cout << "Collecting updates from controller...\n" << std::endl;
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555"); //update correct port and ip

    subscriber.setsockopt(ZMQ_SUBSCRIBE, " ", 0);

    zmq::message_t rx_msg;

    std::cout << "Connected...\n" << std::endl;

    //cout << "connected";
    pwm::PWM motordata;
    //struct pwmmotors p1;
    int motors[4] = {1148,1148,1148,1148};
    boost::asio::streambuf b;
    std::ostream os(&b);

    while(true)
    {
        //quad::
        std::cout << "in here...\n" << std::endl;
        subscriber.recv(&rx_msg);
        std::string msg_str(static_cast<char*>(rx_msg.data()), rx_msg.size());
        //ParseFromString(rx_msg data)
        //std::cout << rx_msg.data();
        motordata.ParseFromString(msg_str);
        motors[0] = motordata.motor1();
        motors[1] = motordata.motor2();
        motors[2] = motordata.motor3();
        motors[3] = motordata.motor4();


        //os << motor1;

        boost::asio::write(serial, boost::asio::buffer(motors));

        std::cout << "Motor 1: " << motors[0] << "| size: "<< sizeof(motors[0])<< std::endl;
        std::cout << "Motor 2: " << motors[1] << std::endl;
        std::cout << "Motor 3: " << motors[2] << std::endl;
        std::cout << "Motor 4: " << motors[3] << std::endl;
    }

    return 0;

}
