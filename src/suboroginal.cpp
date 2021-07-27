#include <zmq.hpp>
#include <iostream>
#include <sstream>
#include <pwm.pb.h>

using namespace std;

int hello;

int main (int argc, char *argv[])
{
    zmq::context_t context (1);

    //  Socket to talk to server
    std::cout << "Collecting updates from controller...\n" << std::endl;
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555");


    //  Subscribe to zipcode, default is NYC, 10001
	//const char *filter = (argc > 1)? argv [1]: "10001 ";
    //subscriber.setsockopt(ZMQ_SUBSCRIBE, "");

    //  Process 100 updates
    int update_nbr;
    long total_temp = 0;
    for (update_nbr = 0; update_nbr < 100; update_nbr++) {

        zmq::message_t update;
        //int zipcode, temperature, relhumidity;

        hello = subscriber.recv(&update);

        std::cout<<update;

        //std::istringstream iss(static_cast<char*>(update.data()));
	//	iss >> zipcode >> temperature >> relhumidity ;

	//	total_temp += temperature;
    }
    //std::cout 	<< "Average temperature for zipcode '"<< filter
    	//		<<"' was "<<(int) (total_temp / update_nbr) <<"F"
    	//		<< std::endl;
    return 0;
}



