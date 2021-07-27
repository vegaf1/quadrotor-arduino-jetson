#include <Servo.h>


Servo ESC1,ESC2,ESC3,ESC4; // object to control the ESC
int incoming; //read the incoming data
int motor[4] = {1148,1148,1148,1148}; // initialize the pwm values of the motors at min position
int maxthrot = 1832;
int minthrot = 1148; 
int voltpin = A1;
int readValue;
int data;
float impedance_factor = 0.918;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(voltpin,INPUT);
pinMode(3, OUTPUT);
pinMode(9, OUTPUT);
pinMode(11, OUTPUT);
pinMode(6, OUTPUT);
ESC1.attach(3, maxthrot, minthrot);
ESC2.attach(9, maxthrot, minthrot);
ESC3.attach(11, maxthrot, minthrot);
ESC4.attach(6, maxthrot, minthrot);
while(!Serial)
  {
    Serial.println("waiting");
  }

calibration();

//arm();

}

void arm()
{
ESC1.writeMicroseconds(maxthrot);
ESC2.writeMicroseconds(maxthrot);
ESC3.writeMicroseconds(maxthrot);
ESC4.writeMicroseconds(maxthrot);
delay(2000);
ESC1.writeMicroseconds(minthrot+200);
ESC2.writeMicroseconds(minthrot+200);
ESC3.writeMicroseconds(minthrot+200);
ESC4.writeMicroseconds(minthrot+200);
delay(2000);
Serial.println("1");

}
void calibration() 
{
  //calibration
//Serial.println("throttle up 1") ;

ESC1.writeMicroseconds(maxthrot);
delay(3000);
//Serial.println("throttle down 1"); 
ESC1.writeMicroseconds(minthrot);
delay(3000);

//Serial.println("throttle up 2") ;
ESC2.writeMicroseconds(maxthrot);
delay(3000);
//Serial.println("throttle down 2"); 
ESC2.writeMicroseconds(minthrot);
delay(3000);

//Serial.println("throttle up 3") ;
ESC3.writeMicroseconds(maxthrot);
delay(3000);
//Serial.println("throttle down 3"); 
ESC3.writeMicroseconds(minthrot);
delay(3000);

//Serial.println("throttle up 4") ;
ESC4.writeMicroseconds(maxthrot);
delay(3000);
//Serial.println("throttle down 4"); 
ESC4.writeMicroseconds(minthrot);
delay(8000);
Serial.println("2");
}
void loop() {

  while(Serial.available())
  {
    float readValue = analogRead(voltpin);
    float converted = readValue*.0049*5.9785*impedance_factor;
    Serial.println(converted);
    for (int i =0; i<4; i++) //fill in all 4 motor value data
    {
      incoming = Serial.read();
      if (incoming != -1) //to avoid the end byte
      {
        motor[i] = incoming; //save to the motor array
      }
      //Serial.println("MOTOR 1"); // print to ensure everything works
      //Serial.println(motor[0]);
      //Serial.println("****************");
      //Serial.println("MOTOR 2");
      //Serial.println(motor[1]);
      //Serial.println("****************");
      //Serial.println("MOTOR 3");
      //Serial.println(motor[2]);
      //Serial.println("****************");
      //Serial.println("MOTOR 4");
      //Serial.println(motor[3]);
      //Serial.println("****************");
      ESC1.writeMicroseconds(motor[0]*10);
      ESC2.writeMicroseconds(motor[1]*10);
      ESC3.writeMicroseconds(motor[2]*10);
      ESC4.writeMicroseconds(motor[3]*10);
    }
//  float readValue = analogRead(voltpin);
//  float converted = readValue*.0049*5.9785*impedance_factor;
//  Serial.println(converted);
    //data = Serial.read();
    //Serial.print(data);
  }
  
  //float readValue = analogRead(voltpin);
  //float converted = readValue*.0049*5.9785*impedance_factor;
  //Serial.println(converted);
}
