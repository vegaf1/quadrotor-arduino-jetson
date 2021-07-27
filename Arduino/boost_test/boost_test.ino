#include "Servo.h"

Servo servo;
int motor[4] = {1148,1148,1148,1148};
int data[16];
int incomingdata;
int i = 0;
void setup() {
  Serial.begin(115200);
 
}

void loop() {
  while(Serial.available()) {
    int incomingdata = Serial.read();
    Serial.println("INCOMING DATA");
    Serial.println(incomingdata);
   
      data[i] = incomingdata;
      
      i+=1;

      if (i == 16)
      {
        i = 0;
        motor[0] = data[0]+(256*data[1]);
        motor[1] = data[4]+(256*data[5]);
        motor[2] = data[8]+(256*data[9]);
        motor[3] = data[12]+(256*data[13]);
         
      }

      Serial.println("MOTOR 1"); // print to ensure everything works
      Serial.println(motor[0]);
      Serial.println("****************");
      Serial.println("MOTOR 2");
      Serial.println(motor[1]);
      Serial.println("****************");
      Serial.println("MOTOR 3");
      Serial.println(motor[2]);
      Serial.println("****************");
      Serial.println("MOTOR 4");
      Serial.println(motor[3]);
      Serial.println("****************");
   
}

}
