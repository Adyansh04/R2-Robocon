//this code is for recieving the luna data, change the address of individual luna
#include <Wire.h>        // Instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1
 
TFLI2C tflI2C;
 
int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr1 = 0x10;  // Use this default I2C address
int16_t  tfAddr2 = 0x1E;
void setup(){
    Serial.begin(115200);  // Initalize serial port
    Wire.begin();           // Initalize Wire library
}
 
void loop(){
  
    if(tflI2C.getData(tfDist, tfAddr1)){
        Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
    }
     if(tflI2C.getData(tfDist, tfAddr2)){
        Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
    }
    delay(50);
}