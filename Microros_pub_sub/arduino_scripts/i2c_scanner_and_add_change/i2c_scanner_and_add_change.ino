/*
Steps to change address:
First verify what is address of component
Then change the Wire.beginTransmission(Address of component)
write the desire address to below given line
 upload the code by uncommenting the commented part
*/
#include <Wire.h>

void setup() {
  Wire.begin();

  Serial.begin(115200);
  while (!Serial); // Leonardo: wait for Serial Monitor
  Serial.println("\nI2C Scanner");
  Serial.print("Ready to Send command to TF-Luna\r\n");
// Uncomment the below for changing of address:
// Change the address
// Wire.beginTransmission(0x10);
// Wire.write(0x5A);
// Wire.write(0x05);
// Wire.write(0x0B);
// Wire.write(0x1E); // Change this line to change the address. (Hexadecimal , Example: 17 = 0X11, 18= 0X12, …)
// Wire.write(0x00);
// Wire.endTransmission(0);
// delay(100);
// // Save the address
// Wire.beginTransmission(0x10);
// Wire.write(0x5A);
// Wire.write(0x04);
// Wire.write(0x11);
// Wire.write(0x6F);
// Wire.endTransmission(0);
}

void loop() {
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000); // Wait 5 seconds for next scan
}
