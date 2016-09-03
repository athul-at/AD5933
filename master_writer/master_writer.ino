// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

#define SLAVE_ID1 0x0D


// Function to convert a 2 char array (conatining 2 hex digits) to a single byte hexadecimal number.
byte char2hex(char buf[])
{
  byte msb = 0 ;
  byte lsb = 0 ;
 if((isHexadecimalDigit(buf[0]))&&((isHexadecimalDigit(buf[0]))))
 {
  if(isDigit(buf[0]))
   {
     msb = (buf[0]-'0');
   }
  else if (isUpperCase(buf[0]))
   {
    msb = (buf[0]-55);
   }
   else if (isLowerCase(buf[0]))
   {
    msb = (buf[0]-87);
   }
   else
   {
    Serial.println("Invalid Input");
   }
  if(isDigit(buf[1]))
   {
     lsb = (buf[1]-'0');
   }
  else if (isUpperCase(buf[1]))
   {
    lsb = (buf[1]-55);
   }
   else if (isLowerCase(buf[1]))
   {
    lsb = (buf[1]-87);
   }
   else
   {
    Serial.println("Invalid Input");
   }
  }
  else
  {
  Serial.println("Invalid Input");
  }
  //Serial.print("buf[0]: "); Serial.print(buf[0]);Serial.print(" buf[1]: ");Serial.println(buf[1]);
  //Serial.print("MSB: "); Serial.print(msb,HEX);Serial.print(" LSB: ");Serial.println(lsb,HEX);
  return ((msb*16)+lsb);
}



void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
  Serial.println("Master IIC device, Arduino Uno");
  Serial.println(" ");
  Serial.println(" ");
}



void loop() 
{
  byte adrs = 0x00;
  byte data = 0x00;
  byte write_cmd = 0x00;
  byte read_cmd = 0x01;
  byte cmd = 0xff;
  
  char adrs_buf[2];
  char data_buf[2];
  char cmd_buf[1];
  
  
// Asking weather to read or write
  Serial.print("Read (1) / Write (0) : ");
  while(Serial.available() < 1);
  Serial.readBytes(cmd_buf,1);
  Serial.println(cmd_buf[0]);
  if (cmd_buf[0] == '0')
   cmd = write_cmd;
  else if (cmd_buf[0] == '1')
   cmd = read_cmd;
  else
    Serial.println ("Invalid Command: Write = 0, Read = 1");
  
// Getting Write Address from user
  Serial.print("Write/Read Address (2 byte, HEX): ");
  Serial.flush();
  while(Serial.available() < 2);
  Serial.readBytes(adrs_buf,2); 
  adrs = char2hex(adrs_buf);
  Serial.print("Read(hex):");
  Serial.println(adrs,HEX);

//Getting write data from user
  Serial.print("Write Data(2 byte, HEX): ");
  Serial.flush();
  while(Serial.available() < 2);
  Serial.readBytes(data_buf,2);
  data = char2hex(data_buf);
  Serial.print("Read(hex) :");
  Serial.println(data,HEX);
  send_command(adrs,cmd,data);
  Serial.println(" ");
  Serial.println(" ");
}

void send_command(byte adrs,byte cmd,byte data)
{
  Wire.beginTransmission(SLAVE_ID1);                 
  Wire.write(adrs);
  Wire.write(cmd);
  Wire.write(data);
  Wire.endTransmission();      // stop transmitting
  delay(500);
}

