// IIC Master device that intefaces with AD5933 I2C interface
// Based on Wire Master Writer by Nicholas Zambetti <http://www.zambetti.com>
// Author        : Athul Asokan Thulasi
// Created on    : September 1, 2016
// Last Modified : September 2, 2016


#include <Wire.h>

#define SLAVE_ID1 0x0D


void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
  Serial.println("Master IIC device, Arduino Uno");
  Serial.println(" ");
  Serial.println(" ");
}


void loop() 
{
  start:
  byte reg_adrs = 0x00;
  byte data = 0x00;
  byte write_cmd = 0x00;
  byte read_cmd = 0x01;
  byte cmd = 0xff;
  uint8_t slave_adrs = SLAVE_ID1;
  char adrs_buf[2];
  char data_buf[2];
  char cmd_buf[1];
  byte read_data;
  
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
  {
    Serial.println ("Invalid Command: Write = 0, Read = 1");
    goto start;
  }
  
// Getting Write Address from user
  Serial.print("Write/Read Address (2 byte, HEX): ");
  Serial.flush();
  while(Serial.available() < 2);
  Serial.readBytes(adrs_buf,2); 
  reg_adrs = char2hex(adrs_buf);
  Serial.println(reg_adrs,HEX);
  if(!(((reg_adrs>=0x80)&&(reg_adrs<=0x8B))||(reg_adrs==0x8F)||((reg_adrs>=0x93)&&(reg_adrs<=0x97))))
  {
    Serial.println("Invalid register address ! (0x80-0x8B;0x8F;0x93-0x97)");
    goto start;
  }
  
  if(cmd == 0)
  {
   //Getting write data from user
   Serial.print("Write Data(2 byte, HEX): ");
   Serial.flush();
   while(Serial.available() < 2);
   Serial.readBytes(data_buf,2);
   data = char2hex(data_buf);
   Serial.println(data,HEX);
  }
  // IIC Write or Read

  if(cmd == 0)
  send_write_command(slave_adrs,reg_adrs,data);
  else if(cmd == 1)
  {
  read_data = send_read_command(slave_adrs,reg_adrs);
  Serial.print("Received value (hex):");
  Serial.println(read_data,HEX);
  }
  Serial.println(" ");
  Serial.println(" ");
}


// Function to write one byte to slave device
void send_write_command(uint8_t slave_adrs, byte reg_adrs,byte data)
{
  Wire.beginTransmission(slave_adrs);                 
  //Wire.write(slave_adrs);
  Wire.write(reg_adrs);
  Wire.write(data);
  Wire.endTransmission();      // stop transmitting
  delay(500);
}


//Function to read one byte from slave device
byte send_read_command(uint8_t slave_adrs,byte reg_adrs)
{
  byte adrs_ptr = 0xB0;
  Wire.beginTransmission(slave_adrs);                 
  //Wire.write(slave_adrs);
  Wire.write(adrs_ptr);
  Wire.write(reg_adrs);
  Wire.endTransmission();      // stop transmitting
  delay(500);
  Wire.requestFrom(slave_adrs,uint8_t(1));
  while(Wire.available()<1);
  return Wire.read();  
}

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
