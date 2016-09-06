// I2C slave device that emulates the functioning of the AD5933 I2C Interface
// Based on Wire Slave Receiver by Nicholas Zambetti <http://www.zambetti.com>
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : September 6, 2016

#include <Wire.h>

/******************************************
 *       Register space of AD5933         *
 ******************************************/
// Control Register
byte reg_80 = 0;
byte reg_81 = 0;
// Start Frequency
byte reg_82 = 0;
byte reg_83 = 0;
byte reg_84 = 0;
// Increment Frequency
byte reg_85 = 0;
byte reg_86 = 0;
byte reg_87 = 0;
// Number of Increments
byte reg_88 = 0;
byte reg_89 = 0;
// Number of settling time cycles
byte reg_8A = 0;
byte reg_8B = 0;
// Status Register
byte reg_8F = 0;
// Temperature Data (25 C)
byte reg_92 = 0x03;
byte reg_93 = 0x20;
// Real Data
byte reg_94 = 0;
byte reg_95 = 10;
// Imaginary Data
byte reg_96 = 0;
byte reg_97 = 10;
/*******************************************/

byte data_reg = 0xff;

void setup() {
  Wire.begin(0x0D);                // join i2c bus with address 0x0D
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Serial.begin(9600);           // start serial for output
}

void loop() 
{
  delay(100);
}

// Function that executes whenever data is received from master 
void receiveEvent(int howMany) 
{
  //Serial.print("Received ");
  //Serial.print(howMany);
  //Serial.println(" bytes");
   if (Wire.available() == 2)  //adrs,data OR adrs_ptr,adrs
   {
    byte byte1 = Wire.read(); 
    byte byte2  = Wire.read();
    Serial.print("Byte1: ");
    Serial.print(byte1,HEX);
    Serial.print("  Byte2: ");
    Serial.println(byte2,HEX);
    execute_cmd(byte1,byte2);
   }
   else
   {
    Serial.println("Invalid Command String :: Check number of bytes sent !");
   }
 }


// Function that executes when the master request for data.
void requestEvent()
{
 Wire.write(data_reg);
}

// Function that writes to register OR updates the data_regiser with requested data
void execute_cmd(byte byte1, byte byte2)
{
  if(byte1 == 0xB0)  // Address Pointer 
  {
    if(byte2 == 0x8F) // Checking for status
    {
      reg_8F = 0x0F; // All done
    }
    switch (byte2) // Address
    {
    case 0x80:
      data_reg = reg_80; 
      Serial.print("Register: 80 Value: ");
      Serial.println(reg_80,HEX);
      break;
    case 0x81:
      data_reg = reg_81;
       Serial.print("Register: 81 Value: ");
      Serial.println(reg_81,HEX);
      break;
    case 0x82:
      data_reg = reg_82;
       Serial.print("Register: 82 Value: ");
      Serial.println(reg_82,HEX);
      break;
    case 0x83:
      data_reg = reg_83;
       Serial.print("Register: 83 Value: ");
      Serial.println(reg_83,HEX);
      break;
    case 0x84:
      data_reg = reg_84;
       Serial.print("Register: 84 Value: ");
      Serial.println(reg_84,HEX);
      break;
    case 0x85:
      data_reg = reg_85;
       Serial.print("Register: 85 Value: ");
      Serial.println(reg_85,HEX);
      break;
    case 0x86:
      data_reg = reg_86;
      Serial.print("Register: 86 Value: ");
      Serial.println(reg_86,HEX);
      break;
    case 0x87:
      data_reg = reg_87;
      Serial.print("Register: 87 Value: ");
      Serial.println(reg_87,HEX);
      break;
    case 0x88:
      data_reg = reg_88;
      Serial.print("Register: 88 Value: ");
      Serial.println(reg_88,HEX);
      break;
    case 0x89:
      data_reg = reg_89;
      Serial.print("Register: 89 Value: ");
      Serial.println(reg_89,HEX);
      break;
    case 0x8A:
      data_reg = reg_8A;
      Serial.print("Register: 8A Value: ");
      Serial.println(reg_8A,HEX);
      break; 
    case 0x8B:
      data_reg = reg_8B;
      Serial.print("Register: 8B Value: ");
      Serial.println(reg_8B,HEX);
      break; 
    case 0x8F:
      data_reg = reg_8F;
      Serial.print("Register: 8F Value: ");
      Serial.println(reg_8F,HEX);
      break;
    case 0x92:
      data_reg = reg_92;
      Serial.print("Register: 92 Value: ");
      Serial.println(reg_92,HEX);
      break;
    case 0x93:
      data_reg = reg_93;
      Serial.print("Register: 93 Value: ");
      Serial.println(reg_93,HEX);
      break;
    case 0x94:
      data_reg = reg_94;
      Serial.print("Register: 94 Value: ");
      Serial.println(reg_94,HEX);
      break;
    case 0x95:
      data_reg = reg_95;
      Serial.print("Register: 95 Value: "); 
      Serial.println(reg_95,HEX);
      break;
    case 0x96:
      data_reg = reg_96;
      Serial.print("Register: 96 Value: "); 
      Serial.println(reg_96,HEX);
      break;
    case 0x97:
      data_reg = reg_97;
      Serial.print("Register: 97 Value: "); 
      Serial.println(reg_97,HEX);
      break;
    default:
      Serial.println("Invalid Address !: Cannot issue a read to this Address");
    break;
  }
  }
  else  // Write
  {
    switch (byte1) 
    {
    case 0x80:
      reg_80 = byte2 ;
      break;
    case 0x81:
      reg_81 = byte2 ;
      break;
    case 0x82:
      reg_82 = byte2 ;
      break;
    case 0x83:
      reg_83 = byte2 ;
      break;
    case 0x84:
      reg_84 = byte2 ;
      break;
    case 0x85:
      reg_85 = byte2 ;
      break;
    case 0x86:
      reg_86 = byte2 ;
      break;
    case 0x87:
      reg_87 = byte2 ;
      break;
    case 0x88:
      reg_88 = byte2 ;
      break;
    case 0x89:
      reg_89 = byte2 ;
      break;
    case 0x8A:
      reg_8A = byte2 ;
      break; 
    case 0x8B:
      reg_8B = byte2 ;
      break; 
    case 0x8F:
      reg_8F = byte2 ;
      break;
    case 0x92:
      reg_92 = byte2 ;
      break;
    case 0x93:
      reg_93 = byte2 ;
      break;
    case 0x94:
      reg_94 = byte2 ;
      break;
    case 0x95:
      reg_95 = byte2 ;
      break;
    case 0x96:
      reg_96 = byte2 ;
      break;
    case 0x97:
      reg_97 = byte2 ;
      break;
    default:
      Serial.println("Invalid Address !: Write failed");
    break;
  }
 }
}

