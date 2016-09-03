// I2C slave device that emulates the functioning of the AD5933 I2C Interface
// Based on Wire Slave Receiver by Nicholas Zambetti <http://www.zambetti.com>
// Author      : Athul Asokan Thulasi
// Created on  : September 1, 2016

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
// Temperature Data
byte reg_92 = 0;
byte reg_93 = 0;
// Real Data
byte reg_94 = 0;
byte reg_95 = 0;
// Imaginary Data
byte reg_96 = 0;
byte reg_97 = 0;
/*******************************************/

void execute_cmd(byte adrs, byte cmd, byte data)
{
  if(cmd == 0x01) // Read
  {
    switch (adrs) 
    {
    case 0x80:
      Wire.write(reg_80) ;
      Serial.print("Register 80 Value: ");
      Serial.println(reg_80,HEX);
      break;
    case 0x81:
      Wire.write(reg_81) ;
       Serial.print("Register 81 Value: ");
      Serial.println(reg_81,HEX);
      break;
    case 0x82:
      Wire.write(reg_82) ;
       Serial.print("Register 82 Value: ");
      Serial.println(reg_82,HEX);
      break;
    case 0x83:
      Wire.write(reg_83) ;
       Serial.print("Register 83 Value: ");
      Serial.println(reg_83,HEX);
      break;
    case 0x84:
      Wire.write(reg_84) ;
       Serial.print("Register 84 Value: ");
      Serial.println(reg_84,HEX);
      break;
    case 0x85:
      Wire.write(reg_85) ;
       Serial.print("Register 85 Value: ");
      Serial.println(reg_85,HEX);
      break;
    case 0x86:
      Wire.write(reg_86) ;
      Serial.print("Register 86 Value: ");
      Serial.println(reg_86,HEX);
      break;
    case 0x87:
      Wire.write(reg_87) ;
      Serial.print("Register 87 Value: ");
      Serial.println(reg_87,HEX);
      break;
    case 0x88:
      Wire.write(reg_88) ;
      Serial.print("Register 88 Value: ");
      Serial.println(reg_88,HEX);
      break;
    case 0x89:
      Wire.write(reg_89) ;
      Serial.print("Register 89 Value: ");
      Serial.println(reg_89,HEX);
      break;
    case 0x8A:
      Wire.write(reg_8A) ;
      Serial.print("Register 8A Value: ");
      Serial.println(reg_8A,HEX);
      break; 
    case 0x8B:
      Wire.write(reg_8B) ;
      Serial.print("Register 8B Value: ");
      Serial.println(reg_8B,HEX);
      break; 
    case 0x8F:
      Wire.write(reg_8F) ;
      Serial.print("Register 8F Value: ");
      Serial.println(reg_8F,HEX);
      break;
    case 0x92:
      Wire.write(reg_92) ;
      Serial.print("Register 92 Value: ");
      Serial.println(reg_92,HEX);
      break;
    case 0x93:
      Wire.write(reg_93) ;
      Serial.print("Register 93 Value: ");
      Serial.println(reg_93,HEX);
      break;
    case 0x94:
      Wire.write(reg_94) ;
      Serial.print("Register 94 Value: ");
      Serial.println(reg_94,HEX);
      break;
    case 0x95:
      Wire.write(reg_95) ;
      Serial.print("Register 95 Value: "); 
      Serial.println(reg_95,HEX);
      break;
    case 0x96:
      Wire.write(reg_96) ;
      Serial.print("Register 96 Value: "); 
      Serial.println(reg_96,HEX);
      break;
    case 0x97:
      Wire.write(reg_97) ;
      Serial.print("Register 97 Value: "); 
      Serial.println(reg_97,HEX);
      break;
    default:
      Serial.println("Invalid Address !: Read failed");
    break;
  }
  }
  else if (cmd == 0x00) // Write
  {
    switch (adrs) 
    {
    case 0x80:
      reg_80 = data ;
      break;
    case 0x81:
      reg_81 = data ;
      break;
    case 0x82:
      reg_82 = data ;
      break;
    case 0x83:
      reg_83 = data ;
      break;
    case 0x84:
      reg_84 = data ;
      break;
    case 0x85:
      reg_85 = data ;
      break;
    case 0x86:
      reg_86 = data ;
      break;
    case 0x87:
      reg_87 = data ;
      break;
    case 0x88:
      reg_88 = data ;
      break;
    case 0x89:
      reg_89 = data ;
      break;
    case 0x8A:
      reg_8A = data ;
      break; 
    case 0x8B:
      reg_8B = data ;
      break; 
    case 0x8F:
      reg_8F = data ;
      break;
    case 0x92:
      reg_92 = data ;
      break;
    case 0x93:
      reg_93 = data ;
      break;
    case 0x94:
      reg_94 = data ;
      break;
    case 0x95:
      reg_95 = data ;
      break;
    case 0x96:
      reg_96 = data ;
      break;
    case 0x97:
      reg_97 = data ;
      break;
    default:
      Serial.println("Invalid Address !: Write failed");
    break;
  }
    
  }
  else
  {
    Serial.println(" Invalid Command (W=0,R=1)");
  }
}


void setup() {
  Wire.begin(0x0D);                // join i2c bus with address 0x0D
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() 
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  Serial.print("Received ");
  Serial.print(howMany);
  Serial.println(" bytes");
   if (Wire.available() == 3)  //adrs,cmd,data
   {
    byte adrs = Wire.read(); 
    byte cmd  = Wire.read();
    byte data = Wire.read();
    Serial.print("Address: ");
    Serial.print(adrs,HEX);
    Serial.print("  Command (W=0,R=1): ");
    Serial.print(cmd,HEX);
    Serial.print("  Data: ");
    Serial.println(data,HEX);
    execute_cmd(adrs,cmd,data);
   }
   else
   {
    Serial.println("Invalid Command String :: Check number of bytes sent !");
   }
 }
 
