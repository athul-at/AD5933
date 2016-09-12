
// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : September 6, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>           

# define DEBUG 1

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned short  temperature = 0;
unsigned long   impedanceK  = 0;
double          impedance   = 0;
double          gainFactor  = 0.0;
double          baseline_impedance =0.0;
double          sweat_impedance = 0.0;
double          delta_impedance = 0.0;
double          glucose_concentration = 0.0;
/******************************************************************************/

// Setup routine runs once when you press reset:
void setup() 
{                
  Serial.begin(9600); 
  Serial.println("##############################################################################");
  Serial.println("#                Sweat Glucose Measurement Program                           #");
  Serial.println("##############################################################################");
  Serial.println("");
  Wire.begin(); 
  /* Reset the device. */
  AD5933_Reset();
  Serial.println("Reset completed. .");
  /* Select the source of the AD5933 system clock. */
  AD5933_SetSystemClk(AD5933_CONTROL_INT_SYSCLK, 0);
  Serial.println("Clock Setup completed");
  /* Set range and gain. */
  AD5933_SetRangeAndGain(AD5933_RANGE_2000mVpp, AD5933_GAIN_X1);
  Serial.println("Setting range and gain done. .");
  /* Read the temperature. */
  temperature = AD5933_GetTemperature();
  Serial.println("");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.println("");
   /*Configure the sweep parameters */
  AD5933_ConfigSweep(100,       // 100 Hz
                       1000,        // 1000 Hz
                       500);        // 500 increments
  Serial.println("Setting the sweep settings completed. . ");
  Serial.println("");
}

// Loop routine runs over and over again forever
void loop()
{
   //Serial.println("Inside the loop function");
   char usr_sel;
   usr_sel = user_input();
   execute_user_function(usr_sel);
}


char user_input()
{
 start_menu:
 Serial.println("##############################################################################");
 Serial.println("# Run the menu options sequentially                                          #");
 Serial.println("# 1. Run Calibration                                                         #");
 Serial.println("# 2. Measure Impedance without sweat sample                                  #");
 Serial.println("# 3. Measure Impedance with sweat sample                                     #");
 Serial.println("# 4. Calculate Sweat Concentration                                           #");
 #ifdef DEBUG
 Serial.println("##############################################################################");
 Serial.println("# 5. Write to Register                                                       #");
 Serial.println("# 6. Read from Register                                                      #");
 Serial.println("# 7. Register Dump                                                           #");
 Serial.println("##############################################################################");
 #endif
 Serial.println("##############################################################################");
 Serial.println("");
 while(Serial.available() == 0);
 char in_byte = Serial.read();
 Serial.print("User selected option:   ");
 Serial.println(in_byte-'0'); 
  #ifdef DEBUG
    if ((in_byte >= '1')&&(in_byte <= '7'))
  #else
    if ((in_byte >= '1')&&(in_byte <= '4'))
  #endif
  {
   #ifdef DEBUG
   Serial.println("Accepted user input");
   Serial.println("");
   Serial.println("");
   #endif
   return in_byte;
  }
  else
  {
   Serial.println("Invalid Input !!");
   Serial.println("");
   Serial.println("");
   goto start_menu;
  }
}

void execute_user_function(char inByte)
{
   unsigned char in_address = 0;;
   unsigned long  in_data = 0;
   unsigned long read_value = 0;
   byte index;
   char adrs_buf[2];
   char data_buf[2];
 switch(inByte)
 {
   case '1':
       Serial.println("Calibarting the AD5933. . .");
       Serial.println("");
       Serial.println("");
        /* Start the sweep operation. */
        AD5933_StartSweep();
        /* Calculate the gain factor for an impedance of 47kohms. */
        gainFactor = AD5933_CalculateGainFactor(47000,AD5933_FUNCTION_REPEAT_FREQ);
       Serial.print("Calculated Calibration Gain: ");
       Serial.println(gainFactor);
       Serial.println("");
       Serial.println("");
       break;
   case '2':
       Serial.println("Measuring baseline impedance. . .");
       Serial.println("");
       Serial.println("");
       impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_REPEAT_FREQ);
       impedanceK = (unsigned long)impedance;
       impedanceK /= 1000;
       baseline_impedance = impedanceK;
       Serial.print("Baseline Impedance (K Ohms): ");
       Serial.println(baseline_impedance);
       Serial.println("");
       Serial.println("");
       break;
   case '3':
      Serial.println("Measuring Impedance with sweat sample. . .");
      Serial.println("");
      Serial.println("");
      impedance = 0.0 ;
      impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_REPEAT_FREQ);
      impedanceK = (unsigned long)impedance;
      impedanceK /= 1000;
      sweat_impedance = impedanceK;
      Serial.print("Impedance with sweat sample (K Ohms): ");
      Serial.println(sweat_impedance);
      Serial.println("");
      Serial.println("");
      break;
   case '4': 
      Serial.println("Calculating the Sweat Concentration. . .");
      Serial.println("");
      Serial.println("");
      delta_impedance = abs(baseline_impedance - sweat_impedance);
      glucose_concentration = calculate_concentration(delta_impedance);
      Serial.print("Glucose Concentration (microGram/milliLiter): ");
      Serial.println(glucose_concentration);
      Serial.println("");
      Serial.println("");
      break;
   case '5':
   write_reg:
      Serial.print("Which register do you need to write ? (HEX): ");
      Serial.flush();
      while(Serial.available() < 2);
      Serial.readBytes(adrs_buf,2); 
      in_address = char2hex(adrs_buf);
      Serial.println(in_address,HEX);
      if(!(((in_address>=0x80)&&(in_address<=0x8B))||(in_address==0x8F)||((in_address>=0x93)&&(in_address<=0x97))))
      {
        Serial.println("Invalid register address ! Valid Range: (0x80-0x8B;0x8F;0x93-0x97)");
      goto write_reg;
      }
      Serial.print("What value do you need to write ? (HEX):");
      Serial.flush();
      while(Serial.available() < 2);
      Serial.readBytes(adrs_buf,2);
      in_data = char2hex(adrs_buf);
      Serial.println(in_data,HEX);
      AD5933_SetRegisterValue(in_address,in_data,1);
      break;
    case '6':
     read_reg:
      Serial.print("What register do you need to read ? (HEX, 1 BYTE):");
      Serial.flush();
      while(Serial.available() < 2);
      Serial.readBytes(adrs_buf,2); 
      in_address = char2hex(adrs_buf);
      Serial.println(in_address,HEX);
      if(!(((in_address>=0x80)&&(in_address<=0x8B))||(in_address==0x8F)||((in_address>=0x93)&&(in_address<=0x97))))
      {
        Serial.println("Invalid register address ! Valid Range: (0x80-0x8B;0x8F;0x93-0x97)");
      goto read_reg;
      }
      read_value = AD5933_GetRegisterValue(in_address,1);
      Serial.print("Read value : ");
      Serial.println(read_value,HEX);
      break;
    case '7':
    Serial.println("Register Address, Register Value");
      for (index = 0x80; index <= 0x97; index++)
      {
       if(!(((index>=0x80)&&(index<=0x8B))||(index==0x8F)||((index>=0x93)&&(index<=0x97))))
        {
        continue;
        }
       else
        {
          read_value = AD5933_GetRegisterValue(index,1);
          Serial.print("0x");
          Serial.print(index,HEX);
          Serial.print(", 0x");
          Serial.println(read_value,HEX);
        }
      }
      break;
   default: 
      Serial.println("Invalid Option !!");
      Serial.println("");
      Serial.println("");
      break;
 }
}

// Function that calculates the sweat glucose concentration from the change in impedance.
// Rewrite this function with a look up table of a poylnomial fit.
double calculate_concentration(double delta_impedance)
{
  return 31.7 + 2.55* log(delta_impedance-0.01); // Natural Log
}

// Function to convert a 2 char array (conatining 2 hex digits) to a single byte hexadecimal number.
unsigned long char2hex(char buf[])
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
  #ifdef DEBUG
  Serial.print("MSB: "); Serial.print(msb,HEX);Serial.print(" LSB: ");Serial.println(lsb,HEX);
  #endif
  return ((msb*16)+lsb);
}

