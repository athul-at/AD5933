// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : September 23, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>           


extern unsigned long freq_step;
extern unsigned long start_freq ;
extern unsigned short increment_number;
   
char user_input()
{
 start_menu:
 #ifdef DEBUG
 Serial.println("##############################################################################");
 Serial.println("# Setup Commands (Run first)                                                 #");
 Serial.println("# A. Reset AD5933                                                            #");
 Serial.println("# B. Set system Clock                                                        #");
 Serial.println("# C. Set range and gain                                                      #");
 Serial.println("# D. Configure Sweep                                                         #");
 Serial.println("# E. Get temperature ( Celsius )                                             #");
 Serial.println("# F. Increment frequency                                                     #");
 Serial.println("# G. Set Settling time                                                       #");
 Serial.println("# H. Plot Impedance Spectrum                                                 #");
 Serial.println("# I. Set AD5933 to powerdown mode                                            #");
 Serial.println("# J. Set AD5933 to standby mode                                              #");
 Serial.println("# K. Two point calibration                                                   #");
 #endif
 Serial.println("##############################################################################");
 Serial.println("# Run the menu options sequentially                                          #");
 Serial.println("# 0. Start Sweep                                                             #");
 Serial.println("# 1. Run Calibration                                                         #");
 Serial.println("# 2. Measure Impedance without sweat sample                                  #");
 Serial.println("# 3. Measure Impedance with sweat sample                                     #");
 Serial.println("# 4. Calculate Sweat Concentration                                           #");
 Serial.println("##############################################################################");
 #ifdef DEBUG
 Serial.println("# 5. Write to Register                                                       #");
 Serial.println("# 6. Read from Register                                                      #");
 Serial.println("# 7. Register Dump                                                           #");
 Serial.println("##############################################################################");
 #endif
 Serial.println("");
 while(Serial.available() == 0);
 char in_byte = Serial.read();
 Serial.print("User selected option:   ");
 Serial.println(in_byte); 
  #ifdef DEBUG 
    if (((in_byte >= '0')&&(in_byte <= '7'))||((in_byte >= 'A')&&(in_byte <= 'K'))||(((in_byte >= 'a')&&(in_byte >= 'k'))))
  #else
    if ((in_byte >= '0')&&(in_byte <= '4'))
  #endif
  {
   #ifdef DEBUG2
   Serial.println("Accepted user input");
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
   unsigned long calib_impedance = 79400; //79.4K Ohm

   #ifdef DEBUG
   char input;
   char in_byte;
   unsigned long frequency =0;
   unsigned long SettlingTime = 4;
   unsigned int Multiplier = 1;
   #endif
 switch(inByte)
 {
   case '0':
       Serial.println("Starting the frequency sweep");
       /* Start the sweep operation. */
        AD5933_StartSweep();
        break;
   case '1':
       Serial.println("Calibarting the AD5933. . .");
       Serial.println("");
       Serial.println("");
        /* Calculate the gain factor for an impedance of 47kohms. */
        #ifdef DEBUG
        Serial.print("Enter the Calibration resistance value: ");
        calib_impedance = read_number();
        Serial.println(calib_impedance);
        #endif
       gainFactor = AD5933_CalculateGainFactor(calib_impedance,AD5933_FUNCTION_REPEAT_FREQ);
       Serial.print("Calculated Calibration Gain: ");
       Serial.println((double)gainFactor);
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
       baseline_impedance = impedance;
       Serial.print("Baseline Impedance (Ohms): ");
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
      sweat_impedance = impedance;
      Serial.print("Impedance with sweat sample (Ohms): ");
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
#ifdef DEBUG
     case'A':
     case'a':
      AD5933_Reset();
      Serial.println("Reset completed. .");
     break;
     case'B':
     case'b':
     set_clock:
     Serial.println("##############################################################################");
     Serial.println("# Select Clock Source                                                        #");
     Serial.println("# 0. Internal                                                                #");
     Serial.println("# 1. External                                                                #");
     Serial.println("##############################################################################");  
     while(Serial.available() == 0);
     in_byte = Serial.read();
     Serial.print("User selected option:   ");
     Serial.println(in_byte-'0');
     if(in_byte == '0')
     {
      AD5933_SetSystemClk(AD5933_CONTROL_INT_SYSCLK, 16000000ul);
      Serial.println("Clock Setup completed");
     }
     else if (in_byte == '1')
     {
      Serial.println("##############################################################################");
      Serial.print("# Enter the external clock frequency (500- 16000000) Hz :");
      frequency = read_number();
      Serial.println(frequency);
      AD5933_SetSystemClk(AD5933_CONTROL_EXT_SYSCLK, frequency);
      Serial.println("Clock Setup completed");
     }
     else
     {
      Serial.println(" Invalid input: Clock not set ! ");
      goto set_clock;
     }
     break;
     case'D':
     case'd':
      Serial.println("##############################################################################");
      Serial.print("1. Enter the start frequency: ");
      start_freq = read_number();
      Serial.println(start_freq);
      Serial.print("2. Enter the frequency step size: ");
      freq_step = read_number();
      Serial.println(freq_step);
      Serial.print("3. Enter the number of steps: ");
      increment_number = (unsigned short)read_number();
      Serial.println(increment_number);
      AD5933_ConfigSweep(start_freq,freq_step,increment_number);
      Serial.println(" Sweep setup completed");
      Serial.println("##############################################################################"); 
     break;
     case'C':
     case'c':
     gain_setting:
     Serial.println("##############################################################################");
     Serial.println("# Select the output range, PGA gain                                          #");
     Serial.println("# 1. 2000mVpp, X1                                                            #");
     Serial.println("# 2. 1000mVpp, X1                                                            #");
     Serial.println("# 3. 400mVpp,  X1                                                            #");
     Serial.println("# 4. 200mVpp,  X1                                                            #");
     Serial.println("# 5. 2000mVpp, X5                                                            #");
     Serial.println("# 6. 1000mVpp, X5                                                            #");
     Serial.println("# 7. 400mVpp,  X5                                                            #");
     Serial.println("# 8. 200mVpp,  X5                                                            #");
     Serial.println("##############################################################################");
     while(Serial.available() == 0);
     input ='0';
     input = Serial.read();
     switch(input)
      {
        case '1':
         AD5933_SetRangeAndGain(AD5933_RANGE_2000mVpp, AD5933_GAIN_X1);
        break;
        case '2':
         AD5933_SetRangeAndGain(AD5933_RANGE_1000mVpp, AD5933_GAIN_X1);
        break;
        case '3':
         AD5933_SetRangeAndGain(AD5933_RANGE_400mVpp, AD5933_GAIN_X1);
        break;
        case '4':
         AD5933_SetRangeAndGain(AD5933_RANGE_200mVpp, AD5933_GAIN_X1);
        break;
        case '5':
         AD5933_SetRangeAndGain(AD5933_RANGE_2000mVpp, AD5933_GAIN_X5);
        break;
        case '6':
         AD5933_SetRangeAndGain(AD5933_RANGE_1000mVpp, AD5933_GAIN_X5);
        break;
        case '7':
         AD5933_SetRangeAndGain(AD5933_RANGE_400mVpp, AD5933_GAIN_X5);
        break;
        case '8':
         AD5933_SetRangeAndGain(AD5933_RANGE_200mVpp, AD5933_GAIN_X5);
        break;
        default:
        Serial.println("Invalid input");
        goto gain_setting;
        break;
      }
     Serial.println("Range and gain setting completed. .");
     break;
     case'E':
     case'e':
       /* Read the temperature. */
       temperature = AD5933_GetTemperature();
       Serial.println("");
       Serial.print("Temperature: ");
       Serial.print(temperature);
       Serial.println(" C");
       Serial.println("");
     break;
     case'F':
     case'f':
       Serial.println("Incrementing the frequency. .");
       AD5933_increment();
     break;
     case'G':
     case'g':
     Serial.println("Set the settling cycles. .");
     Serial.print("1. Enter the cycles (0-511): ");
     SettlingTime = read_number();
     Serial.println(SettlingTime);
     Serial.print("2. Enter the multiplier: (1/2/4): ");
     Multiplier = read_number();
     Serial.println(Multiplier);
     switch(Multiplier)
     {
      case 1:
       AD5933_settling_time(SettlingTime,AD5933_SETTLE_1X);
      break;
      case 2:
       AD5933_settling_time(SettlingTime,AD5933_SETTLE_2X);
      break;
      case 4:
       AD5933_settling_time(SettlingTime,AD5933_SETTLE_4X);
      break;
      default:
      Serial.println("Invalid multipler value: Setting muliplier to 1X");
      AD5933_settling_time(SettlingTime,AD5933_SETTLE_1X);
      break;
     }
     break;
     case 'H':
     case 'h':
     Serial.println("Genetaring the Impedance Spectrum. .");
     Plot_impedance_spectrum();
     break;
     case 'I':
     case 'i':
     Serial.println("Setting the AD5933 to power down. .");
     AD5933_power_down();
     break;
     case 'J':
     case 'j':
     Serial.println("Setting the AD5933 to standby mode. .");
     AD5933_standby();
     break; 
     case 'K':
     case 'k':
     Serial.println("Starting two point calibration. .");
     Serial.println(start_freq);
     Serial.println(freq_step);
     Serial.println(increment_number);
     Serial.println(calib_impedance);
     gain_delta_change = AD5933_Calibration_change(start_freq,freq_step,increment_number,calib_impedance,AD5933_FUNCTION_REPEAT_FREQ); 
     break;
#endif
   default: 
      Serial.println("Invalid Option !!");
      Serial.println("");
      Serial.println("");
      break;
 }
}

// Function that calculates the sweat glucose concentration from the change in impedance.
// Rewrite this function with a look up table or a poylnomial fit (which ever is better)
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

/*! Function that reads a number from serial port*/
unsigned long read_number()
{
      unsigned long num = 0;
      while(Serial.available() == 0);
      while(Serial.available() > 0)
       {
        num = num*10 + Serial.parseInt();
       }
      return num;
}

