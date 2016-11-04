/***************************************************************************//**
 *   @file   AD5933.cpp
 *   @brief  Implementation of AD5933 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *   @author2 Athul Asokan Thulasi (athul.asokanthulasi@utdallas.edu)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 801
*******************************************************************************/

/*
 * Modifications by Athul Asokan Thulasi:
 * 1) Removed the Communcation.c file and included Arduino based IIC Read and Write Functions
 * 2) Modified the setRegster and GetRegister functions
 * 3) Modified file type to C++
 * 4) Added set the settling cycles function and required changes
 * 5) Modified the calibrationa and impedance calculation functions to improve numerical precision.
 * 6) Added power down and standby functions
 * 7) Added two point calibration function.
 */


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include "AD5933.h"
#include <math.h>
/******************************************************************************/
/************************** Constants Definitions *****************************/
/******************************************************************************/
const long POW_2_27 = 134217728ul;      // 2 to the power of 27

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned long currentSysClk      = AD5933_INTERNAL_SYS_CLK;
unsigned char currentClockSource = AD5933_CONTROL_INT_SYSCLK;
unsigned char currentGain        = AD5933_GAIN_X1;
unsigned char currentRange       = AD5933_RANGE_2000mVpp;
double system_phase = 0.0;
extern double impedance_phase;
signed short GrealData = 0;
signed short GimagData = 0;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes.
 *
 * @return None.
*******************************************************************************/
void AD5933_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber)
{
    unsigned char byte          = 0;
    unsigned char writeData0, writeData1;

    for(byte = 0;byte < bytesNumber; byte++)
    {
        writeData0 = registerAddress + bytesNumber - byte - 1;
        writeData1 = (unsigned char)((registerValue >> (byte * 8)) & 0xFF);
        I2C_Write(AD5933_ADDRESS, writeData0, writeData1);
        #ifdef DEBUG2
        Serial.print("Writing: 0x");Serial.print(writeData1,HEX); Serial.print(" to Reg: 0x"); Serial.println(writeData0,HEX);
        #endif
    }
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes. (Max 4)
 *
 * @return registerValue - Value of the register.
*******************************************************************************/
unsigned long AD5933_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber)
{
    unsigned long registerValue = 0;
    unsigned char byte          = 0;
    unsigned char writeData0,writeData1;
    unsigned char readData[2]   = {0, 0};
    
    for(byte = 0;byte < bytesNumber;byte ++)
    {
        /* Set the register pointer. */
        writeData0 = AD5933_ADDR_POINTER;
        writeData1 = registerAddress + byte;
        I2C_Write(AD5933_ADDRESS, writeData0,writeData1);
       
        /* Read Register Data. */
        readData[0] = 0x00;
        I2C_Read(AD5933_ADDRESS, readData);
        #ifdef DEBUG2
        Serial.print("Read: 0x");Serial.print(readData[0],HEX); Serial.print(" from Reg: 0x"); Serial.println(writeData1,HEX);
        #endif
        registerValue = registerValue << 8;
        registerValue += readData[0];
    }
    
    return registerValue;
}

/***************************************************************************//**
 * @brief Writes 1 byte of data to the slave dvice
 *
 * @param slave_adrs - The address of the slave device to be written.
 * @param reg_address - The register address in the slave device that needs to be written
 * @param data - The data byte that needs to be written in the reg_address of the slave device.                     
 * Eg: I2C_Write(AD5933_ADDRESS, writeData, 2, 1);
*******************************************************************************/
void I2C_Write(unsigned char slave_adrs, unsigned char reg_adrs,unsigned char data)
{
  Wire.beginTransmission(slave_adrs);                 
  //Wire.write(slave_adrs);
  Wire.write(reg_adrs);
  Wire.write(data);
  Wire.endTransmission();      // stop transmitting
  delay(500);
}

/***************************************************************************//**
 * @brief Writes 1 byte of data to the slave device
 *
 * @param slave_adrs - The address of the slave device to be read.
 * @param dataBuffer - The pointer to the location where data byte needs to stored.  
 * Eg: I2C_Read(AD5933_ADDRESS, readData);
*******************************************************************************/
void I2C_Read(unsigned char slave_adrs,unsigned char* dataBuffer)
{                
  Wire.requestFrom(slave_adrs,uint8_t(1));
  while(Wire.available()<1);
  *dataBuffer = Wire.read(); 
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return None.
*******************************************************************************/
void AD5933_Reset(void)
{
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_LB, 
                            AD5933_CONTROL_RESET | currentClockSource,
                            1);
}

/***************************************************************************//**
 * @brief Selects the source of the system clock, takes the system out of reset
 *
 * @param clkSource - Selects the source of the system clock.
 *                    Example: AD5933_CONTROL_INT_SYSCLK
 *                             AD5933_CONTROL_EXT_SYSCLK
 * @param extClkFreq - Frequency value of the external clock, if used.
 *
 * @return None.
*******************************************************************************/
void AD5933_SetSystemClk(char clkSource, unsigned long extClkFreq)
{
    currentClockSource = clkSource;
    if(clkSource == AD5933_CONTROL_EXT_SYSCLK)
    {
        currentSysClk = extClkFreq;                 // External clock frequency
    }
    else
    {
        currentSysClk = AD5933_INTERNAL_SYS_CLK;    // 16.77 MHz
    }
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_LB, currentClockSource, 1);
}

/***************************************************************************//**
 * @brief Selects the range and gain of the device.
 *  
 * @param range - Range option.
 *                Example: AD5933_RANGE_2000mVpp
 *                         AD5933_RANGE_200mVpp
 *                         AD5933_RANGE_400mVpp
 *                         AD5933_RANGE_1000mVpp
 * @param gain - Gain option.
 *               Example: AD5933_GAIN_X5
 *                        AD5933_GAIN_X1
 *
 * @return None.
*******************************************************************************/
void AD5933_SetRangeAndGain(char range, char gain)
{
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                         AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_NOP) |
                         AD5933_CONTROL_RANGE(range) | 
                         AD5933_CONTROL_PGA_GAIN(gain),
                         1);
    /* Store the last settings made to range and gain. */
    currentRange = range;
    currentGain = gain;
}

/***************************************************************************//**
 * @brief Reads the temperature from the part and returns the data in
 *        degrees Celsius.
 *
 * @return temperature - Temperature.
*******************************************************************************/
char AD5933_GetTemperature(void)
{
    short         temperature = 0;
    unsigned char status      = 0;
    
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                         AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_MEASURE_TEMP) |
                         AD5933_CONTROL_RANGE(currentRange) | 
                         AD5933_CONTROL_PGA_GAIN(currentGain),                             
                         1);
    while((status & AD5933_STAT_TEMP_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    }
    
    temperature = AD5933_GetRegisterValue(AD5933_REG_TEMP_DATA,2);
    if(temperature < 8192)
    {
        temperature /= 32;
    }
    else
    {
        temperature -= 16384;
        temperature /= 32;
    }
    
    return (char)temperature;
}

/***************************************************************************//**
 * @brief Configures the sweep parameters: Start frequency, Frequency increment
 *        and Number of increments.
 *
 * @param startFreq - Start frequency in Hz;
 * @param incFreq - Frequency increment in Hz;
 * @param incNum - Number of increments. Maximum value is 511(0x1FF).
 *
 * @return None.
*******************************************************************************/
void AD5933_ConfigSweep(unsigned long  startFreq,
                        unsigned long  incFreq,
                        unsigned short incNum)
{
    unsigned long  startFreqReg = 0;
    unsigned long  incFreqReg   = 0;
    unsigned short incNumReg    = 0;
    
    /* Ensure that incNum is a valid data. */
    if(incNum > AD5933_MAX_INC_NUM)
    {
        incNumReg = AD5933_MAX_INC_NUM;
    }
    else
    {
        incNumReg = incNum;
    }
    
    /* Convert users start frequency to binary code. */
    startFreqReg = (unsigned long)((double)startFreq * 4 / currentSysClk *POW_2_27);
   
    /* Convert users increment frequency to binary code. */
    incFreqReg = (unsigned long)((double)incFreq * 4 / currentSysClk * POW_2_27);
    
    /* Configure the device with the sweep parameters. */
    AD5933_SetRegisterValue(AD5933_REG_FREQ_START,
                            startFreqReg,
                            3);
    AD5933_SetRegisterValue(AD5933_REG_FREQ_INC,
                            incFreqReg,
                            3);
    AD5933_SetRegisterValue(AD5933_REG_INC_NUM,
                            incNumReg,
                            2);
}

/***************************************************************************//**
 * @brief Starts the sweep operation.
 *
 * @return None.
*******************************************************************************/
void AD5933_StartSweep(void)
{
    unsigned char status = 0;
    
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_STANDBY) |
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
                            1);
    AD5933_Reset();
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                       AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_INIT_START_FREQ)|
                       AD5933_CONTROL_RANGE(currentRange) | 
                       AD5933_CONTROL_PGA_GAIN(currentGain),
                       1);
                       
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                       AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_START_SWEEP) | 
                       AD5933_CONTROL_RANGE(currentRange) | 
                       AD5933_CONTROL_PGA_GAIN(currentGain),
                       1);
    status = 0;
    while((status & AD5933_STAT_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    };
}

/***************************************************************************//**
 * @brief Incremants the frequency as set by the config sweep function
 *
 * @return None.
*******************************************************************************/
void AD5933_increment(void)
{
   AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                       AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_INC_FREQ) | 
                       AD5933_CONTROL_RANGE(currentRange) | 
                       AD5933_CONTROL_PGA_GAIN(currentGain),
                       1);
}
/***************************************************************************//**
 * @brief Reads the real and the imaginary data and calculates the Gain Factor.
 *
 * @param calibrationImpedance - The calibration impedance value.
 * @param freqFunction - Frequency function.
 *                       Example: AD5933_FUNCTION_INC_FREQ - Increment freq.;
 *                                AD5933_FUNCTION_REPEAT_FREQ - Repeat freq..
 *
 * @return gainFactor - Calculated gain factor.
*******************************************************************************/
double AD5933_CalculateGainFactor(unsigned long calibrationImpedance,
                                  unsigned char freqFunction)
{
    double        gainFactor = 0;
    double        magnitude  = 0;
    signed short  realData   = 0;
    signed short  imagData   = 0;
    unsigned long int R2 = 0L;
    unsigned long int I2 = 0L;
    unsigned char status = 0;
    double phase_angle_rad = 0.0;
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(freqFunction) |
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),    
                            1);
    status = 0;
    while((status & AD5933_STAT_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    }
    realData = AD5933_GetRegisterValue(AD5933_REG_REAL_DATA,2);
    imagData = AD5933_GetRegisterValue(AD5933_REG_IMAG_DATA,2);
    R2 = pow(realData,2);
    I2 = pow(imagData,2);
    magnitude = R2 + I2;
    #ifdef DEBUG3
     Serial.println("Calculating gain factor magnitude");
     Serial.print("R: ");Serial.print(realData);Serial.print(" R(HEX): 0x");Serial.println(realData,HEX);
     Serial.print("I: ");Serial.print(imagData);Serial.print(" I(HEX): 0x");Serial.println(imagData,HEX);
     Serial.print("R^2: ");Serial.println(R2);
     Serial.print("I^2: ");Serial.println(I2);
     Serial.print("R^2 + I^2: ");Serial.println(magnitude);
    #endif
     magnitude = sqrt(magnitude);
     gainFactor = (1.0 / (magnitude * calibrationImpedance*1.0))*1000000000;
    #ifdef DEBUG3
    Serial.print("sqrt(R^2 + I^2): ");Serial.println(magnitude);
    Serial.print("Gain Factor*10^9: ");Serial.println(gainFactor);
    #endif
   //Phase measurement method 1 - in standard degrees 
    system_phase = phase_in_degrees((double)realData,(double)imagData);
    phase_angle_rad = phase_in_radians((double)realData,(double)imagData);
    #ifdef DEBUG3 
     Serial.println("");
     Serial.println("Calculating the system phase");
     Serial.print("System Phase (degrees): ");Serial.println(system_phase);
     Serial.print("System Phase (radians): ");Serial.println(phase_angle_rad);
    #endif
    return gainFactor;
}

/***************************************************************************//**
 * @brief Reads the real and the imaginary data and calculates the Impedance.
 *
 * @param gainFactor - The gain factor.
 * @param freqFunction - Frequency function.
 *                       Example: AD5933_FUNCTION_INC_FREQ - Increment freq.;
 *                                AD5933_FUNCTION_REPEAT_FREQ - Repeat freq..
 *
 * @return impedance - Calculated impedance.
*******************************************************************************/
double AD5933_CalculateImpedance(double gainFactor,
                                 unsigned char freqFunction)
{
    signed short  realData  = 0;
    signed short  imagData  = 0;
    double        magnitude = 0;
    double        impedance = 0;
    unsigned long int R2 = 0L;
    unsigned long int I2 = 0L;
    unsigned char status    = 0;
    //Phase calculation values
    double real_comp = 0.0;
    double delta_real_comp =0.0;
    double imag_comp = 0.0;
    double delta_imag_comp =0.0;
    double angle_ratio = 0.0;
    double delta_angle_ratio =0.0;
    double phase_angle_rad =0.0;
    double impedance_sys_phase =0.0;
    
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(freqFunction) | 
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
                            1);
    status = 0;
    while((status & AD5933_STAT_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    }
    realData = AD5933_GetRegisterValue(AD5933_REG_REAL_DATA,2);
    imagData = AD5933_GetRegisterValue(AD5933_REG_IMAG_DATA,2);
    GrealData = realData;
    GimagData = imagData;
    R2 = pow(realData,2);
    I2 =pow(imagData,2);
    magnitude = R2 + I2;
    #ifdef DEBUG4
    Serial.println(" ");
    Serial.println(" ");
    Serial.println("Calculating the Impedance magnitude");
    Serial.print("R: ");Serial.print(realData);Serial.print(" R(HEX): 0x");Serial.println(realData,HEX);
    Serial.print("I: ");Serial.print(imagData);Serial.print(" I(HEX): 0x");Serial.println(imagData,HEX);
    Serial.print("R^2 :");Serial.println(R2);
    Serial.print("I^2 :");Serial.println(I2);
    Serial.print("R^2 + I^2: ");Serial.println(magnitude);
    #endif
    magnitude = sqrt(magnitude);
    impedance = 1000000000.0 *(1.0 / (magnitude * gainFactor*1.0L)); 
    #ifdef DEBUG4
    Serial.print("sqrt(R^2 + I^2): ");Serial.println(magnitude);
    Serial.print("Impedance: ");Serial.println(impedance);
    #endif

 // Calculating the phase
    // Method 1 : Standard phase in degrees - standard phase in degrees of the system
    impedance_phase = phase_in_degrees((double)realData,(double)imagData);
    #ifdef DEBUG4
    Serial.println(" ");
    Serial.println("Calculating the phase");
    Serial.print("System Phase(degrees): ");Serial.println(impedance_phase);
    #endif
    impedance_phase = impedance_phase - system_phase;
    #ifdef DEBUG4
    Serial.print("Phase due to added impedance(degrees): ");Serial.println(impedance_phase);
    Serial.println(" ");
    #endif
    return impedance;    
}

/***************************************************************************//**
 * @brief: Two point calibration function that generates the calibration gain factor at the start and end of the frequency sweep 
 *  It can be used to approximate the gain factor at any intermediate frequency by linear interpolation.
 *  Returns the change in gain factor for the frequency increment step configured by the configure frequency seep function. 
 *  Eg: double AD5933_Calibration_change(unsigned long start_frequency,unsigned long frequency_step_size, unsigned short frequency_step_count, unsigned long calibrationImpedance, unsigned char freqFunction);
 */
double AD5933_Calibration_change(unsigned long start_frequency,unsigned long frequency_step_size, unsigned short frequency_step_count, unsigned long calibrationImpedance, unsigned char freqFunction)
{
  double gain_change = 0.0;
  double init_gain = 0.0;
  double final_gain =0.0;
  unsigned long stop_frequency = start_frequency + (frequency_step_count*frequency_step_size);
  AD5933_ConfigSweep(start_frequency,frequency_step_size,0);
  AD5933_StartSweep();
  init_gain = AD5933_CalculateGainFactor(calibrationImpedance,freqFunction);
  #ifdef DEBUG
  Serial.print("Initial Gain calculated: ");
  Serial.println(init_gain);
  #endif
  AD5933_ConfigSweep(stop_frequency,frequency_step_size,0);
  AD5933_StartSweep();
  final_gain = AD5933_CalculateGainFactor(calibrationImpedance,freqFunction);
  #ifdef DEBUG
  Serial.print("Final Gain calculated: ");
  Serial.println(final_gain);
  #endif
  AD5933_ConfigSweep(start_frequency,frequency_step_size,frequency_step_count);
  AD5933_StartSweep();
  gain_change = (final_gain - init_gain)/(frequency_step_count*1.0);
  #ifdef DEBUG
  Serial.print("Gain chage per frequency step: ");
  Serial.println(gain_change);
  Serial.println("");
  #endif
  return gain_change;
}

/***************************************************************************//**
 * @brief Sets the settling cycles before the ADC conversion starts
 *
 * @param settlingTime - number between 1 and 511
 * @param multiplier -  AD5933_SETTLE_1X
 *                      AD5933_SETTLE_2X
 *                      AD5933_SETTLE_4X

*******************************************************************************/
void AD5933_settling_time(unsigned long settlingTime, unsigned char multiplier)
{
  byte bit_9 = (unsigned char)((settlingTime >> 8) & 0x01);
  byte settling_count = (unsigned char)(settlingTime & 0xFF);
  
  AD5933_SetRegisterValue(AD5933_REG_SETTLING_MULTIPLIER,
                            AD5933_SETTLING_TIME(multiplier)|(bit_9),
                            1);
  AD5933_SetRegisterValue(AD5933_REG_SETTLING_CYCLES,
                            settling_count,
                            1);
  #ifdef DEBUG2
  Serial.print("BIT {11:10:9}: ");Serial.println(AD5933_SETTLING_TIME(multiplier));
  Serial.print("BIT 9: ");Serial.println(bit_9); 
  Serial.print("Lower Byte: ");Serial.println(settling_count);
  #endif
}


/***************************************************************************//**
 * @brief Sets the AD5933 in powerdown mode
 * Eg; AD5933_power_down();
 * In power down mode VIN and VOUT pins are internally connected to ground and the device in low power mode
 * 
*******************************************************************************/
void AD5933_power_down()
{
  AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_POWER_DOWN) | 
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
                            1);
}

/***************************************************************************//**
 * @brief Sets the AD5933 in standby mode
 * Eg; AD5933_standby();
 * In standby mode VIN and VOUT pins are internally connected to ground but the rest of the device is functional
 * 
*******************************************************************************/
void AD5933_standby()
{
  AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_STANDBY) | 
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
                            1);
}

/***************************************************************************//**
 * @brief Calculate the phase in degrees / radians
 * Eg; double degrees = phase_in_degrees(signed short Real_part, signed short Imaginary_part)

*******************************************************************************/
double phase_in_degrees(double R, double I)
{
  double phase_degree =0.0;
  double ratio = (I*1.0)/(R*1.0);
  if((R >=0)&&(I>=0))  // I st Quadrant
  {
    phase_degree = (atan(ratio)*180)/PI;
  }
  else if ((R >=0)&&(I<=0)) // IV rd Quadrant
  {
    phase_degree = 360+ ((atan(ratio)*180)/PI);
  }
  else  // IInd or IIIrd Quadrant
  {
     phase_degree = 180+((atan(ratio)*180)/PI);
  }
  return phase_degree;
}

double phase_in_radians(double R, double I)
{
  double phase_rad =0.0;
  double ratio = (I*1.0)/(R*1.0);
  if((R >=0)&&(I>=0))  // I st Quadrant
  {
    phase_rad = atan(ratio);
  }
  else if ((R >=0)&&(I<=0)) // IV rd Quadrant
  {
    phase_rad = ((2.0*PI) + atan(ratio));
  }
  else  // IInd or IIIrd Quadrant
  {
     phase_rad = PI+(atan(ratio));
  }
  return phase_rad;
}
/*************************************************************************************/



