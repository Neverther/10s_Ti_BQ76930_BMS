/*
NOTE: This file has been modified from the original for debugging purposes - it was hanging up upon the begin function call - hardware watchdog timer tripped on ESP82885 and caused restart


    bq769x0.cpp - Battery management system based on bq769x0 for Arduino
    Copyright (C) 2015  Martin Jäger (m.jaeger@posteo.de)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>.
*/

/*
  TODO:
  - SOC calculation + coulomb counting

*/

#include <Arduino.h>
#include <Wire.h>     // I2C/TWI (for Battery Management IC)
#include <math.h>     // log for thermistor calculation

#include "bq769x0.h"
#include "registers.h"

// for the ISR to know the bq769x0 instance
bq769x0* bq769x0::instancePointer = 0;


//#if BQ769X0_DEBUG //removed since not in the mbed library

const char *byte2char(int x)
{
  static char b[9];
  b[0] = '\0';

  int z;
  for (z = 128; z > 0; z >>= 1)
  {
    strcat(b, ((x & z) == z) ? "1" : "0");
  }

  return b;
}

// CRC bit taken from mbed library , crc function called from othr
uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
  uint8_t   i;
  uint8_t   data;

  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}




//#endif      //not in mbed library

//----------------------------------------------------------------------------

bq769x0::bq769x0(byte bqType, byte cellcount)
{
  type = bqType;
  numberOfCells = cellcount;
  
  // prevent errors if someone reduced MAX_NUMBER_OF_CELLS accidentally
  if (numberOfCells > MAX_NUMBER_OF_CELLS) {
    numberOfCells = MAX_NUMBER_OF_CELLS;
  }
}


//-----------------------------------------------------------------------------

int bq769x0::begin(byte alertPin, byte cellcount, uint8_t reg_RSNS)
{
  balancingCells = 0xff;
  overChargeCurrent = false;
  Wire.begin();        // join I2C bus
  Wire.setClock(100000UL);
  delay(100);
  // initialize variables
  for (byte i = 0; i < cellcount; i++) {
    cellVoltages[i] = 0;
  }
  if (reg_RSNS < 2)
    RSNS = reg_RSNS;
    
  
  if (determineAddressAndCrc())
  {
    //debug prints to show that initial comms were successful
    //~ Serial.println("Address and CRC detection successful");
    //~ Serial.print("Address:  ");
    //~ Serial.println(I2CAddress, HEX);
    //~ Serial.print("CRC Enabled:  ");
    //~ Serial.println(crcEnabled);

    // attach ALERT interrupt to this instance
    instancePointer = this;  //taken from arduino compatible code
    attachInterrupt(digitalPinToInterrupt(alertPin), bq769x0::alertISR, RISING); //taken from arduino compatible code

    // get ADC offset and gain
    adcOffset = (signed int) readRegister(ADCOFFSET);  // convert from 2's complement
    adcGain = 365 + (((readRegister(ADCGAIN1) & 0b00001100) << 1) |
                    ((readRegister(ADCGAIN2) & 0b11100000) >> 5)); // uV/LSB
                    

    return 0;
  }
  connectionErrorFlag = true;
  return 1;
}

int bq769x0::initialize(byte bootPin)
{
  // Boot IC if pin is defined (else: manual boot via push button has to be
  // done before calling this method)
  if (bootPin < 127)
  {
    pinMode(bootPin, OUTPUT);
    digitalWrite(bootPin, HIGH);
    delay(5);   // wait 5 ms for device to receive boot signal (datasheet: max. 2 ms)
    pinMode(bootPin, INPUT);     // don't disturb temperature measurement
    delay(10);  // wait for device to boot up completely (datasheet: max. 10 ms)

    // 250ms for 20, 400ms for 30 and 800ms for 40
    delay(250);
    for (byte i = 0; i < type ; i++ )
      delay(200);
  }

  // initial settings for bq769x0
  byte sys_ctrl1;
  sys_ctrl1 = readRegister(SYS_CTRL1);
  writeRegister(SYS_CTRL1, sys_ctrl1 | 0b00010000);  // Turn ADC on

  byte sys_ctrl2;
  sys_ctrl2 = readRegister(SYS_CTRL2);
  writeRegister(SYS_CTRL2, sys_ctrl2 | 0b01000000);  // switch CC_EN on

  

  return 0;
}


/*  // From original arduino startup code
  if (readRegister(CC_CFG) == 0x19)
  {
	  Serial.println("Inside readregister for cc_cfg if");
    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, B00011000);  // switch external thermistor and ADC on
    writeRegister(SYS_CTRL2, B01000000);  // switch CC_EN on

    // attach ALERT interrupt to this instance
    instancePointer = this;
    attachInterrupt(digitalPinToInterrupt(alertPin), bq769x0::alertISR, RISING);

    // get ADC offset and gain
    adcOffset = (signed int) readRegister(ADCOFFSET);  // convert from 2's complement
    adcGain = 365 + (((readRegister(ADCGAIN1) & B00001100) << 1) |
      ((readRegister(ADCGAIN2) & B11100000) >> 5)); // uV/LSB

    return 0;
  }
  else
  {
	  Serial.println("outisde readregister cc_CFG - the else statement");
    #if BQ769X0_DEBUG
    Serial.println("BMS communication error");
    #endif
    return 1;
  }
  **/



//----------------------------------------------------------------------------
// automatically find out address and CRC setting

bool bq769x0::determineAddressAndCrc(void)
{
  I2CAddress = 0x08;
  crcEnabled = true;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) {
    return true;
  }

  I2CAddress = 0x18;
  crcEnabled = true;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) {
    return true;
  }

  I2CAddress = 0x08;
  crcEnabled = false;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) {
    return true;
  }

  I2CAddress = 0x18;
  crcEnabled = false;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) {
    return true;
  }

  return false;
}




//----------------------------------------------------------------------------
// Fast function to check whether BMS has an error
// (returns 0 if everything is OK)

int bq769x0::checkStatus()
{
  //  Serial.print("errorStatus: ");
  //  Serial.println(errorStatus);
  if (readRegister(CC_CFG) != 0x19) {
    connectionErrorFlag = true;
  }
  
  if (alertInterruptFlag == false && errorStatus == 0) {
    return 0;
  }
  else {

    regSYS_STAT_t sys_stat;
    sys_stat.regByte = readRegister(SYS_STAT);

    if (sys_stat.bits.CC_READY == 1) {
      //Serial.println("Interrupt: CC ready");
      updateCurrent(true);  // automatically clears CC ready flag
    }

    // Serious error occured
    if (sys_stat.regByte & B00111111)
    {
      if (alertInterruptFlag == true) {
        secSinceErrorCounter = 0;
      }
      errorStatus = sys_stat.regByte;

      int secSinceInterrupt = (millis() - interruptTimestamp) / 1000;

      // check for overrun of millis() or very slow running program
      if (abs(secSinceInterrupt - secSinceErrorCounter) > 2) {
        secSinceErrorCounter = secSinceInterrupt;
      }

      // called only once per second
      if (secSinceInterrupt >= secSinceErrorCounter)
      {
        if (sys_stat.regByte & B00100000) { // XR error
          // datasheet recommendation: try to clear after waiting a few seconds
          if (secSinceErrorCounter % 3 == 0) {
            internalFaultFlag = true;
            writeRegister(SYS_STAT, B00100000);
          }
        }
        if (sys_stat.regByte & B00010000) { // Alert error
          if (secSinceErrorCounter % 10 == 0) {
            externalPullupFlag = true;
            writeRegister(SYS_STAT, B00010000);
          }
        }
        if (sys_stat.regByte & B00001000) { // UV error
          updateVoltages();
          underVoltageFlag = true;
          if (cellVoltages[idCellMinVoltage] > minCellVoltage) {
            writeRegister(SYS_STAT, B00001000);
          }
        }
        if (sys_stat.regByte & B00000100) { // OV error
          updateVoltages();
          overVoltageFlag = true;
          if (cellVoltages[idCellMaxVoltage] < maxCellVoltage) {
            writeRegister(SYS_STAT, B00000100);
          }
        }
        if (sys_stat.regByte & B00000010) { // SCD
          shortCircuitDischargingFlag = true;
          if (secSinceErrorCounter % 60 == 0) {
            writeRegister(SYS_STAT, B00000010);
          }
        }
        if (sys_stat.regByte & B00000001) { // OCD
          overCurrentDischargingFlag = true;
          if (secSinceErrorCounter % 60 == 0) {
            writeRegister(SYS_STAT, B00000001);
          }
        }

        secSinceErrorCounter++;
      }
    }
    else {
      errorStatus = 0;
    }

    return errorStatus;

  }

}

//----------------------------------------------------------------------------
// should be called at least once every 250 ms to get correct coulomb counting

void bq769x0::update()
{
  updateCurrent();  // will only read new current value if alert was triggered
  updateVoltages();
  updateTemperatures();
  updateBalancingSwitches();
}

//----------------------------------------------------------------------------
// puts BMS IC into SHIP mode (i.e. switched off)

void bq769x0::shutdown()
{
  writeRegister(SYS_CTRL1, 0x0);
  writeRegister(SYS_CTRL1, 0x1);
  writeRegister(SYS_CTRL1, 0x2);
}

//----------------------------------------------------------------------------

bool bq769x0::enableCharging()
{
  long chargeTime = (millis() - chargeTimestamp);
  // check for millis() overflow
  if (chargeTime < 0) {
    chargeTime = 0;
  }
  
  if (checkStatus() == 0 &&
      cellVoltages[idCellMaxVoltage] < maxCellVoltage &&
      temperatures[0] < maxCellTempCharge &&
      temperatures[0] > minCellTempCharge &&
      chargeTime > 30000)
  {
    byte sys_ctrl2;
    sys_ctrl2 = readRegister(SYS_CTRL2);
    writeRegister(SYS_CTRL2, sys_ctrl2 | B00000001);  // switch CHG on

    return true;
  }
  else {
    return false;
  }
}

void bq769x0::disableCharging()
{
  byte sys_ctrl2;
  sys_ctrl2 = readRegister(SYS_CTRL2);
  writeRegister(SYS_CTRL2, sys_ctrl2 & ~B00000001);  // switch CHG off

}

//----------------------------------------------------------------------------

bool bq769x0::enableDischarging()
{
  if (checkStatus() == 0 &&
      cellVoltages[idCellMinVoltage] > minCellVoltage &&
      temperatures[0] < maxCellTempDischarge &&
      temperatures[0] > minCellTempDischarge)
  {
    byte sys_ctrl2;
    sys_ctrl2 = readRegister(SYS_CTRL2);
    writeRegister(SYS_CTRL2, sys_ctrl2 | B00000010);  // switch DSG on
    return true;
  }
  else {
    return false;
  }
}

void bq769x0::disableDischarging()
{
  byte sys_ctrl2;
  sys_ctrl2 = readRegister(SYS_CTRL2);
  writeRegister(SYS_CTRL2, sys_ctrl2 & ~B00000010);  // switch DSG off
}

//----------------------------------------------------------------------------

void bq769x0::enableAutoBalancing(bool enable)
{
  autoBalancingEnabled = enable;
}


//----------------------------------------------------------------------------

void bq769x0::setBalancingThresholds(int idleTime_min, int absVoltage_mV, byte voltageDifference_mV)
{
  balancingMinIdleTime_s = idleTime_min * 60;
  balancingMinCellVoltage_mV = absVoltage_mV;
  balancingMaxVoltageDifference_mV = voltageDifference_mV;
}

// Are we balancing?
bool bq769x0::isBalancing(void)
{
  return balancingActive;
}

//----------------------------------------------------------------------------
// sets balancing registers if balancing is allowed
// (sufficient idle time + voltage)

void bq769x0::updateBalancingSwitches(void)
{
  long idleSeconds = (millis() - idleTimestamp) / 1000;
  byte numberOfSections = type;
  // check for millis() overflow
  if (idleSeconds < 0) {
    idleTimestamp = 0;
    idleSeconds = millis() / 1000;
  }

  // check if balancing allowed
  if (autoBalancingEnabled && checkStatus() == 0 &&
      idleSeconds >= balancingMinIdleTime_s &&
      cellVoltages[idCellMaxVoltage] > balancingMinCellVoltage_mV &&
      (cellVoltages[idCellMaxVoltage] - cellVoltages[idCellMinVoltage]) > balancingMaxVoltageDifference_mV)
  {
    balancingActive = true;
    regCELLBAL_t cellbal;
    byte balancingFlags;
    byte balancingFlagsTarget;
    uint16_t balancingCells_tmp;
    uint8_t cnt = 0;
    for (int section = 0; section < numberOfSections; section++)
    {
      balancingFlags = 0;
      for (int i = 0; i < 5; i++)
      {
        if (cellVoltages[section*5 + i] > 500)
          cnt++;
        if ((cellVoltages[section*5 + i] - cellVoltages[idCellMinVoltage]) > balancingMaxVoltageDifference_mV) {

          // try to enable balancing of current cell
          balancingFlagsTarget = balancingFlags | (1 << i);

          // check if attempting to balance previous cell
          if (i>0 && !(balancingFlags && (1 << i-1)))
            continue;

          // Handle non-connected cells, last cell and two first cells in group are always connected
          // Only the last cell may have shorted cells before it
          if (i==4 && cellVoltages[section*5 + 3] < 500 && !(balancingFlags && (1 << 2)))
            continue;

          // Max two shorted outputs
          if (i==4 && cellVoltages[section*5 + 3] < 500 && cellVoltages[section*5 + 2] < 500 && !(balancingFlags && (1 << 1)))
            continue;

          // Valid with no prior conflicts
          balancingFlags = balancingFlagsTarget;
          balancingCells_tmp |= (1 << (cnt-1));
        }
      }
      //~ Serial.print("Setting CELLBAL");
      //~ Serial.print(section+1);
      //~ Serial.print(" register to: ");
      //~ Serial.println(byte2char(balancingFlags));

      // set balancing register for this section
      writeRegister(CELLBAL1+section, balancingFlags);
      balancingCells = balancingCells_tmp;
    }
  }
  else if (balancingActive == true)
  {
    // clear all CELLBAL registers
    for (int section = 0; section < numberOfSections; section++)
    {
      //~ Serial.print("Clearing Register CELLBAL");
      //~ Serial.println(section+1);
      writeRegister(CELLBAL1+section, 0x00);
    }
    balancingCells = 0;
    balancingActive = false;
  }
}

void bq769x0::setShuntResistorValue(float res_mOhm)
{
  shuntResistorValue_mOhm = res_mOhm;
}

void bq769x0::setThermistorBetaValue(int beta_K)
{
  thermistorBetaValue = beta_K;
}

void bq769x0::setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC,
                                   int minCharge_degC, int maxCharge_degC)
{
  // Temperature limits (°C/10)
  minCellTempDischarge = minDischarge_degC * 10;
  maxCellTempDischarge = maxDischarge_degC * 10;
  minCellTempCharge = minCharge_degC * 10;
  maxCellTempCharge = maxCharge_degC * 10;
}

void bq769x0::setIdleCurrentThreshold(int current_mA)
{
  idleCurrentThreshold = current_mA;
}


//----------------------------------------------------------------------------

float bq769x0::setShortCircuitProtection(float current_A, int delay_us)
{
  regPROTECT1_t protect1;
  protect1.bits.RSNS = RSNS;

  protect1.bits.SCD_THRESH = 0;
  for (int i = 0; i < sizeof(SCD_threshold_setting); i++) {
    if ((current_A * shuntResistorValue_mOhm) < ((float)(SCD_threshold_setting[i] * (RSNS+1))))
    {
      protect1.bits.SCD_THRESH = i;
      break;
    }
  }

  protect1.bits.SCD_DELAY = 0;
  for (int i = sizeof(SCD_delay_setting)-1; i > 0; i--) {
    if (delay_us >= SCD_delay_setting[i]) {
      protect1.bits.SCD_DELAY = i;
      break;
    }
  }

  writeRegister(PROTECT1, protect1.regByte);

  // returns the actual current threshold value
  return (float)(SCD_threshold_setting[protect1.bits.SCD_THRESH]*(RSNS+1)) /
         shuntResistorValue_mOhm;
}

//----------------------------------------------------------------------------

void bq769x0::setOvercurrentChargeProtection(float current_A, int delay_ms)
{
  maxChargeCurrent = current_A;
  maxChargeCurrentDelay = delay_ms;
}

//----------------------------------------------------------------------------

float bq769x0::setOvercurrentDischargeProtection(float current_A, int delay_ms)
{
  regPROTECT2_t protect2;

  protect2.bits.OCD_THRESH = 0;
  for (int i = 0; i < sizeof(OCD_threshold_setting); i++) {
    if (current_A * shuntResistorValue_mOhm < ((float)OCD_threshold_setting[i]*(RSNS+1))) {
      protect2.bits.OCD_THRESH = i;
      break;
    }
  }

  protect2.bits.OCD_DELAY = 0;
  for (int i = sizeof(OCD_delay_setting)-1; i > 0; i--) {
    if (delay_ms >= OCD_delay_setting[i]) {
      protect2.bits.OCD_DELAY = i;
      break;
    }
  }

  writeRegister(PROTECT2, protect2.regByte);

  // returns the actual current threshold value
  return (float)(OCD_threshold_setting[protect2.bits.OCD_THRESH]*(RSNS+1)) /
         shuntResistorValue_mOhm;
}


//----------------------------------------------------------------------------

int bq769x0::setCellUndervoltageProtection(int voltage_mV, int delay_s)
{
  regPROTECT3_t protect3;
  byte uv_trip = 0;

  minCellVoltage = voltage_mV;

  protect3.regByte = readRegister(PROTECT3);

  uv_trip = ((((long)voltage_mV - adcOffset) * 1000 / adcGain) >> 4) & 0x00FF;
  uv_trip += 1;   // always round up for lower cell voltage
  writeRegister(UV_TRIP, uv_trip);

  protect3.bits.UV_DELAY = 0;
  for (int i = sizeof(UV_delay_setting)-1; i > 0; i--) {
    if (delay_s >= UV_delay_setting[i]) {
      protect3.bits.UV_DELAY = i;
      break;
    }
  }

  writeRegister(PROTECT3, protect3.regByte);

  // returns the actual current threshold value
  return ((long)1 << 12 | uv_trip << 4) * adcGain / 1000 + adcOffset;
}

//----------------------------------------------------------------------------

int bq769x0::setCellOvervoltageProtection(int voltage_mV, int delay_s)
{
  regPROTECT3_t protect3;
  byte ov_trip = 0;

  maxCellVoltage = voltage_mV;

  protect3.regByte = readRegister(PROTECT3);

  ov_trip = ((((long)voltage_mV - adcOffset) * 1000 / adcGain) >> 4) & 0x00FF;
  writeRegister(OV_TRIP, ov_trip);

  protect3.bits.OV_DELAY = 0;
  for (int i = sizeof(OV_delay_setting)-1; i > 0; i--) {
    if (delay_s >= OV_delay_setting[i]) {
      protect3.bits.OV_DELAY = i;
      break;
    }
  }

  writeRegister(PROTECT3, protect3.regByte);

  // returns the actual current threshold value
  return ((long)1 << 13 | ov_trip << 4) * adcGain / 1000 + adcOffset;
}


uint8_t bq769x0::getCellCount()
{
  return detectedCells;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getBalancingStatus()
{
  return balancingCells;
}

//----------------------------------------------------------------------------

float bq769x0::getBatteryCurrent()
{
  return batCurrent;
}

//----------------------------------------------------------------------------

float bq769x0::getBatteryVoltage()
{
  return batVoltage/1000.0;
}

//----------------------------------------------------------------------------

int bq769x0::getMinCellVoltage()
{
  return cellVoltages[idCellMinVoltage];
}

//----------------------------------------------------------------------------

int bq769x0::getMaxCellVoltage()
{
  return cellVoltages[idCellMaxVoltage];
}

//----------------------------------------------------------------------------

int bq769x0::getCellVoltage(byte idCell)
{
  return cellVoltages[idCell];
}

// Return natural order of connected cells
int bq769x0::getCellVoltageFiltered(byte idCell)
{
  return cellVoltagesF[idCell];
}

//----------------------------------------------------------------------------

fault_t bq769x0::getFaults(void)
{
  fault_t bmsfault = none;

  if (shortCircuitDischargingFlag)
  {
    bmsfault = shortcircuitFault;
    shortCircuitDischargingFlag = false;
  }
  if (overCurrentDischargingFlag)
  {
    bmsfault = overcurrentDischargeFault;
    overCurrentDischargingFlag = false;
  }
  if (overVoltageFlag)
  {
    bmsfault = overvoltageFault;
    overVoltageFlag = false;
  }
  if (underVoltageFlag)
  {
    bmsfault = undervoltgeFault;
    underVoltageFlag = false;
  }
  if (overCurrentChargingFlag)
  {
    bmsfault = overcurrentChargeFault;
    overCurrentChargingFlag = false;
  }
  if (externalPullupFlag)
  {
    bmsfault = externalPullupFault;
    externalPullupFlag = false;
  }
  if (internalFaultFlag)
  {
    bmsfault = internalFault;
    internalFaultFlag = false;
  }
  if (connectionErrorFlag)
  {
    bmsfault = connectionErrorFault;
    connectionErrorFlag = false;
  }

  return bmsfault;
}

//----------------------------------------------------------------------------

// Only return package specific channels
float bq769x0::getTemperatureDegC(byte channel)
{
  if (channel >= 0 && channel < type) {
    return (float)temperatures[channel] / 10.0;
  }
  else
    return -273.15;   // Error: Return absolute minimum temperature
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegF(byte channel)
{
  return getTemperatureDegC(channel) * 1.8 + 32;
}

bool bq769x0::getChgFet()
{
  return (readRegister(SYS_CTRL2) & 0x01);
}

bool bq769x0::getDsgFet()
{
  return (readRegister(SYS_CTRL2) & 0x03);
}


//----------------------------------------------------------------------------

void bq769x0::updateTemperatures()
{
  float tmp = 0;
  int adcVal = 0;
  int vtsx = 0;
  unsigned long rts = 0;

  Wire.beginTransmission(I2CAddress);
  Wire.write(0x2C);
  Wire.endTransmission();

  if (Wire.requestFrom(I2CAddress, 2) == 2)
  {
    // calculate R_thermistor according to bq769x0 datasheet
    adcVal = ((Wire.read() & B00111111) << 8) | Wire.read();
    vtsx = adcVal * 0.382; // mV
    rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm

    // Temperature calculation using Beta equation
    // - According to bq769x0 datasheet, only 10k thermistors should be used
    // - 25°C reference temperature for Beta equation assumed
    tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue*log(rts/10000.0)); // K

    temperatures[0] = (tmp - 273.15) * 10.0;
  }
}


//----------------------------------------------------------------------------
// If ignoreCCReadFlag == true, the current is read independent of an interrupt
// indicating the availability of a new CC reading

void bq769x0::updateCurrent(bool ignoreCCReadyFlag)
{
  int adcVal = 0;
  regSYS_STAT_t sys_stat;
  sys_stat.regByte = readRegister(SYS_STAT);

  if (ignoreCCReadyFlag == true || sys_stat.bits.CC_READY == 1)
  {
    adcVal = (readRegister(CC_HI_BYTE) << 8) | readRegister(CC_LO_BYTE);
    batCurrent = adcVal * 8.44 / shuntResistorValue_mOhm;  // uV/mOhm -> mA
    batCurrent = batCurrent / 1000.0; // A

    // reset idleTimestamp, only balance when charging with sufficent current, not when full to prevent balancing staying on after disconnection
    if (batCurrent < -idleCurrentThreshold) {
      idleTimestamp = millis();
    }

    if (batCurrent < -maxChargeCurrent)
    {
      if (!overChargeCurrent)
      {
        overChargeCurrent = true;
        chargeTimestamp = millis();
      }
    }
    else
    {
      overChargeCurrent = false;
    }

    long chargeTime = (millis() - chargeTimestamp);
    // check for millis() overflow
    if (chargeTime < 0) {
      chargeTimestamp = 0;
      chargeTime = millis();
    }
    if (overChargeCurrent && chargeTime > maxChargeCurrentDelay) // No internal protections, do on software
    {
      disableCharging();
      overCurrentChargingFlag = true;
    }

    byte writeback = B10000000;
    // no error occured which caused alert
    if (!(sys_stat.regByte & B00111111)) {
      alertInterruptFlag = false;
      writeRegister(SYS_STAT, writeback);  // Clear CC ready flag
      return;
    }
    if (!(sys_stat.regByte & B00100000)) {
      internalFaultFlag = true;
      writeback |= B00100000;
    }
    if (!(sys_stat.regByte & B00010000)) {
      externalPullupFlag = true;
      writeback |= B00010000;
    }
    if (!(sys_stat.regByte & B00001000)) {
      underVoltageFlag = true;
      writeback |= B00001000;
    }
    if (!(sys_stat.regByte & B00000100)) {
      overVoltageFlag = true;
      writeback |= B00000100;
    }
    if (!(sys_stat.regByte & B00000010)) {
      shortCircuitDischargingFlag = true;
      writeback |= B00000010;
    }
    if (!(sys_stat.regByte & B00000001)) {
      overCurrentDischargingFlag = true;
      writeback |= B00000001;
    }

    writeRegister(SYS_STAT, writeback);
  }
}

//----------------------------------------------------------------------------
// reads all cell voltages to array cellVoltages[4] and updates batVoltage

void bq769x0::updateVoltages()
{

  // read cell voltages

  uint32_t adcVal = 0;
  uint8_t buf[4];
  int connectedCellsTemp = 0;
  idCellMaxVoltage = 0; //resets to zero before writing values to these vars
  idCellMinVoltage = 0;
  byte connectedCells = 0;

  uint8_t crc;
  crc = 0;
  buf[0] = (char) VC1_HI_BYTE; // start with the first cell


  Wire.beginTransmission(I2CAddress); //arduino version
  Wire.write(buf[0]); //Tells slave that this is the device and the address it is interested in
  Wire.endTransmission(false); // end transmission so that read can begin



  // Note that each cell voltage is 14 bits stored across two 8 bit register locations in the BQ chip
  // This means that first we need to read register HI (in first instance this is VC1_HI_BYTE) - however this 8 bit piece of data has two worthless first digits - garbage
  // So to remove the first to bits, the bitwise & is used. By saying A & 00111111, only the last 6 bits are used.
  // all of the 8 bits on the low side are used. So the overall reading is the last 6 bits of high in front of the 8 bits from low.
  // To add the hi and lo readings together, the << is used to shift the hi value over by 8 bits, then adding it to the 8 bits
  // This is done by using the OR operator |. So the total thing looks like: adcVal = (VC_HI_BYTE & 0b00111111) << 8 | VC_LO_BYTE;

  uint8_t packagecells = 5;
  if (type == bq76930)
    packagecells = 10;
  if (type == bq76940)
    packagecells = 15;

  for (int i = 0; i < packagecells; i++) // will run once for each cell up to the total num of cells
  {
    if (crcEnabled == true) {
      /** Mbed version - request 4 bytes and write to buf *******/
      // first request data from BQ chip - 4 bytes total, two for hi and lo, and two for CRC for each byte
      //Note that the return is: buf[0]: hi byte, buf[1] : CRC for 1st byte, buf[2] : Lo Byte, buf[3] CRC for lo byte
      //     _i2c.read(I2CAddress << 1, buf, 4); // will request data from buffer from : (i2c address, data storage location,# of bytes)

      //   adcVal = (buf[0] & 0b00111111) << 8 | buf[2]; // From datasheet V(cell) = GAIN x ADC(cell) + OFFSET

      /******************* Arduino Version ******************/


      Wire.requestFrom(I2CAddress, 4, true); // requests 4 bytes: 1)hi Byte 2)Hi byte CRC 3) Lo Byte 4) Lo byte crc
      //~ // bytes are written to register, so now they need to be called one at a time and processed
      buf[0] = Wire.read(); // hi data - note that only bottom 6 bits are good
      buf[1] = Wire.read(); // hi data CRC - done on address and data
      buf[2] = Wire.read(); // lo data byte - all 8 bits are used
      buf[3] = Wire.read(); // lo data CRC - done on just the data byte

      //~ // 1st check if CRC matches data bytes
      //~ // CRC of first bytes includes slave address (including R/W bit) and data
      crc = _crc8_ccitt_update(0, (I2CAddress << 1) | 1);
      crc = _crc8_ccitt_update(crc, buf[0]);
      if (crc != buf[1]) {
        connectionErrorFlag = true;
        return; //don't save corrupt value and exit
      }

      //~ // CRC of subsequent bytes contain only data
      crc = _crc8_ccitt_update(0, buf[2]);
      if (crc != buf[3]) {
        connectionErrorFlag = true;
        return; // don't save corrupted value
      }


    }
    else
    { // If CRC is disabled only read 2 bytes and call it a day :)

      /*****************   ARM Mbed version *******************
              _i2c.read(I2CAddress << 1, buf, 2);
              adcVal = (buf[0] & 0b00111111) << 8 | buf[1];

      	******************************************************/
      /************* Arduino versino **************************/


      Wire.requestFrom(I2CAddress, 2); // requests 4 bytes: 1)hi Byte 2)Hi byte CRC 3) Lo Byte 4) Lo byte crc
      // bytes are written to register, so now they need to be called one at a time and processed
      buf[0] = Wire.read(); // hi data - note that only bottom 6 bits are good
      buf[2] = Wire.read(); // lo data byte - all 8 bits are used

    }

    /************* Get Cell Voltages now ***********/
    adcVal = ((uint16_t)buf[0] & 0b00111111) << 8 | buf[2];  // reads 1st byte from register and drop the first two bits, shift left then use lo data

    cellVoltages[i] = adcVal * adcGain / 1000 + adcOffset; // calculates real voltage in mV

    /********** Filter out fake voltage readings from non-connected cells ****************/
    if (cellVoltages[i] > 500) {
      cellVoltagesF[connectedCellsTemp] = cellVoltages[i];
      connectedCellsTemp++; //adds one to the temporary cell counter var - only readings above 500mV are counted towards real cell count
    }

    if (cellVoltages[i] > cellVoltages[idCellMaxVoltage]) { // if the current cell voltage is higher than the last cells, bump this up
      idCellMaxVoltage = i;
    }
    if (cellVoltages[i] < cellVoltages[idCellMinVoltage] && cellVoltages[i] > 500) {
      idCellMinVoltage = i;
    }
  }
  connectedCells = connectedCellsTemp;

  // read battery pack voltage
  adcVal = (readRegister(BAT_HI_BYTE) << 8) | readRegister(BAT_LO_BYTE);
  batVoltage = 4.0 * adcGain * adcVal / 1000.0 + connectedCells * adcOffset;
  detectedCells = connectedCells;

}







// Taken from mbed library
void bq769x0::writeRegister(byte address, int data) //from mbed library - changed var type of address to byte from int
{
  uint8_t crc = 0;
  char buf[3];

  buf[0] = (char) address;
  buf[1] = data;
  //Note that writes to BQ chip are 1)start - 2) address - 3)register address - 4)data - 5)CRC8 - 6)stop bit
  Wire.beginTransmission(I2CAddress); //first two steps - in write is start bit followed by device address
  Wire.write(buf[0]); //writes register address
  Wire.write(buf[1]); // writes data - the fourth step

  if (crcEnabled == true) {
    // CRC is calculated over the slave address (including R/W bit), register address, and data.
    crc = _crc8_ccitt_update(crc, (I2CAddress << 1) | 0);
    crc = _crc8_ccitt_update(crc, buf[0]);
    crc = _crc8_ccitt_update(crc, buf[1]);
    buf[2] = crc;

    Wire.write(buf[2]); //writes CRC check for arduino
  }

  Wire.endTransmission(false);

}



//----------------------------------------------------------------------------
//From the arduino library, doesn't support CRC
int bq769x0::readRegister(byte address)
{
  Wire.beginTransmission(I2CAddress);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(I2CAddress, 1);
  return Wire.read();
}

/***
int bq769x0::readRegister(int address)
{
    uint8_t crc = 0;
    char buf[2];

    #if BQ769X0_DEBUG
    //printf("Read register: 0x%x \n", address);
    #endif

    buf[0] = (char)address;
	//BQ chip read works as:  1)Start bit - 2)Slave address - 3)Register Address --4)Start bit again 5) Slave address -- 6) Slave drives data 7)Slave drives CRC

	Wire.beginTransmission(I2CAddress); // first two steps - start bit then slave address is written
	Wire.write(buf[0]); //3rd step, writes register address
    //_i2c.write(I2CAddress << 1, buf, 1);; //ARM mbed specific
Wire.beginTrans
	//from the ARM mbed library - modded to work with arduino
    if (crcEnabled == true) {
        do {
            _i2c.read(I2CAddress << 1, buf, 2);
            // CRC is calculated over the slave address (including R/W bit) and data.
            crc = _crc8_ccitt_update(crc, (I2CAddress << 1) | 1);
            crc = _crc8_ccitt_update(crc, buf[0]);
        } while (crc != buf[1]);
        return buf[0];
    }
    else {
        _i2c.read(I2CAddress << 1, buf, 1);
        return buf[0];
    }
}
**/
//----------------------------------------------------------------------------
// the actual ISR, called by static function alertISR()

void bq769x0::setAlertInterruptFlag()
{
  interruptTimestamp = millis();
  alertInterruptFlag = true;
}

//----------------------------------------------------------------------------
// The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
// a new value (either new CC reading or an error)

void bq769x0::alertISR()
{
  if (instancePointer != 0)
  {
    instancePointer->setAlertInterruptFlag();
  }
}


#if BQ769X0_DEBUG

//----------------------------------------------------------------------------
// for debug purposes

void bq769x0::printRegisters()
{
  Serial.print(F("0x00 SYS_STAT:  "));
  Serial.println(byte2char(readRegister(SYS_STAT)));

  Serial.print(F("0x01 CELLBAL1:  "));
  Serial.println(byte2char(readRegister(CELLBAL1)));

  Serial.print(F("0x02 CELLBAL2:  "));
  Serial.println(byte2char(readRegister(CELLBAL2)));

  Serial.print(F("0x04 SYS_CTRL1: "));
  Serial.println(byte2char(readRegister(SYS_CTRL1)));

  Serial.print(F("0x05 SYS_CTRL2: "));
  Serial.println(byte2char(readRegister(SYS_CTRL2)));

  Serial.print(F("0x06 PROTECT1:  "));
  Serial.println(byte2char(readRegister(PROTECT1)));

  Serial.print(F("0x07 PROTECT2:  "));
  Serial.println(byte2char(readRegister(PROTECT2)));

  Serial.print(F("0x08 PROTECT3   "));
  Serial.println(byte2char(readRegister(PROTECT3)));

  Serial.print(F("0x09 OV_TRIP:   "));
  Serial.println(byte2char(readRegister(OV_TRIP)));

  Serial.print(F("0x0A UV_TRIP:   "));
  Serial.println(byte2char(readRegister(UV_TRIP)));

  Serial.print(F("0x0B CC_CFG:    "));
  Serial.println(byte2char(readRegister(CC_CFG)));

  Serial.print(F("0x32 CC_HI:     "));
  Serial.println(byte2char(readRegister(CC_HI_BYTE)));

  Serial.print(F("0x33 CC_LO:     "));
  Serial.println(byte2char(readRegister(CC_LO_BYTE)));

  Serial.print(F("0x50 ADCGAIN1:  "));
  Serial.println(byte2char(readRegister(ADCGAIN1)));

  Serial.print(F("0x51 ADCOFFSET: "));
  Serial.println(byte2char(readRegister(ADCOFFSET)));

  Serial.print(F("0x59 ADCGAIN2:  "));
  Serial.println(byte2char(readRegister(ADCGAIN2)));

  Serial.print(F("VC1 Hi:     "));
  Serial.println(byte2char(readRegister(VC1_HI_BYTE)));

  Serial.print(F("VC1 Lo:     "));
  Serial.println(byte2char(readRegister(VC1_LO_BYTE)));

  Serial.print(F("VC2 Hi:     "));
  Serial.println(byte2char(readRegister(VC2_HI_BYTE)));

  Serial.print(F("VC2 Lo:     "));
  Serial.println(byte2char(readRegister(VC2_LO_BYTE)));

  Serial.print(F("VC3 Hi:     "));
  Serial.println(byte2char(readRegister(VC3_HI_BYTE)));

  Serial.print(F("VC3 Lo:     "));
  Serial.println(byte2char(readRegister(VC3_LO_BYTE)));
}

#endif
