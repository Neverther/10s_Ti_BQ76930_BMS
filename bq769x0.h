/*
    bq769x0.h - Battery management system based on bq769x0 for Arduino
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

#ifndef BQ769X0_H
#define BQ769X0_H

#include <Arduino.h>
#include <Wire.h>

// can be reduced to save some memory if smaller ICs are used
#define MAX_NUMBER_OF_CELLS 15
#define MAX_NUMBER_OF_THERMISTORS 3

// IC type/size
#define bq76920 1
#define bq76930 2
#define bq76940 3

// output information to serial console for debugging
#define BQ769X0_DEBUG 1


enum fault_t
{
  none = 0,
  internalFault = 1,
  undervoltgeFault = 2,
  overvoltageFault = 3,
  shortcircuitFault = 4,
  overcurrentDischargeFault = 5,
  overcurrentChargeFault = 6,
  externalPullupFault = 7,
  connectionErrorFault = 8
};

class bq769x0 {

  public:



    // initialization, status update and shutdown
    bq769x0(byte bqType = bq76930, byte cellcount = 10);
    int begin(byte alertPin, byte bootPin = -1, uint8_t reg_RSNS = 0);
    int checkStatus();  // returns 0 if everything is OK
    void update(void);
    void shutdown(void);
    int initialize(byte bootpin = 255);
    // charging control
    bool enableCharging(void);
    void disableCharging(void);
    bool enableDischarging(void);
    void disableDischarging(void);

    // hardware settings
    void setShuntResistorValue(float res_mOhm);
    void setThermistorBetaValue(int beta_K);

    // limit settings (for battery protection)
    void setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC);    // °C
    float setShortCircuitProtection(float current_A, int delay_us = 70); // Also sets the RSNS value
    void setOvercurrentChargeProtection(float current_A, int delay_ms = 8);
    float setOvercurrentDischargeProtection(float current_A, int delay_ms = 8);
    int setCellUndervoltageProtection(int voltage_mV, int delay_s = 1);
    int setCellOvervoltageProtection(int voltage_mV, int delay_s = 1);

    // balancing settings
    void setBalancingThresholds(int idleTime_min = 30, int absVoltage_mV = 3400, byte voltageDifference_mV = 20);
    void setIdleCurrentThreshold(int current_mA);

    // automatic balancing when battery is within balancing thresholds
    void enableAutoBalancing(bool enable);
    bool isBalancing(void);

    // Fet driver
    bool getChgFet(void);
    bool getDsgFet(void);

    // battery status
    uint8_t getCellCount(void);
    float  getBatteryCurrent(void);
    float  getBatteryVoltage(void);
    int  getCellVoltage(byte idCell);    // from 1 to 15
    int  getCellVoltageFiltered(byte idCell);    // from 1 to 15
    int  getMinCellVoltage(void);
    int  getMaxCellVoltage(void);
    float getTemperatureDegC(byte channel = 1);
    float getTemperatureDegF(byte channel = 1);
    uint16_t getBalancingStatus(void);
    
    // interrupt handling (not to be called manually!)
    void setAlertInterruptFlag(void);
    
    fault_t getFaults(void);

#if BQ769X0_DEBUG
    void printRegisters(void);
#endif

  private:

    // Variables

    int I2CAddress;
    byte type;
    bool crcEnabled; //taken from mbed library - needed for determineAddressAndCrc funtion

    float shuntResistorValue_mOhm;
    int thermistorBetaValue = 3435;  // typical value for Semitec 103AT-5 thermistor

    // indicates if a new current reading or an error is available from BMS IC
    bool alertInterruptFlag = true;   // init with true to check and clear errors at start-up

    int numberOfCells;
    int cellVoltages[MAX_NUMBER_OF_CELLS];          // mV
    int cellVoltagesF[MAX_NUMBER_OF_CELLS];          // mV
    byte idCellMaxVoltage;
    byte idCellMinVoltage;
    float batVoltage;                                // mV
    float batCurrent;                                // mA
    int temperatures[MAX_NUMBER_OF_THERMISTORS];    // °C/10
    uint16_t balancingCells;

    // Current limits (mA)
    float maxChargeCurrent;
    float maxDischargeCurrent;
    uint16_t maxChargeCurrentDelay;
    int idleCurrentThreshold = 30; // mA
    bool overChargeCurrent;

    // Temperature limits (°C/10)
    int minCellTempCharge;
    int minCellTempDischarge;
    int maxCellTempCharge;
    int maxCellTempDischarge;

    // Cell voltage limits (mV)
    uint8_t detectedCells;
    int maxCellVoltage;
    int minCellVoltage;
    int balancingMinCellVoltage_mV;
    byte balancingMaxVoltageDifference_mV;

    int adcGain;    // uV/LSB
    int adcOffset;  // mV

    int errorStatus = 0;
    bool autoBalancingEnabled = false;
    bool balancingActive = false;
    int balancingMinIdleTime_s = 1800;    // default: 30 minutes
    unsigned long idleTimestamp = 0;
    unsigned long chargeTimestamp = 0;

    unsigned int secSinceErrorCounter = 0;
    unsigned long interruptTimestamp = 0;

    static bq769x0* instancePointer;

    // Methods
    bool determineAddressAndCrc(void); // taken from mbed - made to work on arduino
    static void alertISR(void);

    void  updateVoltages(void);
    void  updateCurrent(bool ignoreCCReadyFlag = false);
    void  updateTemperatures(void);

    void updateBalancingSwitches(void);
    byte RSNS;
    int  readRegister(byte address);
    void writeRegister(byte address, int data);
    
    // Faults
    bool overCurrentChargingFlag;
    bool overCurrentDischargingFlag;
    bool shortCircuitDischargingFlag;
    bool overVoltageFlag;
    bool underVoltageFlag;
    bool internalFaultFlag;
    bool externalPullupFlag;
    bool connectionErrorFlag;

};

#endif // BQ769X0_H

