# bq769x0 Arduino Library

Note - Testing only done on BQ7693003 WITH CRC

Arduino-compatible library for battery management system based on Texas Instruments bq769x0 IC (bq76920, bq76930 and bq76940).

The library offerst most features for a simple BMS (including automatic fault handling and balancing). See also BMS48V hardware files.


## Example Arduino sketch

```C++
#include <EEPROM.h>
#include <bq769x0.h>
#include <registers.h>
#include <avr/eeprom.h>

// BQ76200PWR High side driver in use

#define BMS_ALERT_PIN 8     // attached to interrupt INT0
#define BMS_BOOT_PIN 9      // connected to TS1 input
#define BMS_I2C_ADDRESS 0x08 //0x08 for  part nums ending in BQ769300?DBT with ? being 0-3. For 6 or 7, use 0x18. Chip from LCSC was 3DBT
#define CELLCOUNT 7
#define PRECHARGE_PIN 4
#define PMON_EN 6
#define L_G 14
#define L_Y 16
#define L_R 10
#define BUZZER_PIN 7
#define CHARGEPUMP_EN 5


#define PRINT(x) Serial.print(x); Serial1.print(x);
#define PRINTLN(x) Serial.println(x); Serial1.println(x);

bq769x0 BMS(bq76930, CELLCOUNT);    // battery management system object

unsigned long previousMillis = 0;        // will store last time status was updated
const long interval = 250;

struct BMSCal_t {
  float est_soc;
  float r_sens;
  int8_t min_charge_temp;
  int8_t max_charge_temp;
  int8_t min_discharge_temp;
  int8_t max_discharge_temp;
  float pack_capacity;
  uint16_t thermistorbeta;
  uint8_t use_thermistors;
  uint8_t RSNS;
  uint16_t mincell_v;
  uint16_t maxcell_v;
  uint8_t cellhysteresis_d;
  uint8_t cellhysteresis_c;
  float scd_current;
  float overcurrent_d;
  float overcurrent_c;
  uint16_t balance_minv;
  uint8_t balance_diff;
  uint16_t balance_mincurrent;
  uint16_t lowvolt_alert;
} bms_cal;


bool initEEPROM(void) {
  char eeinit;
  EEPROM.get(0, eeinit);
  if (eeinit != 'a') {
    bms_cal.est_soc = 0.0;
    bms_cal.r_sens = 0.5;
    bms_cal.min_charge_temp = 0;
    bms_cal.max_charge_temp = 45;
    bms_cal.min_discharge_temp = -20;
    bms_cal.max_discharge_temp = 50;
    bms_cal.pack_capacity = 75000;
    bms_cal.thermistorbeta = 3435;
    bms_cal.use_thermistors = 0;
    bms_cal.RSNS = 1;
    bms_cal.mincell_v = 3050;
    bms_cal.maxcell_v = 4150;
    bms_cal.cellhysteresis_d = 100;
    bms_cal.cellhysteresis_c = 50;
    bms_cal.scd_current = 170;
    bms_cal.overcurrent_d = 140;
    bms_cal.overcurrent_c = 20;
    bms_cal.balance_minv = 3800;
    bms_cal.balance_diff = 20;
    bms_cal.balance_mincurrent = 1000;
    bms_cal.lowvolt_alert = bms_cal.mincell_v + 150;
    size_t addr = sizeof(char);
    EEPROM.put(addr, bms_cal);
    EEPROM.update(0, 'i');  
    return true;
  }
  return false;
}



void handleFaults(void)
{

 fault_t fault = BMS.getFaults();
 uint8_t forceexit=0;
 while (fault != none && forceexit < 8) // Print all faults
 {
   switch (fault)
   {
     case (internalFault):
        PRINTLN("F,Internalfault");
        forceexit++;
       break;
     case (undervoltgeFault):
       tone(BUZZER_PIN,98,1000);
       forceexit++;
       PRINTLN("F,Undervoltage");
       break;
     case (overvoltageFault):
       tone(BUZZER_PIN,330,1000);
       forceexit++;
       PRINTLN("F,Overvoltage");
       break;
     case (shortcircuitFault):
       tone(BUZZER_PIN,220,1000);
       forceexit++;
       PRINTLN("F,SCD");
       break;
     case (overcurrentDischargeFault):
       tone(BUZZER_PIN,3322,1000);
       forceexit++;
       PRINTLN("F,OvercurrentDsg");
       break;
     case (overcurrentChargeFault):
       tone(BUZZER_PIN,1568,1000);
       forceexit++;
       PRINTLN("F,OvercurrentChg");
       break;
     case (externalPullupFault):
       tone(BUZZER_PIN,4699,1000);
       forceexit++;
       PRINTLN("F,ExtPullup");
       break;
     case (connectionErrorFault):
       tone(BUZZER_PIN,1200,200);
       delay(200);
       tone(BUZZER_PIN,1200,200);
       forceexit++;
       PRINTLN("F,BMSCONN");
       break;
     default:
       forceexit++;
       PRINTLN("F,UNKWN");
       break;
   }
 }
}

void enablePrecharge()
{
  digitalWrite(PRECHARGE_PIN, HIGH);
}
void disablePrecharge()
{
  digitalWrite(PRECHARGE_PIN, LOW);
}

void setup() {
  Serial1.begin(9600);
  delay(500);
  pinMode(PRECHARGE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(L_R, true);
  digitalWrite(L_Y, true);
  digitalWrite(L_G, true);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(L_R, OUTPUT);
  pinMode(L_Y, OUTPUT);
  pinMode(L_G, OUTPUT);
  digitalWrite(PRECHARGE_PIN, LOW);
  delay(4000);
  Serial.begin(115200);
  PRINTLN("M,Starting BMS object");

  if (initEEPROM())
  {
    PRINTLN("M,Initializing EEPROM");
  }
  // Read eeprom
  size_t addr = sizeof(char);
  EEPROM.get(addr, bms_cal);
  PRINTLN("M,Init");
  BMS.initialize(BMS_BOOT_PIN);
  PRINTLN("M,Firstconnect");
  int err = BMS.begin(BMS_ALERT_PIN,CELLCOUNT, bms_cal.RSNS);
  PRINT("M,Checking: ");
  PRINTLN(err);
  do
  {
    if (err != 0)
    {
      PRINTLN("M,Filed to connect to bq796x0, engaging boot");
      BMS.initialize(BMS_BOOT_PIN);
      delay(1000);
      err = BMS.begin(BMS_ALERT_PIN,CELLCOUNT, bms_cal.RSNS);
    }
    tone(BUZZER_PIN, 77, 333);
    if (err == 0)
    {
      PRINTLN("M,Boot success");
      BMS.setTemperatureLimits(bms_cal.min_discharge_temp, bms_cal.max_discharge_temp, bms_cal.min_charge_temp, bms_cal.max_charge_temp);
      BMS.setShuntResistorValue(bms_cal.r_sens);
      PRINT("S,SCD,");
      PRINTLN(BMS.setShortCircuitProtection(bms_cal.scd_current, 200));  // delay in us
      BMS.setOvercurrentChargeProtection(bms_cal.overcurrent_c, 200);  // delay in ms
      PRINT("S,OCD,");
      PRINTLN(BMS.setOvercurrentDischargeProtection(bms_cal.overcurrent_d, 320)); // delay in ms
      PRINT("S,CUV,");
      PRINTLN(BMS.setCellUndervoltageProtection(bms_cal.mincell_v, 2)); // delay in s
      PRINT("S,COV,");
      PRINTLN(BMS.setCellOvervoltageProtection(bms_cal.maxcell_v, 2));  // delay in s
      BMS.setBalancingThresholds(0, bms_cal.balance_minv, bms_cal.balance_diff);  // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
    
      BMS.setIdleCurrentThreshold(bms_cal.balance_mincurrent); // mA
      //BMS.enableAutoBalancing();
      // BMS.disableAutoBalancing(); //ensure balancing is off so we don't fry anything during testing
      // BMS.enableDischarging();
      tone(BUZZER_PIN, 999, 333);
      tone(BUZZER_PIN, 3333, 333);
      PRINTLN("M,BMS Settings set");
    }
  } while (err != 0);

  PRINTLN("M,BMS object started");
  BMS.update();
  previousMillis = millis();
}

void LedR(bool on)
{
  digitalWrite(L_R, !on);
}
void LedY(bool on)
{
  digitalWrite(L_Y, !on);
}
void LedG(bool on)
{
  digitalWrite(L_G, !on);
}

void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    BMS.update();  // should be called at least every 250 ms
    handleFaults();
    

    // Enable discharging once voltage has risen
    if (BMS.getMinCellVoltage() + bms_cal.cellhysteresis_d > bms_cal.mincell_v)
    {
      if (!BMS.enableDischarging() && !BMS.getDsgFet())
        PRINTLN("F,DSGFETCommand");
        
    }
    // Low battery alert when running.
    if (BMS.getMinCellVoltage() < bms_cal.lowvolt_alert)
    {
      if (BMS.getBatteryCurrent() > 50) // Need a bit drain, not when idle or charging
      {
        tone(BUZZER_PIN,2400, 100);
        delay(50);
        tone(BUZZER_PIN,2100, 100);
        
      }
    }


    // Enable charging
    if (BMS.getMaxCellVoltage() < bms_cal.maxcell_v - bms_cal.cellhysteresis_c)
    {
      // Normal charging
      if (BMS.getMinCellVoltage() > bms_cal.mincell_v + (bms_cal.cellhysteresis_c/4))
      {
        if (!BMS.enableCharging() && !BMS.getChgFet())
          PRINTLN("F,CHGFETCommand");
        disablePrecharge();
      }
      else if (BMS.getMinCellVoltage() > 0.6)// Precharge from depleted state
      {
        BMS.disableCharging();
        enablePrecharge();
        tone(BUZZER_PIN,2093, 150);
      }
      else // Battery? Where are you?
      {
        BMS.disableCharging();
        disablePrecharge();
      }
    }
    
    
    for (uint8_t i = 0; i < BMS.getCellCount();i++)
    {
      PRINT("C,");
      PRINT(i);
      PRINT(",");
      if (!(BMS.getBalancingStatus() & (1 << i)))
      {
        PRINT("X,");
      }
      else
      {
        PRINT("O,");
      }
      
      PRINTLN(BMS.getCellVoltageFiltered(i));
    }
    PRINT("Max,");
    PRINTLN(BMS.getMaxCellVoltage());
    PRINT("Min,");
    PRINTLN(BMS.getMinCellVoltage());
    PRINT("PackV,");
    PRINTLN(BMS.getBatteryVoltage());
    PRINT("PackC,");
    PRINTLN(BMS.getBatteryCurrent());
    PRINT("PCHG,");
    PRINTLN(digitalRead(PRECHARGE_PIN));
    PRINT("CHG,");
    PRINTLN(BMS.getChgFet());
    PRINT("DSG,");
    PRINTLN(BMS.getDsgFet());
    LedG(BMS.getDsgFet());
    LedY(BMS.getChgFet());
    LedR(digitalRead(PRECHARGE_PIN));

  }
}
```

## To Do list

- Proper SOC estimation and coloumb counter implementation
