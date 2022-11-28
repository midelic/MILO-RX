
/* **************************
    * By Midelic on RCGroups *
    **************************
    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    MiLo Rx code is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/


/**************************************/
/*** SELECT ONE AND ONLY ONE MODULE ***/
/**************************************/
//#define EL24P//flywoo RX
//#define DIY_RX
//#define MATEK_RX_R24D
//#define MATEK_RX_R24S
//#define BETA_FPV_RX_NANO // or Cycloon clone
//#define NAMIMNO_RX_NANO_FLASH
//#define ESP8266_E28_2G4M20S
#define RP2040_E28_2G4M12S

/**************************************/
/***       SELECT SOME OPTIONS      ***/
/**************************************/
#define MSW_SERIAL                 // use software serial (with interrupts) to send and receive Sport data on the sport bus
//#define HC_BIND
//#define USER_MAX_POWER           // limit the max power to the value specified here (can be less than what is allowed by the module)
#define TELEMETRY                  // use RF frames between TX and RX (for uplink or downlink (link quality and/or sensor data))   
//#define SWAMPING
//#define RSSI_AVG
#define SPORT_TELEMETRY            // 
#define TX_FAILSAFE                // accept to let the Tx define the failsafe values (otherwise they can be defined on RX level)
#define FAILSAFE                   // apply failsafe values (on Sbus and PWM) when connection is lost

//#define MinPower PWR_10mW
//#define MaxPower PWR_100mW
#define SBUS                       // let RX generates Sbus signal with RC channels
//#define PWM_SERVO                // let RX generates PWM signals for the servo
//#define ADC_VOLT
#define STATISTIC
//#define SERVO_RATE
//#define PARALLEL_SERVO
#define USE_WIFI                 // allow to activate the wifi for firmware updates

/********************/
/*** LOCAL CONFIG ***/
/********************/
//If you know parameters you want for sure to be enabled or disabled which survives in future, you can use a file named "_MyConfig.h".
//An example is given within the file named "_MyConfig.h.example" which needs to be renamed if you want to use it.
//To enable this config file remove the // from the line below.
#define USE_MY_CONFIG


