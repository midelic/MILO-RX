//Pinout definitions
//GPIO 6 to GPIO11 are conected to SPI flash and not recommended to use
//***********************************************************
//SX1280
#define BIND_pin         0 //GPIO 0 ;Flashing  pin
#define LED_pin          16
#define SX1280_RST_pin   2      
#define SX1280_BUSY_pin  5
#define SX1280_DIO1_pin  4  
//SPI   
#define SX1280_MOSI_pin    13
#define SX1280_MISO        12
#define SX1280_SCK         14
#define SX1280_CSN_pin     15
//Serial
#define SPORT_pin 3 // RX and TX SPORT
//#define SBUS_pin   1//SBUS   // for ESP8266, Sbus is always generated on the pin 1 because it uses Serial.print()
//Frontend PA/LNA
#ifdef MATEK_RX_R24D
    #define SX1280_TXEN_pin      10
    #define SX1280_RXEN_pin      -1 
    #define SX1280_ANTENNA_SELECT_pin  9
#elif defined BETA_FPV_RX_NANO || defined MATEK_RX_R24S
    #define SX1280_RXEN_pin      9 
    #define SX1280_TXEN_pin      10
#elif defined NAMIMNO_RX_FLASH_NANO
    #define SX1280_TXEN_pin            0
    #define SX1280_RXEN_pin      -1 
    #define SX1280_ANTENNA_SELECT_pin  9
#elif defined DIY_RX
    #define SX1280_ANTENNA_SELECT_pin  9
#endif

#ifdef ESP8266_E28_2G4M20S
    #undef SX1280_BUSY_pin
    #undef SX1280_RXEN_pin
    #undef SX1280_TXEN_pin
    #undef LED_pin
    #define LED_pin         -1
    #define SX1280_BUSY_pin  16
    #define SX1280_RXEN_pin -1
    #define SX1280_TXEN_pin  5
#endif
    
#define IS_BIND_BUTTON_on   ( (BIND_pin != -1) && (digitalRead(BIND_pin)==LOW))
    
#define IS_SX1280_DIO1_on       ( digitalRead(SX1280_DIO1_pin)==HIGH )
#define IS_SX1280_DIO1_off      ( digitalRead(SX1280_DIO1_pin)==LOW )
#define IS_SX1280_BUSY_on       ( digitalRead(SX1280_BUSY_pin)==HIGH )
#define IS_SX1280_BUSY_off      ( digitalRead(SX1280_BUSY_pin)==LOW)

#define SX1280_RST_on              digitalWrite(SX1280_RST_pin,HIGH)
#define SX1280_RST_off             digitalWrite(SX1280_RST_pin,LOW)
#define SX1280_TXEN_on             digitalWrite(SX1280_TXEN_pin,HIGH)
#define SX1280_RXEN_on             digitalWrite(SX1280_RXEN_pin,HIGH)
#define SX1280_TXEN_off            digitalWrite(SX1280_TXEN_pin,LOW)
#define SX1280_RXEN_off            digitalWrite(SX1280_RXEN_pin,LOW)
#define SX1280_ANT_SEL_on          if (SX1280_ANTENNA_SELECT_pin != -1) digitalWrite(SX1280_ANTENNA_SELECT_pin,HIGH)
#define SX1280_ANT_SEL_off         if (SX1280_ANTENNA_SELECT_pin != -1) digitalWrite(SX1280_ANTENNA_SELECT_pin,LOW)
#define SX1280_CSN_on              digitalWrite(SX1280_CSN_pin,HIGH)
#define SX1280_CSN_off             digitalWrite(SX1280_CSN_pin,LOW)

#define LED_on                     digitalWrite(LED_pin,HIGH)
#define LED_off                    digitalWrite(LED_pin,LOW)
#define LED_toggle                 digitalWrite(LED_pin ,!digitalRead(LED_pin))

#define USE_SX1280_DCDC
