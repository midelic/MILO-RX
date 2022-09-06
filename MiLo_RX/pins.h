//Pinout definitions
//GPIO 6 to GPIO11 are conected to SPI flash and not recommended to use
//***********************************************************
//SX1280
#define BIND_pin         0 //GPIO 0 ;Flashing  pin
#define LED_pin          16
#define	SX1280_RST_pin   2		
#define	SX1280_BUSY_pin  5
#define	SX1280_DIO1_pin  4	
//SPI	
#define SX1280_MOSI_pin    13
#define SX1280_MISO        12
#define SX1280_SCK         14
#define SX1280_CSN_pin     15
//Serial
#define SX1280_SPORT_RX_pin 3 // RX SPORT
#define SX1280_SPORT_TX_pin 3 // TX SPORT
#define SX1280_SBUS_TX_pin   1//SBUS
//Frontend PA/LNA
#define SX1280_RXEN_pin      -1 
#define SX1280_TXEN_pin      -1
#ifdef MATEK_RX_R24D
    #define SX1280_TXEN_pin            10
    #define SX1280_ANTENNA_SELECT_pin  9
    #define POWER_OUTPUT_FIXED         3
#elif defined BETA_FPV_RX_NANO || defined MATEK_RX_R24S
    #define SX1280_RXEN_pin      9 
    #define SX1280_TXEN_pin      10
    #define POWER_OUTPUT_FIXED   3
#elif defined NAMIMNO_RX_FLASH_NANO
    #define SX1280_TXEN_pin            0
    #define SX1280_ANTENNA_SELECT_pin  9
    #define POWER_OUTPUT_FIXED         3   
#endif

#ifdef EL24P
    #define POWER_OUTPUT_FIXED      13	
#endif

#ifdef DIY_RX
	#define POWER_OUTPUT_FIXED      13
	#define SX1280_ANTENNA_SELECT_pin  9
#endif

#define BIND_SET_INPUT		pinMode(BIND_pin,INPUT)
#define BIND_SET_PULLUP		digitalWrite(BIND_pin,HIGH)	
#define BIND_SET_OUTPUT		pinMode(BIND_pin,OUTPUT)
    
#define IS_BIND_BUTTON_on	(digitalRead(BIND_pin)==LOW)
    
#define	IS_SX1280_DIO1_on		( digitalRead(SX1280_DIO1_pin)==HIGH )
#define	IS_SX1280_DIO1_off		( digitalRead(SX1280_DIO1_pin)==LOW )
#define	IS_SX1280_BUSY_on		( digitalRead(SX1280_BUSY_pin)==HIGH )
#define	IS_SX1280_BUSY_off		( digitalRead(SX1280_BUSY_pin)==LOW)
#define	IS_LED_on		                ( digitalRead(LED_pin)==HIGH)

#define	SX1280_RST_on	           digitalWrite(SX1280_RST_pin,HIGH)
#define	SX1280_RST_off	           digitalWrite(SX1280_RST_pin,LOW)
#define	SX1280_TXEN_on	           digitalWrite(SX1280_TXEN_pin,HIGH)
#define	SX1280_RXEN_on	           digitalWrite(SX1280_RXEN_pin,HIGH)
#define	SX1280_TXEN_off	           digitalWrite(SX1280_TXEN_pin,LOW)
#define	SX1280_RXEN_off	           digitalWrite(SX1280_RXEN_pin,LOW)
#define SX1280_ANT_SEL_on 	   digitalWrite(SX1280_ANTENNA_SELECT_pin,HIGH)
#define SX1280_ANT_SEL_off         digitalWrite(SX1280_ANTENNA_SELECT_pin,LOW)
#define SX1280_CSN_on              digitalWrite(SX1280_CSN_pin,HIGH)
#define SX1280_CSN_off             digitalWrite(SX1280_CSN_pin,LOW)

#define	LED_on			               digitalWrite(LED_pin,HIGH)
#define	LED_off			               digitalWrite(LED_pin,LOW)
#define	LED_toggle		           digitalWrite(LED_pin ,!digitalRead(LED_pin))
#define	LED_output		            pinMode(LED_pin,OUTPUT)

#define USE_SX1280_DCDC

