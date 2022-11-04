/*
Here some documentation to understand the flow.


uplink telemetry frame:
- data from SX1280 are read in copied to RxData[] in main loop()
- data from  RxData[] are copied into ReceivedSportData[i]
- each byte of ReceivedSportData[i] is processed one by one by smartPortDataReceive() 
- smartPortDataReceive() synchronize on 0X7E, remove stuffing et copy into sportMSPdata[]
- when we have 8 bytes in sportMSPdata[], set sportMSPflag = true and call sportMSPstuff()
- sportMSPstuff() rempli sportMSPdatastuff[] avec 0X7E et le contenu de sportMSPdata[] avec stuffing + CRC
- at regular interval (12msec), a timer1 fire and call SportPollISR() that set a flag sportPollIsrFlag
- main loop call handleSportPoll()  when sportPollIsrFlag is set 
- handleSportPoll() , if sportMSPflag is set, copies  sportMSPdatastuff[] into sTxData[] and call sendMSPpacket()
- sendMSPpacket() sent sTxDAta[] to the sensor via UART
   

Incomming sport data
- at regular interval (12msec), a timer1 fire and call SportPollISR() that set a flag sportPollIsrFlag
- main loop call handleSportPoll()  when sportPollIsrFlag is set 
- handleSportPoll() , if sportMSPflag is NOT set,  call tx_sport_poll()
- tx_sport_poll() fill sTxData[] with 2 bytes for a polling request and output the 2 bytes on the Sport port (UART)
- after sending the 2 bytes, Sport bus listen for a reply from sensor
- normally a sensor replies on the sport port and data are accumulated in sportbuff[[]
      With #define DEBUG_SIM_SPORT_SENSOR , we can simulate incomming data from sensor (filling Sxdata[] and calling ProcesssportData()). 
- main loop, call callSportSwSerial() 
- callSportSwSerial() look if we have a full frame available (>= 8 bytes and more than 500usec since previous income)
- if so, copy sportbuff[[] to sRxData[] and call ProcessSportData()
- ProcessSportData() remove stuffing inside sRxData, check CRC and if ok call checkSimilarSport()
- checkSimilarSport() update or add an entry in circular buffer sportData[] with sRxData
      circular buffer is managed by sportTail, sportTailIfAck, sportHead and sportDatalen
      each entry in sportData has 8 bytes (PHID, PRIM, ID1, ID2, Val1, Val2, val3, Val4)
- In main loop, when slot is for a downlink tlm frame and Rx is connected, call MiloTlmSent()
- MiloTlmSent() call MiLoTlm_build_frame() and then will write to SX1280
- MiLoTlm_build_frame() fill frame[] with data from link quality and or data from circular buffer sportData[]
    MiLoTlm_build_frame() checks if previous frame has been confirmed by Tx (update sportTail accordingly)
                          performs some compression in order to be able to put 2 set of data in one packet. 

*/