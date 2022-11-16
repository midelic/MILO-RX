
/*
    Protocol description:
    2.4Ghz LORA modulation
    - 142 Hz frame rate(7ms)
    - Data Rate ~76kb/s(-108dBm)
    - Bw-812; SF6 ; CR -LI -4/7 .
    - Preamble length 12 symbols
    - Fixed length packet format(implicit) -16 bytes
    - Downlink telemetry rate(1:3)
    - Uplink telemetry rate(1:6)
    - Hardware CRC is ON.
    
    # Normal frame channels 1-8 or 9-16; frame rate 7ms.
    ----------------------------------------------------
    0. - bits 7..6 = reserve (2 bits)
       - bits 5..4 = next expected telemetry down link frame counter(sequence) (2 bits=4 val)
       - bit 3     = Flag EU LBT
       - bits 2..0 = Frame type (3 bits)
    1. txid1 TXID on 16 bits
    2. txid2
    3. - bit 7     = flag next frame must be dwn tlm frame
       - bit 6     = flag requesing starting WIFI
       - bits 5..0 = Model ID /Rx_Num (6 bits) 
    4. channels 8 channels/frame ; 11bits/channel
    5. channels total 11 bytes of channels data in the packet frame
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
    15. Index of RF channel (0...37) ; used by Rx to (check) synchronize

    # TX uplink telemetry frame can be sent separate ;frame rate 7ms;1:6 telemetry data rate.
    -----------------------------------------------------------------------------------------
    0. - bits 7..6 = reserve (2 bits)
       - bits 5..4 = next expected telemetry down link frame counter(sequence) (2 bits=4 val)
       - bit 3     = reserve
       - bits 2..0 = Frame type (3 bits)
    1. txid1 TXID on 16 bits
    2. txid2
    3. - bits 7..6 = telemetry uplink counter sequence(2 bits)
       - bits 5..0 = Model ID /Rx_Num(6 bits)
    4.Sport data byte1
    5.Sport data byte 2
    6.Sport data byte 3
    7.Sport data byte 4
    8.SPort data byte 5
    9.SPort data byte 6
    10.SPort data byte 7
    11.SPort data byte 8
    12.Reserve
    13.Reserve
    14.Reserve  
    15.Index of RF channel (0...37) ; used by Rx to (check) synchronize

    # RX downlink telemetry frame sent separate at a fixed rate of 1:3;frame rate 7ms. Can contain 2 Sport frames 
    -------------------------------------------------------------------------------------------------------------
    0. - bits 7...2 : MSB of TXID1 (6 bits)
       - bits 1...0 : current downlink tlm counter (2 bits); when received TX should send this counter + 1type of link data packet(RSSI/SNR /LQI) (2 bits= 3 values currently) 
    1. - bits 7...2 : MSB of TXID2 (6 bits)
       - bits 1...0 : last upllink tlm counter received (2 bits); 
    2. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame1 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>3)
         bits 4..0 : PHID from sport frame1 (5 bits using a mask 0X1F; 0X1F = no data; 0X1E = link quality data)
    3. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame2 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>0)
         bits 4..0 : PHID from sport frame2 (5 bits using a mask 0X1F; 0X1F = no data; 0X1E = link quality data)
    4. field ID1 from sport frame1
    5. field ID2 from sport frame1
    6...9. Value from sport frame1 (4 bytes)
    10. field ID1 from sport frame2
    11. field ID2 from sport frame2
    12...15. Value from sport frame2 (4 bytes)
    
    
    # bind packet
    -------------
    0. Frame type = BIND_PACKET = 0
    1. rx_tx_addr[3];
    2. rx_tx_addr[2];
    3. rx_tx_addr[1];
    4. rx_tx_addr[0];
    5. RX_num;
    6. chanskip;
    7. up to 15.  0xA7


    # Frame Sequence for sub protocol MCH16 or MEU_16
    -------------------------------------------------
    0- downlink telemetry
    1- RC channels 1_8
    2- RC channels 9_16
    3- downlink telemetry
    4- RC channels 1_8
    5 -uplink telemetry                 
    6- downlink telemetry
    7- RC channels 9_16
    8- RC channels 1_8
    9- downlink telemetry
    10- RC channels 9_16
    11- uplink telemetry               
    12- downlink telemetry
    13- RC channels 1_8
    14- RC channels 9_16
    15- downlink telemetry
    
    # Frame Sequence for sub protocol MCH_8 or MEU_8
    -------------------------------------------------
    0 - downlink telemetry
    1- RC channels 1_8         
    2- RC channels 1_8      
    3- downlink telemetry      
    4- RC channels 1_8       
    5 -uplink telemetry              
    6- downlink telemetry      
    7- RC channels 1_8         
    8- RC channels 1_8          
    9- downlink telemetry            
    10- RC channels 1_8       
    11- uplink telemetry           
    12- downlink telemetry
    13- RC channels 1_8            
    14- RC channels 1_8              
    15- downlink telemetry
    16- RC channels 1_8            
    17  uplink telemetry              
    15- downlink telemetry
    
-------------------------
Here some documentation to understand the flow.
A while(1) in Main loop 
    - check if a frame has been received (an DOI1ISR() set a flag)
          when received packet is stored in RxData[],  packet is checked for validity and RX is eventually synchronized with TX.
          if OK, we prepare an eventual freq hop and decode the frame decodeSX1280Packet()
          then exit
    - check if a timeout occured
        - if we were connected,
            - if timeout occurs when we where in a slot for downlink , go back to transmit mode
            - if to many missing packets, connection is lost and we start listening on first channel (faster reconnection)
            - perform a frequency hop if required
        - if we were not connected, perform a frequency hop

incomming RcData frame
- data from SX1280 are read in copied to RxData[] in main loop()
- data from  RxData[] are processed in decodeSX1280Packet() for different types of frame.
- for a RcData, it call saveRcFrame()
- saveRcFrame() put the values into
        ServoData[] (for PWM)
        channel[] (for Sbus)
        MiLoStorage.FS_data[] (when it is a failsafe frame)

incoming uplink telemetry frame:
- data from SX1280 are read in copied to RxData[] in main loop()
- data from  RxData[] are processed in decodeSX1280Packet() for different types of frame.
- for uplink tlm packet, it call sportMSPstuff() 
- sportMSPstuff() process the 8 bytes of RxData and put them into a circular array sportMspData[] increasing sportMspHead and sportMspCount
   Note: sportMspData[] is a circular array (containing "len" and "frame") controlled by sportMspTail, sportMspHead and sportMspCount
   Note: sportMSPstuff() add START (0X7E), stuffing + CRC and so the data are in the sport expected format.

- at regular interval (12msec), a timer1 fire and call SportPollISR() that set a flag sportPollIsrFlag
- main loop call handleSportPoll()  when sportPollIsrFlag is set 
- handleSportPoll() , if sportMspCount > 0 (= tlm data to send), 
       copies sportMspData[] into sTxData[] , increases sportMspTail and decreases sportMspCount 
       call sendMSPpacket(len)
- sendMSPpacket() sent sTxDAta[] to the sensor via UART and interrupts
   

Polling sport data (to be sent in a downlink tlm packet)
- at regular interval (12msec), a timer1 fire and call SportPollISR() that set a flag sportPollIsrFlag
- main loop call handleSportPoll()  when sportPollIsrFlag is set 
- handleSportPoll() , if sportMspCount==0 (no uplink tlm frame to send),  call tx_sport_poll()
- tx_sport_poll() fill sTxData[] with 2 bytes for a polling request and output the 2 bytes on the Sport port (UART)
- after sending the 2 bytes, Sport bus listen for a reply from sensor
Handling the Sport data received from sensor
- normally a sensor replies on the sport port and data are accumulated in sportbuff[[]
      With #define DEBUG_SIM_SPORT_SENSOR , we can simulate incomming data from sensor (filling Sxdata[] and calling ProcesssportData()). 
- main loop, call callSportSwSerial() 
- callSportSwSerial() look if we have a full frame available (>= 8 bytes and more than 500usec since previous income)
- if so, copy sportbuff[[] to sRxData[] and call ProcessSportData()
- ProcessSportData() remove stuffing inside sRxData, check CRC and if ok call checkSimilarSport()
- checkSimilarSport() update or add an entry in circular buffer sportData[] with sRxData
      circular buffer is managed by sportTail, sportTailIfAck, sportHead and sportDatalen
      each entry in sportData has 8 bytes (PHID, PRIM, ID1, ID2, Val1, Val2, val3, Val4)
Sending downlink tlm frame
- In main loop, when slot is for a downlink tlm frame and Rx is connected, call MiloTlmSent()
- MiloTlmSent() call MiLoTlm_build_frame() and then will write to SX1280
- MiLoTlm_build_frame() fill frame[] with data from link quality and or data from circular buffer sportData[]
    MiLoTlm_build_frame() checks if previous frame has been confirmed by Tx (update sportTail accordingly)
                          performs some compression in order to be able to put 2 set of data in one packet. 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Checks on the sequences for dowlink messages:
-------------------TX--------------------------   ---------------------RX---------------------------
                                                   -create downlink tlm frame---MiLoTlm_build_frame()
                                                        - put downlinkTlmId in frame[0]
                                                        - if downlinkTlmId == dwnlnkTlmExpectedId
                                                                - advance data to send next entry in circular buffer
                                                                - dwnlnkTlmExpectedId = (downlinkTlmId +1) %3  // New line added !!!!!!!!!!!!!!!!
                                                        - else roll back to previously sent data
                                                        //- increase dwnlnkTlmExpectedId by 1 (put as comment)
- receive a downlink tlm frame in frsky_process_telemetry()                                                        
    - if buffer[0](2 bits)  == telemetry_counter
         - increase telemetry_counter
         - handle the frame
    - else discard

- create a frame - Milo_data_frame() or Milo_telemetry_frame()
    - fill the frame with telemetry_counter
                                                       
                                                     - receive a frame (data or uplink) in decodeSX1280Packet()
                                                           downlinkTlmId = bits 5..4 from message RxData[0] 
examples of Downlink sequence
                            telemetry counter     frame       downlinkTlmId  dwnlnkTlmExpectedId  data
    initial values                0                                   0            0
    Tx sent                       0                 0                 
    Rx reveives                                                       0
    Rx sent                                         0                 0            0->1           put first data
    Tx receives (0==0)           0->1 + handle data                  
    TX sent                                         1
    RX does not receive
    Rx sent  (0!=1)                                 0            roll back                        put initial data
    Tx receives (0!=1)  discard                              
    TX sent                                         1
    RX does not receive
    Rx sent  (0!=1)                                 0            roll back                        put initial data
    Tx receives (0!=1)  discard                              
    TX sent                                         1
    RX receives                                                     0->1            
    Rx sent  (1==1)                                 1                             1->2            put new data
    Tx does not received
    Tx sent                                         1
    Rx receives                                                     1->1
    Rx sent (1!=2)                                  1             rollback                        put previous data
    Tx receives (1==1)           1->2 + handle data                  


example with initial values that are not equal
                            telemetry counter     frame       downlinkTlmId  dwnlnkTlmExpectedId  data
    initial values                0                                   2            3
    Tx sent                       0                 0                 
    Rx reveives                                                       0
    Rx sent                                         0                 0            0->1           put first data
    Tx receives (0==0)           0->1 + handle data                  
    So from here, sequence are synchronized again.

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Checks on the sequences for uplink messages:
-------------------RX--------------------------     ---------------------TX---------------------------
                                                    -create uplink tlm frame---Milo_telemetry_frame()
                                                    This happens only when slot could be a uplink and SportCount>1    
                                                        - put uplinkTlmId in packet[3] bits 7..6
                                                        - if uplinkTlmId == expectedUplinkTlmId
                                                                - expectedUplinkTlmId = (uplinkTlmId +1) %3 
                                                        fill the data and increase SportTail by 8
- receive a uplinkTlm frame in decodeSX1280Packet()                                                        
    - if (RxData[3] & 0XC0) >> 6 ) == uplinkTlmId 
         - increase uplinkTlmId
         - handle the frame 
    - else discard
- create a downlink frame - MiLoTlm_build_frame()
    - fill frame[1] bits 1..0 with uplinkTlmId                                                  
                                                    - receive a frame (downlink) in frsky_process_telemetry()
                                                            uplinkTlmId = buffer[1] & 0X03
                                                            - if uplinkTlmId == expectedUplinkTlmId
                                                                remove one entry from circular buffer (if possible)
                                                                    SportToAck = SportTail
                                                                    if (SportCount > 0) sportCount-- 
                                                            - else roll back SportTail to SportToAck

example of uplink sequence
                            uplinkTlmId(Rx )     frame       uplinkTlmId(TX)      expectedUlnkTlmId(TX)  data   SportCount
    initial values                0                                   0            0                               0
    Tx does not sent uplink                                                                                        
    Handset provide Sport data                                                                                     1
    Tx sent uplink                                  0<<                           0->1           put first data 
    Rx receives (0==0)           0->1 + handle data                  
    RX sent                                         >>1
    TX does not receive
    Tx sent  (0!=1)                                 0<<          roll back                        put initial data
    Rx receives (0!=1)  discard (already handled)                              
    RX sent                                         >>1
    TX does not receive
    Tx sent  (0!=1)                                 0<<            roll back                        put initial data
    Rx receives (0!=1)  discard                              
    RX sent                                         >>1
    TX receives                                                     0->1                          move to next data  1->0           
    Tx does not sent uplink  (no data)                     
    Handset provide new Sport data                                                                                   0->1   
    Tx sent  (1==1)                                 1                             1->2            put new data
    Rx does not received
    Rx sent                                         1
    Tx receives (1!=2)                                               1->1                     rollback (tail = ToAck)
    Tx sent (1!=2)                                  1                                          put previous data
    Rx receives (1==1)           1->2 + handle data                  
    RX sent                                         2
    Handset provide new Sport data                                                                                   1->2
    TX receives (2==2)                                               1->2                         move to next data  2->1           
    Tx sent (2==2)                                  2                                          put new data
    

example of uplink with initial values that are not equal
                            uplinkTlmId(Rx )     frame       uplinkTlmId(TX)      expectedUlnkTlmId(TX)  data    SportCount
    initial values                0                                   2            3                              0
    Tx does not sent uplink (SportCount == 0)
    Rx does not receive (no uplink sent)
    Rx sent                                         0
    TX receives  (0!=3)                                            2->0                          rollback (nihil) 0->0                                                                    
    Handset provide Sport data                                                                                     1
    Tx sent uplink                                  0                             0->1           put first data 
    Rx receives (0==0)           0->1 + handle data                  
    RX sent                                         1
    TX does not receive
    Tx sent  (0!=1)                                 0            roll back                        put initial data
    Rx receives (0!=1)  discard (already handled)                              
    RX sent                                         1
    TX does not receive
    Tx sent  (0!=1)                                 0            roll back                        put initial data
    Rx receives (0!=1)  discard                              
    RX sent                                         1
    TX receives                                                     0->1                          move to next data  1->0           
        



*/