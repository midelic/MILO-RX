# MILO-RX
New protocol based on SX128x LORA chip
-2.4Ghz Receiver-

This project is the receiver part of a new protocol  based on SX128x LORA RF chip.
## General characteristics ##
- Long range
- Cheap HW(existing already)
- 16 full channels(11bits/channel).
- Sport telemetry.
- SBUS output on receiver.
- Use of existing ExpressLRS hardware comercially available on the market.
- Compatible with multiprotocol.
- WiFi OTA update via webserver(config server)

Ths receiver will bind with an Tx ExpressLRS hacked module attached to exiting TX OpenTX handsets(Taranis,Radiomaster,Jumper)
The TX counterpart will be made compatible and be attached to multiprotocol main code.
For testing I used a 2.4G BETAFPV - 500 expresslrs Tx module based on Esp32 chip.

Another posibility is to make a DIY TX module based on ESP32.
The diagram details and pinout are on my repo https://github.com/midelic/DIY-Multiprotocol-TX-Module/tree/ESP-32/docs/ESP32
The hacked details of 2.4G BETAFPV-500 Tx module you will find on the same repo.

At the moment the MILO Rx code is based on ESP8285 target as most Expresslrs receivers are.
When developing the code I used for test an ExpressLRS receiver using the same pinout(I used Flywoo EL24P in this particular case).
So in order to use this new protocol you have to buy and reflash an 2.4G expresslrs receiver.

## Community ##

Discussion thread at rcgroups: https://www.rcgroups.com/forums/showthread.php?4144003-New-2-4G-LORA-protocol

## Project Status ##

The project is work in progress,in testing, and there is still a lot to go before it is completed.

Main operation mode:

- LORA modulation
- Frame rate 142 HZ(7ms)
- Data Rate ~76kb/s(-108dBm)
- Bw-812; SF6 ; CR -LI -4/7 
- Preamble 12 symbols
- Fixed length packet format(implicit) -15 bytes
- Sport downlink telemetry rate(1:3)
- Sport uplink telemetry rate (1:6)


## Software Installation  -Flashing ##
- Serial ,connect USB-FTDI serial device TX,RX,5V,GND pins to  coresponding rx pins(TX ->RX and RX->TX) and upload the firmware.
- OTA via WiFi,select "WIFI-RX" mode from TX handset screen in protocol menu.

## Binding ##
- Connect RX to power withput starting the Tx handset.After 30 seconds Rx enters in bind mode automatically.
- Start Tx ,enter in protocol menu and start  bind process.


