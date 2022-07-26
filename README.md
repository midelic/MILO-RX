# MILO-RX
New protocol based on SX128x LORA chip
-2.4Ghz Receiver-

This project is the receiver part of a new protocol  based on SX128x LORA RF chip.
Protocol general characteristics:
- Long range
- Cheap HW(existing already)
- 16 full channels(11bits/channel).
- Sport telemetry.
- SBUS output on receiver.
- Use of existing expresslrs hardware comercially available on the market.
- Compatible with multiprotocol.

Ths receiver will bind with an Tx expresslrs hacked module attached to exiting TX OpenTX handsets(Taranis,Radiomaster,Jumper)
The TX counterpart will be made compatible and be attached to multiprotocol main code.
For testing I used a 2.4G BETAFPV - 500 expresslrs Tx module based on Esp32 chip.

Or with a DIY TX module based on ESP32.
The documents diagram and pinout are on my repo multiprotocol/docs/ESP32.
The hacked details of 2.4G BETAFPV-500 Tx module you will find on the same repo.

At the moment the MILO Rx code is based on esp8285 target as most expresslrs receivers are.
When developing the code I used for test an expresslrs receiver using the same pinout(I used Flywoo EL24P in this particular case).
So in order to use this new protocol you have to buy and reflash an 2.4G expresslrs receiver.


## Project Status ##

The project is work in progress,in testing, and there is still a lot to go before it is completed.
Main operation mode:
- 142 Hz frame rate(7ms)
- data Rate ~76kb/s(-108dBm)
- Bw-812; SF6 ; CR -LI -4/7 .
- preamble 12 symbols
- Fixed length packet format(implicit) -15 bytes
- downlink telemetry rate(1:3)
- uplink telemetry rate (1:6)
