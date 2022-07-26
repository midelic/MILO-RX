# MILO-RX
New protocol based on SX128x LORA chip
-2.4Ghz Receiver-

This project is the receiver part of a new protocol  based on SX128x LORA RF chip.
Protocol general charcateristics:
1.Long range
2.Cheap HW(existing already)
2.16 full channels(11bits/channel).
3.Sport telemetry.
4.SBUS output on receiver.
5.Use of existing expresslrs hardware comercially available on the market.
6 Compatible with multiprotocol.

Ths receiver will bind with an  expresslrs hacked module attached to exiting TX OpenTX handsets(Taranis,Radiomaster,Jumper)
The TX counterpart will be compatible and be attached to multiprotocol main code.
For testing I used a 2.4G BETAFPV-500 expresslrs Tx module based on Esp32 chip.

Or with a DIY TX module based on ESP32.
The documents diagram and pinout are on my repo multiprotocol/docs/ESP32.
The hacked details of 2.4G BETAFPV -500 Tx module you will find on the same repo.

At the momebt the MILO Rx code is based on esp8285 target as most expresslrs receivers are.
When developing the code I used for test an expresslrs receiver using the same pinout(I used Flywoo EL24P in this particular case).
So in order to use this new protocol you have to buy and reflash an 2.4G expresslrs receiver.
