# MILO-RX
New protocol based on SX128x LORA chip
**2.4Ghz Receiver**

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

Ths receiver will bind with an Tx ExpressLRS module attached to exiting Tx OpenTX handsets(Taranis,Radiomaster,Jumper)
The TX counterpart will be made compatible and be attached to the multiprotocol main code.
There are several posibili ties to have acces to an Milo/multiprotocol compatible TX module.

- First posibility - I used a hacked 2.4G BETAFPV - 500 expresslrs Tx module based on ESP32 chip.See below.
https://github.com/midelic/DIY-Multiprotocol-TX-Module/tree/ESP_COMMON/docs/ESP32

- Second  posibility is to make a DIY Tx module based on ESP32.
The diagram details and pinout are on my repo https://github.com/midelic/DIY-Multiprotocol-TX-Module/tree/ESP_COMMON/docs/ESP32

- Third posiblity is to use ExpressLRS RX as Tx module.Maybe it is better that way as as there is no need to hack any EXpressLRS tx module or make a DIY module.
The Rx has enough I/O pins to connect to Tx handset.More details about that you may find below.
https://www.rcgroups.com/forums/showpost.php?p=49637731&postcount=451

At the moment the MILO Rx code is based on ESP8285 target as most Expresslrs receivers are.
When developing the code I used for test an ExpressLRS receiver using the same pinout(I used Flywoo EL24P in this particular case).
So in order to use this new protocol you have to buy and reflash an 2.4G ExpressLRS receiver.
For now there is no DIY receiver.In the future I may make one if there is enough interest.A good candidate for DIY receiver I found a diversity module on Aliexpress
https://nl.aliexpress.com/item/1005004493252273.html?spm=a2g0o.productlist.0.0.60365557RBwXoW&algo_pvid=93500978-b51b-4ecb-868e-ee74ff17a57a&algo_exp_id=93500978-b51b-4ecb-868e-ee74ff17a57a-5&pdp_ext_f=%7B%22sku_id%22%3A%2212000029349714709%22%7D&pdp_npi=2%40dis%21EUR%2112.48%2111.85%21%21%213.56%21%21%402100bb5116617854190017455ea104%2112000029349714709%21sea&curPageLogUid=mPXKkaUXRYpz

## Community ##

Discussion thread at rcgroups: https://www.rcgroups.com/forums/showthread.php?4144003-New-2-4G-LORA-protocol

## Project Status ##

The project is work in progress,in testing, and there is more work and testing before it is completed.

Main operation mode:

- LORA modulation
- Frame rate 142 HZ (7ms)
- Data Rate ~76kb/s (-108dBm)
- Bw-812 ; SF6 ; CR - LI - 4/7 
- Preamble 12 symbols
- Fixed length packet format(implicit)->15 bytes
- Sport downlink telemetry rate (1:3)
- Sport uplink telemetry rate (1:6)

## Software Installation ##
First Install in ARduino IDE the ESP8266 arduino core and coresponding libraries.

Start Arduino and open the **Preferences** window.
Enter https://arduino.esp8266.com/stable/package_esp8266com_index.json into the File>Preferences>Additional Boards Manager URLs field of the Arduino IDE. You can add multiple URLs, separating them with commas.

Open **Boards Manager from Tools > Board menu** and install esp8266 platform (and don't forget to select your ESP8266 board from Tools > Board menu after installation).

Along with ESP8266 core  you need to install 2 more libraries **AsyncElegantOTA and  ESPAsyncWebserver and ESPAsyncTCP** like below :

For  AsyncElegantOTA  Go to Sketch > Include Library > Library Manager > Search for "AsyncElegantOTA" > Install

Close Arduino and re-open again.

For ESPAsyncWebserver and  ESPAsyncTCP you need to do install manually see below.

Click [AsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer/archive/refs/heads/master.zip)  to download the ESPAsyncWebServer library.

Click [ESPAsyncTCP](https://github.com/me-no-dev/ESPAsyncTCP/archive/refs/heads/master.zip)   to download the ESPAsyncTCP library.

Next open Arduino IDE Sketch/Include Library/Add Zip library...and select the .zip file).Do this for both libraries. Next close Arduino IDE and start again to re-load libraries automatically.

Before compiling uncomment the line coresponding to your ExpressLRS receiver in _config.h file.
- Project built in arduino IDE(version > 1.8.13) under ESP8266 arduino core ,select **Tools, ESP8266-cores**

       - Board "Generic ESP8285 module";
       - Builtin  Led "16";
       - Flash size "2M(FS:64KB ~ OTA 992KB)";
       - MMU: "16KB cache + 48 KB RAM(IRAM)"
       
## Flashing ##
- Serial ,connect USB-FTDI serial device TX,RX,5V,GND pins to  coresponding receiver pins(TX ->RX and RX->TX) and power the receiver on with button pressed .Release the button and upload the firmware.For flashing OTA you need to get a .bin file. For that press Sketch ,select **Export compiled Binary**.Browse to the location of the binary(.bin file) to get the file and  store it in an acessible folder.
- OTA via WiFi,select "WIFI-RX" mode from Tx handset screen in protocol menu, switch off TX handset .Power on the RX,next start the TX handset and  observe RX LED blinking fast. Find your Wifi network and see an AP with SSID name **"MiLo_RX"** introduce password **"milo_sx1280"**.It will open a captive portal on **"10.0.0.1"** adress.Inside you will browse to the firmware(you already stored before) and upload it.After uploading is completed restart the RX with the new firmware.

## Binding ##
- Connect Rx to power without starting the Tx handset.After 20 seconds Rx enters in bind mode automatically.This feature is available only at start and when RX is not bound with TX.
- Start Tx ,enter in protocol menu and start  bind process.

## Failsafe ##
At the moment it is implemented only Failsafe from Rx.
When Tx is bound with Rx, you position your sticks, pots(you can use the mixer menu) as you want the to be for failsafe( alternatively you can use a script) and press the Rx button. Rx Led will blink for several seconds. When Led blinking stopped the FS data will be saved in Rx memory.
Aditionally you have the option to resetting FS data to "NO PULSE" when pressing button, while Rx is not bound.
I planned to introduce also FS from Tx.It is not implemnted yet but it is intented to work the same as for FrSkyX(D16) protocol.

