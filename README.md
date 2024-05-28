# arduino-goodix
# New Branch created: "arduino-goodix-Teensy4x-MM".

Arduino goodix touch screen driver

## Now supports GT9271 on Teensy T4.x and Micromod.
   Tested on a BuyDisplay ER-TFTM101-1 10.1" TFT with GT9271 CTS.

# Changes:
- Update to 16bit for the GT9271.
- Fix "readInfo()" function.
- Add ablility to use choice of Wire, Wire1, Wire2, (MicroMod Wire3).
- Add ability to ignore reset signal if "resetPin" set to 255.
- Changed constructor to take interrupt, reset pins and I2C address
  as arguments. Defaults are INT_PIN=2, RST_PIN=255(ignored if 255)
  and I2C address=0x5D.
  

Prototype of arduino-based library for Goodix touchscreen driver chips (tested with GT9271)
Thanks to linux/android Goodix drivers developers for references
* https://github.com/goodix/gt1x_driver_generic
* https://github.com/hadess/gt9xx (Also You can found specs in this repo)

![Hardware](https://github.com/ploys/arduino-goodix/blob/master/img/lenovoTC.jpg)
