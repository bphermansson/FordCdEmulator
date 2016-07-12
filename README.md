# FordCdEmulator

This is a hardware device that emulates a Ford cd-changer. It's works with the Ford 6000CD radio (maybe more) and tricks the radio to think there's a cd-changer connected. This makes the radio enable it's aux in, thus makes it possible to connect external audio sources (the radio normally lacks aux in). The project is based on work by niou_ns, Andrew Hammond (Yampp), Krzysztof Pintscher and more. See http://www.instructables.com/id/Ford-CD-Emulator-Arduino-Mega/.

I've added a #define to make it possible to compile Arduino code with or without LCD support. The LCD is good for debugging but not necessary for a working product. I've also added support for I2C. When keys on the radio is pressed their codes are sent out on the I2C-bus, this can be received and used by an external media player (or a Raspberry Pi). 

 
To use Spi, I2c and Gpio as user:
/etc/udev/rules.d/raspberrypi.rules:

KERNEL=="spidev0.[0-9]*", GROUP="spi"
KERNEL=="gpiomem", GROUP="gpio"
KERNEL=="i2c-[1-9]*", GROUP="i2c", MODE="0666"

