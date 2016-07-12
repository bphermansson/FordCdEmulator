# FordCdEmulator

To use Spi, I2c and Gpio as user:
/etc/udev/rules.d/raspberrypi.rules:

KERNEL=="spidev0.[0-9]*", GROUP="spi"
KERNEL=="gpiomem", GROUP="gpio"
KERNEL=="i2c-[1-9]*", GROUP="i2c", MODE="0666"

