testREADME


Do the following first

Enable:

I2C1

PB8 → SCL

PB9 → SDA

USART2

PA2 → TX

PA3 → RX

115200 baud recommended

SYS → Debug → Serial Wire (optional, but recommended)

Clock: use default 84 MHz (F411).





When the STM32 boots it should do the following:

1) It prints: (LCD test)

==== FULL SYSTEM TEST START ====


2️) It scans the I²C bus: (OLED and INA test)

Found device at 0x3C   (OLED)
Found device at 0x40   (INA219 #1)
Found device at 0x41   (INA219 #2)


3️) It initializes both INA219s.

4️ )It initializes the OLED and draws a white bar.

5️) It sends an AT command to the ESP-01S. (wi-fi emitter)

6️) It enters a loop printing:

INA1: 5.01 V  0.432 A  2.17 W | INA2: 12.03 V  0.890 A  10.69 W


*







