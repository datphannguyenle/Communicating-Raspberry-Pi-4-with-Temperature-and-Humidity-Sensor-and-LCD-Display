# Communicating Raspberry Pi 4 with DHT11 Sensor and LCD Display

## Authors

- TA: Le Phan Nguyen Dat
- Instructor: MEng. Truong Quang Phuc 

## Hardware
- Raspberry Pi 4 Model B
- DHT11 Temperature and Humidity Sensor
- 16x2 LCD with I2C module

## Connections

### DHT11 Sensor
| DHT11 Pin | Raspberry Pi Pin |
|-----------|-------------------|
| VCC       | 3.3V (Pin 1)      |
| GND       | GND (Pin 6)       |
| DATA      | GPIO 17 (Pin 11)  |

### 16x2 LCD with I2C Module
| I2C Module Pin | Raspberry Pi Pin        |
|----------------|--------------------------|
| VCC            | 5V (Pin 4)               |
| GND            | GND (Pin 9)              |
| SDA            | GPIO 2 (SDA, Pin 3)      |
| SCL            | GPIO 3 (SCL, Pin 5)      |

## Enable I2C Bus
1. Open the Raspberry Pi configuration tool:
    ```bash
    sudo raspi-config
    ```

2. Navigate to `Interfacing Options` using the arrow keys and press `Enter`.

3. Select `I2C` and press `Enter`.

4. When asked to enable I2C, select `Yes` and press `Enter`.

5. Exit the `raspi-config` tool by navigating to `Finish` and pressing `Enter`.

6. Reboot your Raspberry Pi to apply the changes:
    ```bash
    sudo reboot
    ```

## Install Python Libraries
1. Update the package lists:
    ```bash
    sudo apt-get update
    ```

2. Install the `python3-smbus` and `i2c-tools` packages:
    ```bash
    sudo apt-get install -y python3-smbus i2c-tools
    ```

3. Install the DHT11 library:
    ```bash
    sudo pip3 install Adafruit_DHT
    ```

## Create and Run Code
1. Create a Python file using SCP or nano:
    ```bash
    nano dht11_lcd.py
    ```

2. Copy the following code into the file:
    ```python
    import Adafruit_DHT
    import RPi.GPIO as GPIO
    import time
    import smbus
    import sys

    # DHT11 Sensor Settings
    DHT_SENSOR = Adafruit_DHT.DHT11
    DHT_PIN = 17

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    # I2C parameters
    I2C_ADDR = 0x27  # I2C device address, if any error, check your device address
    LCD_WIDTH = 16   # Maximum characters per line

    # LCD constants
    LCD_CHR = 1  # Mode - Sending data
    LCD_CMD = 0  # Mode - Sending command
    LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
    LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line
    LCD_BACKLIGHT = 0x08  # On

    ENABLE = 0b00000100  # Enable bit

    # Timing constants
    E_PULSE = 0.0005
    E_DELAY = 0.0005

    # Open I2C interface
    try:
        bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
    except Exception as e:
        print(f"Error initializing I2C bus: {e}")
        sys.exit(1)

    def lcd_init():
        lcd_byte(0x33, LCD_CMD)  # 110011 Initialise
        lcd_byte(0x32, LCD_CMD)  # 110010 Initialise
        lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
        lcd_byte(0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
        lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
        lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
        time.sleep(E_DELAY)

    def lcd_byte(bits, mode):
        bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
        bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

        # High bits
        bus.write_byte(I2C_ADDR, bits_high)
        lcd_toggle_enable(bits_high)

        # Low bits
        bus.write_byte(I2C_ADDR, bits_low)
        lcd_toggle_enable(bits_low)

    def lcd_toggle_enable(bits):
        time.sleep(E_DELAY)
        bus.write_byte(I2C_ADDR, (bits | ENABLE))
        time.sleep(E_PULSE)
        bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
        time.sleep(E_DELAY)

    def lcd_string(message, line):
        message = message.ljust(LCD_WIDTH, " ")

        lcd_byte(line, LCD_CMD)

        for i in range(LCD_WIDTH):
            lcd_byte(ord(message[i]), LCD_CHR)

    def lcd_clear():
        lcd_byte(0x01, LCD_CMD)

    if __name__ == '__main__':
        try:
            lcd_init()
            while True:
                humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
                if humidity is not None and temperature is not None:
                    print(f"Temp={temperature:.1f}C  Humidity={humidity:.1f}%")
                    lcd_string(f"Temp:{temperature:.1f}C", LCD_LINE_1)
                    lcd_string(f"Hum:{humidity:.1f}%", LCD_LINE_2)
                else:
                    print("Failed to retrieve data from humidity sensor")
                time.sleep(2)

        except KeyboardInterrupt:
            print("Measurement stopped by User")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            lcd_clear()
            GPIO.cleanup()
    ```

3. Save and exit the file by pressing `Ctrl+X`, then `Y`, and `Enter`.

4. Make the file executable (optional but recommended):
    ```bash
    chmod +x dht11_lcd.py
    ```

5. Run the LCD program:
    ```bash
    sudo python3 dht11_lcd.py
    ```
