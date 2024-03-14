#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adjust the address 0x27 depending on your display

void setup() {
  lcd.init();                      // Initialize the lcd
  lcd.backlight();
  Serial.begin(9600);              // Start serial communication at 9600 baud rate
}

void loop() {
  if (Serial.available()) {         // Check if data is available to read
    lcd.clear();                    // Clear the screen
    while (Serial.available()) {    // While there is data to read
      lcd.write(Serial.read());     // Read a byte and write it to the LCD
    }
  }
}
