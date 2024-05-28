// GT9271_Teensy_Touch.ino
/*
 * License: Refer to LICENSE file in main directory.
 * Modified: For use with Teensy 4.x and MicroMod
 *       By: Warren Watson 2024.
*/

#include <Wire.h>
#include "Goodix.h"

#define INT_PIN 2
#define RST_PIN 255 // 255 means do not use reset pin else reset using pin number.

Goodix touch = Goodix(INT_PIN, RST_PIN, GOODIX_I2C_ADDR_BA);

// Callback from irq function.
void handleTouch(int8_t contacts, GTPoint *points) {
  Serial.printf("Number Of Contact Points: %d\n", contacts);
  for (uint8_t i = 0; i < contacts; i++) {
    // X and Y coords are inverted side to side and up and down.
    // Subtract from screen width and height to get x/y origins at
    // 0,0 instead of 1023,599. Also generate 16bit coords from two
    // 8bit coords (big endian).
    uint16_t x = (uint16_t)LCD_WIDTH - ((points[i].x_msb<<8) | points[i].x_lsb);
    uint16_t y = (uint16_t)LCD_HEIGHT - ((points[i].y_msb<<8) | points[i].y_lsb);
    uint16_t area = (uint16_t)((points[i].area_msb<<8) | points[i].area_lsb);
    Serial.printf("Contact #%d: Track ID %d, X position %d, Y position %d, area %d\n", i, points[i].trackId,x,y,area);
    yield();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("%cGoodix GT9172 touch driver for Teensy\n",12);
  
  // Setup callback.
  touch.setHandler(handleTouch);
  // We are using Wire2 on T41. Set this to your wire usage.
  touch.setWireObject(&Wire2); // Default set to Wire on reset or power up.
  // Start touchscreen.
  if (touch.begin()!=true) {
    Serial.println("! Module Initialize: Failed");
  } else {
    Serial.println("Module Initialize: Succeded");
  }
  Serial.printf("\nUp to 10 contact points can be setup for use with this driver.\n");
  Serial.printf("Currently it is setup for 5 in Goodix.h (GOODIX_CONTACT_SIZE 5).\n\n");
  Serial.printf("The screen is ready to be touched...\n");
}

void loop() {  
  touch.loop();
  delay(1);
}
