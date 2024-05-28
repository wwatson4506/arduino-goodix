/*
 *  License: Refer to LICENSE file in this directory.
 * Modified: For use with Teensy 4.x and MicroMod
 *       By: Warren Watson 2024.
*/
 
#include "Goodix.h"
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

// Interrupt handling
volatile uint8_t goodixIRQ = 0;

void  _goodix_irq_handler() {
  noInterrupts();
  goodixIRQ = 1;
  interrupts();
}

// Implementation
Goodix::Goodix(uint8_t interruptPin=2, uint8_t resetPin=255, uint8_t addr=GOODIX_I2C_ADDR_BA) {
    intPin = interruptPin;
    rstPin = resetPin;
    i2cAddr = addr; // Default 0x5D for Buydisplay ER-TFTM101-1 10.1" TFT with GT9271 CTS.
}

void Goodix::setHandler(void (*handler)(int8_t, GTPoint*)) {
  touchHandler = handler;
}

bool Goodix::begin() {
  _wire->setClock(400000);
  _wire->begin();
  bool result = reset();
  _wire->beginTransmission(i2cAddr);  
  int error = _wire->endTransmission();
  if (error != 0) {    
    return false;
  }

  return result;
}

bool Goodix::reset() {
  delay(1);

  pinMode(intPin, OUTPUT);
  digitalWrite(intPin, LOW);
 
  if(rstPin != 255) { 
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, LOW);
  }
  /* begin select I2C slave addr */

  /* T2: > 10ms */
  delay(11);

  /* HIGH: 0x28/0x29 (0x14 7bit), LOW: 0xBA/0xBB (0x5D 7bit) */
  digitalWrite(intPin, i2cAddr == GOODIX_I2C_ADDR_28);

  /* T3: > 100us */
  delayMicroseconds(110);
  pinMode(rstPin, INPUT);

  /* T4: > 5ms */
  delay(6);
  digitalWrite(intPin, LOW);
  /* end select I2C slave addr */

  /* T5: 50ms */
  delay(51);
  pinMode(intPin, INPUT_PULLUP); // INT pin has pullups so use.

  previousContacts = 0;
  for (uint8_t i = 0; i < GOODIX_MAX_CONTACTS*GOODIX_CONTACT_SIZE; i++) previousPoints[i] = 0;
    
  attachInterrupt(intPin, _goodix_irq_handler, RISING); // Active on rising edge.
  //  detachInterrupt(intPin, _goodix_irq_handler);

  return true;
}

/**
   Read goodix touchscreen version
   set 4 chars + zero productID to target
*/
uint8_t Goodix::productID(char *target) {
  uint8_t error;
  uint8_t buf[4];

  error = read(GOODIX_REG_ID, buf, 4);
  if (error) {
    return error;
  }

  memcpy(target, buf, 4);
  target[4] = 0;

  return 0;
}

/**
   goodix_i2c_test - I2C test function to check if the device answers.

   @client: the i2c client
*/
uint8_t Goodix::test() {
  uint8_t testByte;
  return read(GOODIX_REG_CONFIG_DATA,  &testByte, 1);
}

uint8_t Goodix::calcChecksum(uint8_t* buf, uint8_t len) {
  uint8_t ccsum = 0;
  for (uint8_t i = 0; i < len; i++) {
    ccsum += buf[i];
  }
  //ccsum %= 256;
  ccsum = (~ccsum) + 1;
  return ccsum;
}

uint8_t Goodix::readChecksum() {
  uint16_t aStart = GT_REG_CFG;
  uint16_t aStop = 0x80FE;
  uint8_t len = aStop - aStart + 1;
  uint8_t buf[len];

  read(aStart, buf, len);
  return calcChecksum(buf, len);
}

void Goodix::fwResolution(uint16_t maxX, uint16_t maxY) {
  uint8_t len = 0x8100 - GT_REG_CFG + 1;
  uint8_t cfg[len];
  read(GT_REG_CFG, cfg, len);

  cfg[1] = (maxX & 0xff);
  cfg[2] = (maxX >> 8);
  cfg[3] = (maxY & 0xff);
  cfg[4] = (maxY >> 8);
  cfg[len - 2] = calcChecksum(cfg, len - 2);
  cfg[len - 1] = 1;

  write(GT_REG_CFG, cfg, len);
}

GTConfig* Goodix::readConfig() {
  read(GT_REG_CFG, (uint8_t *) &config, sizeof(config));
  return &config;
}

GTInfo* Goodix::readInfo() {
  read(GT_REG_DATA, (uint8_t *) &info, sizeof(info));
  return &info;
}

uint8_t Goodix::readConfigVersion() {
    uint8_t version = 0;
    read(GT_REG_CFG, (uint8_t *) &version, sizeof(version));
    return version;
}

void Goodix::armIRQ() {
  attachInterrupt(intPin, _goodix_irq_handler, RISING);
}

void Goodix::onIRQ() {
  //uint8_t buf[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
  bool changed = false;
  int8_t contacts;

  contacts = readInput(points);
  if (contacts < 0) return;
    
  if (contacts != previousContacts) {
    changed = true;
    //Serial.println("Change no of contacts:" + String(contacts) + "==" + String(previousContacts));
    previousContacts = contacts;
  }
    
  if (contacts >= 0) {
      for (uint8_t i = 0; i < GOODIX_CONTACT_SIZE * contacts; i++) {
          //Serial.println(String(changed) + "-" + String(i) + ":" + String(points[i]) + "==" + String(previousPoints[i]));
          if (points[i] != previousPoints[i]) {
              previousPoints[i] = points[i];
              changed = true;
          }
      }
  }
  if (changed) touchHandler(contacts, (GTPoint *)points);
    
  write(GOODIX_READ_COORD_ADDR, 0);
  /*struct goodix_ts_data *ts = dev_id;

    goodix_process_events(ts);

    write(GOODIX_READ_COORD_ADDR, 0);
    //if (write(GOODIX_READ_COORD_ADDR, 0) < 0)
    //  dev_err(&ts->client->dev, "I2C write end_cmd error\n");

    return IRQ_HANDLED;
  */
}

void Goodix::loop() {
  noInterrupts();
  uint8_t irq = goodixIRQ;
  goodixIRQ = 0;
  interrupts();

  if (irq) {
    onIRQ();
  }
}
#define EAGAIN 100 // Try again error

int16_t Goodix::readInput(uint8_t *data) {
  int touch_num;
  int error;
    
  uint8_t regState[1];

  error = read(GOODIX_READ_COORD_ADDR, regState, 1);
  //log_printf("regState: %#06x\n", regState);

  if (error) {
    //dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
    return -error;
  }

  if (!(regState[0] & 0x80))
    return -EAGAIN;

  
  touch_num = regState[0] & 0x0f;
  //if (touch_num > ts->max_touch_num)
  //  return -EPROTO;

  //log_printf("touch num: %d\n", touch_num);

  if (touch_num > 0) {
    /*    data += 1 + GOODIX_CONTACT_SIZE;
        error = read(GOODIX_READ_COORD_ADDR + 1 + GOODIX_CONTACT_SIZE, data,
              GOODIX_CONTACT_SIZE * (touch_num - 1));
    */
    error = read(GOODIX_READ_COORD_ADDR + 1, data, GOODIX_CONTACT_SIZE * (touch_num));

    if (error)
      return -error;
  }

  return touch_num;
}

//----- Utils -----
void Goodix::i2cStart(uint16_t reg) {
  _wire->beginTransmission(i2cAddr);
  _wire->write(highByte(reg));
  _wire->write(lowByte(reg));
}

void Goodix::i2cRestart() {
  _wire->endTransmission(false);
  _wire->beginTransmission(i2cAddr);
}

uint8_t Goodix::i2cStop() {
  return _wire->endTransmission(true);
}

uint8_t Goodix::write(uint16_t reg, uint8_t *buf, size_t len) {
  uint8_t error;
  uint16_t startPos = 0;

  while (startPos < len) {
    i2cStart(reg + startPos);
    startPos += _wire->write(buf + startPos, len - startPos);
    error = _wire->endTransmission();
    if (error)
      return error;
  }
  return 0;
}

uint8_t Goodix::write(uint16_t reg, uint8_t buf) {
  i2cStart(reg);
  _wire->write(buf);
  return _wire->endTransmission();
}

uint8_t Goodix::read(uint16_t reg, uint8_t *buf, size_t len) {
  uint8_t res;

  i2cStart(reg);

  res = _wire->endTransmission(false);
  if (res != GOODIX_OK) {
    return res;
  }

  uint16_t pos = 0, prevPos = 0;
  //size_t readLen = 0;
  uint8_t maxErrs = 3;

  while (pos < len) {
    _wire->requestFrom(i2cAddr, (len - pos));

    prevPos = pos;
    while (_wire->available()) {
      buf[pos] = _wire->read();
      pos++;
    }

    if (prevPos == pos)
      maxErrs--;

    if (maxErrs <= 0) {
      break;
    }
    delay(0);
  }
  return 0;
}



