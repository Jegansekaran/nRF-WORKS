#include <Wire.h>

#define VM511_ADDR   0x50

// Config registers
#define REG_SYS_FUN  0x03
#define REG_SYS_STA  0x20
#define REG_FS_FMIN  0x0F
#define REG_FS_FMAX  0x10
#define REG_FS_STEP  0x11
#define REG_EXS_TH   0x1D
#define REG_S_FRQ    0x23
#define REG_TEMP     0x29

// SYS_FUN command codes (from manual)
#define CMD_RESET          0x0001  // Reset/restart
#define CMD_FACTORY        0x0002  // Restore factory params
#define CMD_MEASURE_3      0x0013  // Single measurement, 3 readings
#define CMD_MEASURE_CLR_3  0x0033  // Clear history + 3 readings

// SYS_STA bits
#define STA_MEASURE_DONE   (1 << 4)  // bit4 = measurement completed
#define STA_LOW_QUALITY    (1 << 3)  // bit3 = low signal quality
#define STA_SWEEP_TIMEOUT  (1 << 6)  // bit6 = sweep timeout
#define STA_NO_COIL        (1 << 15) // bit15 = no coil detected

bool vm511_writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(VM511_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return (Wire.endTransmission(true) == 0);
}

uint16_t vm511_readRegister(uint8_t reg) {
  Wire.beginTransmission(VM511_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  if (Wire.requestFrom((uint8_t)VM511_ADDR, (uint8_t)2, (uint8_t)true) < 2) return 0xFFFF;
  return ((uint16_t)Wire.read() << 8) | Wire.read();
}

bool vm511_waitForMeasurement(uint32_t timeoutMs = 3000) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint16_t sta = vm511_readRegister(REG_SYS_STA);
    if (sta == 0xFFFF) return false;          // read error
    if (sta & STA_MEASURE_DONE) return true;  // done
    delay(50);
  }
  return false;  // timeout
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("=== VM511 Test ===");

  // Write sweep config
  vm511_writeRegister(REG_FS_FMIN, 600);   // 600 Hz lower limit
  vm511_writeRegister(REG_FS_FMAX, 1100);  // 1100 Hz upper limit
  vm511_writeRegister(REG_FS_STEP, 5);     // 5 Hz steps

  // EXS_TH: method=1 (signal amplitude average, recommended), threshold=50%
  vm511_writeRegister(REG_EXS_TH, (1 << 8) | 50);

  delay(300);

  // Verify config was actually saved
  uint16_t fmin  = vm511_readRegister(REG_FS_FMIN);
  uint16_t fmax  = vm511_readRegister(REG_FS_FMAX);
  uint16_t fstep = vm511_readRegister(REG_FS_STEP);
  uint16_t quality = vm511_readRegister(REG_EXS_TH);

  Serial.print("FS_FMIN : "); Serial.print(fmin);  Serial.println(fmin  == 600  ? " Hz [OK]" : " Hz [MISMATCH]");
  Serial.print("FS_FMAX : "); Serial.print(fmax);  Serial.println(fmax  == 1100 ? " Hz [OK]" : " Hz [MISMATCH]");
  Serial.print("FS_STEP : "); Serial.print(fstep); Serial.println(fstep == 5    ? " Hz [OK]" : " Hz [MISMATCH]");
  Serial.print("REG_EXS_TH : "); Serial.println(quality);
}

void loop() {
  // Trigger 3-reading single measurement (correct command)
  if (!vm511_writeRegister(REG_SYS_FUN, CMD_MEASURE_3)) {
    Serial.println("Trigger failed!");
    delay(2000);
    return;
  }

  // Wait for measurement complete via SYS_STA bit4 (not fixed delay)
  if (!vm511_waitForMeasurement(3000)) {
    Serial.println("Measurement timeout!");
    delay(2000);
    return;
  }

  // Check for errors in SYS_STA
  uint16_t sta = vm511_readRegister(REG_SYS_STA);
  if (sta & STA_NO_COIL)        Serial.println("Warning: No coil detected");
  if (sta & STA_SWEEP_TIMEOUT)  Serial.println("Warning: Sweep timeout - check FS_FMIN/FMAX range");
  if (sta & STA_LOW_QUALITY)    Serial.println("Warning: Low signal quality");

  uint16_t rawFreq = vm511_readRegister(REG_S_FRQ);
  uint16_t rawTemp = vm511_readRegister(REG_TEMP);

  if (rawFreq == 0xFFFF || rawTemp == 0xFFFF) {
    Serial.println("Read failed!");
    delay(2000);
    return;
  }

  if (rawFreq == 0) {
    Serial.println("No frequency - transitioning or sensor issue");
    delay(500);
    return;
  }

  // Unit: S_FRQ is 0.1 Hz, TEMP is 0.1 C
  float freq = rawFreq / 10.0;
  float temp = rawTemp / 10.0;

  Serial.print("Frequency  : "); Serial.print(freq, 1); Serial.println(" Hz");
  Serial.print("Temperature: "); Serial.print(temp, 1); Serial.println(" C");
  Serial.println("---");

  delay(2000);
}