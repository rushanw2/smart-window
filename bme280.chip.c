#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define BME280_I2C_ADDR 0x76
#define BME280_CHIP_ID 0x60

#define BME280_REG_CHIP_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_HUM_MSB 0xFD
#define BME280_REG_HUM_LSB 0xFE

#define BME280_SOFT_RESET_CMD 0xB6
#define BME280_TEMP_DEFAULT_C 22.0f

// Factory-style Bosch BME280 temperature calibration coefficients (0x88-0x8D)
#define BME280_DIG_T1 27504U
#define BME280_DIG_T2 26435
#define BME280_DIG_T3 (-1000)

typedef struct {
  uint32_t temp_attr;
  uint8_t regs[256];
  uint8_t reg_addr;
  bool expect_reg_addr;
} chip_state_t;

static void write_le16(uint8_t *regs, uint8_t addr, uint16_t val) {
  regs[addr] = (uint8_t)(val & 0xFFU);
  regs[(uint8_t)(addr + 1U)] = (uint8_t)((val >> 8) & 0xFFU);
}

static int32_t bme280_calc_t_fine(int32_t adc_T) {
  int32_t var1 = (int32_t)((adc_T / 8) - ((int32_t)BME280_DIG_T1 * 2));
  var1 = (var1 * ((int32_t)BME280_DIG_T2)) / 2048;
  int32_t var2 = (int32_t)((adc_T / 16) - ((int32_t)BME280_DIG_T1));
  var2 = (((var2 * var2) / 4096) * ((int32_t)BME280_DIG_T3)) / 16384;
  return var1 + var2;
}

static uint32_t bme280_encode_temp_adc(float temp_c) {
  if (temp_c < -10.0f) {
    temp_c = -10.0f;
  } else if (temp_c > 50.0f) {
    temp_c = 50.0f;
  }

  const int32_t target_centi =
      (int32_t)(temp_c * 100.0f + (temp_c >= 0.0f ? 0.5f : -0.5f));

  int32_t lo = 0;
  int32_t hi = 0xFFFFF;
  int32_t best_adc = 519888;
  int32_t best_err = 0x7FFFFFFF;

  while (lo <= hi) {
    const int32_t mid = lo + (hi - lo) / 2;
    const int32_t t_fine = bme280_calc_t_fine(mid);
    const int32_t t_centi = (t_fine * 5 + 128) / 256;
    const int32_t err =
        (t_centi >= target_centi) ? (t_centi - target_centi) : (target_centi - t_centi);

    if (err < best_err) {
      best_err = err;
      best_adc = mid;
    }

    if (t_centi < target_centi) {
      lo = mid + 1;
    } else if (t_centi > target_centi) {
      hi = mid - 1;
    } else {
      return (uint32_t)mid;
    }
  }

  return (uint32_t)best_adc;
}

static void bme280_update_measurements(chip_state_t *chip) {
  const float temp_c = attr_read_float(chip->temp_attr);
  const uint32_t adc_T = bme280_encode_temp_adc(temp_c);

  // Raw 20-bit temperature registers (0xFA-0xFC): MSB, LSB, XLSB[7:4]
  chip->regs[BME280_REG_TEMP_MSB] = (uint8_t)((adc_T >> 12) & 0xFFU);
  chip->regs[BME280_REG_TEMP_MSB + 1U] = (uint8_t)((adc_T >> 4) & 0xFFU);
  chip->regs[BME280_REG_TEMP_MSB + 2U] = (uint8_t)((adc_T << 4) & 0xF0U);

  // Fixed baseline raw pressure (~1013 hPa) and humidity (~50% RH)
  chip->regs[BME280_REG_PRESS_MSB] = 0x55U;
  chip->regs[BME280_REG_PRESS_MSB + 1U] = 0x00U;
  chip->regs[BME280_REG_PRESS_MSB + 2U] = 0x00U;

  chip->regs[BME280_REG_HUM_MSB] = 0x6CU;
  chip->regs[BME280_REG_HUM_LSB] = 0x00U;
}

static void bme280_reset_regs(chip_state_t *chip) {
  memset(chip->regs, 0, sizeof(chip->regs));

  // Temperature calibration coefficients (0x88-0x8D)
  write_le16(chip->regs, 0x88, BME280_DIG_T1);
  write_le16(chip->regs, 0x8A, (uint16_t)BME280_DIG_T2);
  write_le16(chip->regs, 0x8C, (uint16_t)BME280_DIG_T3);

  // Pressure calibration coefficients (0x8E-0x9F)
  write_le16(chip->regs, 0x8E, 36477U);
  write_le16(chip->regs, 0x90, (uint16_t)(int16_t)-10685);
  write_le16(chip->regs, 0x92, 3024U);
  write_le16(chip->regs, 0x94, 2855U);
  write_le16(chip->regs, 0x96, 140U);
  write_le16(chip->regs, 0x98, (uint16_t)(int16_t)-7);
  write_le16(chip->regs, 0x9A, 15500U);
  write_le16(chip->regs, 0x9C, (uint16_t)(int16_t)-14600);
  write_le16(chip->regs, 0x9E, 6000U);

  // Humidity calibration coefficients (0xA1, 0xE1-0xE7)
  chip->regs[0xA1] = 75U;
  write_le16(chip->regs, 0xE1, 365U);
  chip->regs[0xE3] = 0U;
  // dig_H4 = 307 (0x133 -> 0x13 << 4 | 0x03), dig_H5 = 50 (0x032 -> 0x03 << 4 | 0x02)
  chip->regs[0xE4] = 0x13U;
  chip->regs[0xE5] = 0x23U;
  chip->regs[0xE6] = 0x03U;
  chip->regs[0xE7] = 30U;

  // Chip ID and status
  chip->regs[BME280_REG_CHIP_ID] = BME280_CHIP_ID;
  chip->regs[BME280_REG_STATUS] = 0x00U;

  bme280_update_measurements(chip);
}

static bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (address != BME280_I2C_ADDR) {
    return false;
  }

  if (!read) {
    chip->expect_reg_addr = true;
  } else if (chip->reg_addr >= BME280_REG_PRESS_MSB &&
             chip->reg_addr <= BME280_REG_HUM_LSB) {
    bme280_update_measurements(chip);
  }
  return true;
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->reg_addr == BME280_REG_TEMP_MSB) {
    bme280_update_measurements(chip);
  }
  const uint8_t val = chip->regs[chip->reg_addr];
  chip->reg_addr++;
  return val;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->expect_reg_addr) {
    chip->reg_addr = data;
    chip->expect_reg_addr = false;
  } else {
    if (chip->reg_addr == BME280_REG_RESET && data == BME280_SOFT_RESET_CMD) {
      bme280_reset_regs(chip);
    } else if (chip->reg_addr == BME280_REG_CTRL_HUM ||
               chip->reg_addr == BME280_REG_CTRL_MEAS ||
               chip->reg_addr == BME280_REG_CONFIG) {
      chip->regs[chip->reg_addr] = data;
    }
    chip->reg_addr++;
  }
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  (void)user_data;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->temp_attr = attr_init_float("temperature", BME280_TEMP_DEFAULT_C);
  chip->reg_addr = 0;
  chip->expect_reg_addr = false;

  bme280_reset_regs(chip);

  const i2c_config_t i2c_config = {
      .address = BME280_I2C_ADDR,
      .scl = pin_init("SCL", INPUT),
      .sda = pin_init("SDA", INPUT),
      .connect = on_i2c_connect,
      .read = on_i2c_read,
      .write = on_i2c_write,
      .disconnect = on_i2c_disconnect,
      .user_data = chip,
  };
  i2c_init(&i2c_config);
}
