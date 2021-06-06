#include <mma8451.h>

static void mma8451_write_register_8(const mma8451_t *mma, uint8_t reg,
                                     uint8_t value) {
  while (HAL_I2C_Mem_Write(mma->hi2c, mma->i2caddr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1,
                           MMA8451_TIMEOUT_DELAY) != HAL_OK)
    ;
}

static uint8_t mma8451_read_register_8(const mma8451_t *mma, uint8_t reg) {
  uint8_t data;
  while (HAL_I2C_Mem_Read(mma->hi2c, mma->i2caddr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1,
                          MMA8451_TIMEOUT_DELAY) != HAL_OK)
    ;
  return data;
}

uint8_t mma8451_begin(mma8451_t *mma, I2C_HandleTypeDef *hi2c, uint8_t addr) {
  mma->i2caddr = addr;
  mma->hi2c = hi2c;

  /* Check connection */
  uint8_t deviceid = mma8451_read_register_8(mma, MMA8451_REG_WHOAMI);
  if (deviceid != 0x1A) {
    /* No MMA8451 detected ... return false */
    // Serial.println(deviceid, HEX);
    return 0;
  }

  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG2, 0x40); // reset

  while (mma8451_read_register_8(mma, MMA8451_REG_CTRL_REG2) & 0x40)
    ;

  // enable 4G range
  mma8451_write_register_8(mma, MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G);
  // High res
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG2, 0x02);
  // DRDY on INT1
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG4, 0x01);
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG5, 0x01);

  // Turn on orientation config
  mma8451_write_register_8(mma, MMA8451_REG_PL_CFG, 0x40);

  // Activate at max rate, low noise mode
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG1, 0x01 | 0x04);

  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
  }
  */

  return 1;
}

uint8_t mma8451_begin_default(mma8451_t *mma, I2C_HandleTypeDef *hi2c) {
  return mma8451_begin(mma, hi2c, MMA8451_DEFAULT_ADDRESS);
}

void mma8451_read(mma8451_t *mma) {
  // read x y z at once
  uint8_t data[6];
  HAL_I2C_Mem_Read(mma->hi2c, mma->i2caddr, MMA8451_REG_OUT_X_MSB, 1, data, 6,
                   MMA8451_TIMEOUT_DELAY);

  mma->x = data[0];
  mma->x <<= 8;
  mma->x |= data[1];
  mma->x >>= 2;
  mma->y = data[2];
  mma->y <<= 8;
  mma->y |= data[3];
  mma->y >>= 2;
  mma->z = data[4];
  mma->z <<= 8;
  mma->z |= data[5];
  mma->z >>= 2;

  uint8_t range = mma8451_get_range((const mma8451_t *)mma);
  uint16_t divider = 1;
  if (range == MMA8451_RANGE_8_G)
    divider = 1024;
  if (range == MMA8451_RANGE_4_G)
    divider = 2048;
  if (range == MMA8451_RANGE_2_G)
    divider = 4096;

  mma->x_g = (float)mma->x / divider;
  mma->y_g = (float)mma->y / divider;
  mma->z_g = (float)mma->z / divider;
}

void mma8451_set_range(const mma8451_t *mma, mma8451_range_t range) {
  uint8_t reg1 = mma8451_read_register_8(mma, MMA8451_REG_CTRL_REG1);
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG1, 0x00); // deactivate
  mma8451_write_register_8(mma, MMA8451_REG_XYZ_DATA_CFG, range & 0x3);
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG1, reg1 | 0x01); // activate
}

mma8451_range_t mma8451_get_range(const mma8451_t *mma) {
  /* Read the data format register to preserve bits */
  return (mma8451_range_t)(
      mma8451_read_register_8(mma, MMA8451_REG_XYZ_DATA_CFG) & 0x03);
}

void mma8451_set_data_rate(const mma8451_t *mma,
                           mma8451_data_rate_t data_rate) {
  uint8_t ctl1 = mma8451_read_register_8(mma, MMA8451_REG_CTRL_REG1);
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG1, 0x00); // deactivate
  ctl1 &= ~(MMA8451_DATARATE_MASK << 3);                      // mask off bits
  ctl1 |= (data_rate << 3);
  mma8451_write_register_8(mma, MMA8451_REG_CTRL_REG1, ctl1 | 0x01); // activate
}

mma8451_data_rate_t mma8451_get_data_rate(const mma8451_t *mma) {
  return (mma8451_data_rate_t)(
      (mma8451_read_register_8(mma, MMA8451_REG_CTRL_REG1) >> 3) &
      MMA8451_DATARATE_MASK);
}

uint8_t mma8451_get_orientation(const mma8451_t *mma) {
  return mma8451_read_register_8(mma, MMA8451_REG_PL_STATUS) & 0x07;
}
