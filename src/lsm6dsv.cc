#include "lsm6dsv-lib/lsm6dsv.h"

#include "lsm6dsv-lib/lsm6dsv_register.h"

namespace lsm6dsv_lib {

bool LSM6DSV::Init() {
  bool is_success = true;
  is_success &= WriteI3cEnabled(LSM6DSV::EnableState::kDisabled);
  is_success &= WriteAutoIncrementEnabled(LSM6DSV::EnableState::kEnabled);
  is_success &= EnableBlockDataUpdate(LSM6DSV::EnableState::kEnabled);
  is_success &= WriteTimestamp(LSM6DSV::EnableState::kEnabled);
  is_success &= EnableTimestampRounding(LSM6DSV::EnableState::kEnabled);
  return is_success;
}

bool LSM6DSV::ReadAccel(float* x, float* y, float* z) {
  return true;  // or false on failure
}

bool LSM6DSV::ReadGyro(float* x, float* y, float* z) {
  return true;  // or false on failure
}

bool LSM6DSV::WriteRegister(Register address, uint8_t* data) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  tx_buf[0] = static_cast<uint8_t>(address);
  tx_buf[1] = *data;

  if (enable_cs) { enable_cs(); }
  spi_transmit_and_receive(tx_buf, rx_buf, 2);
  if (disable_cs) { disable_cs(); }

  return true;
}

bool LSM6DSV::ReadRegister(Register address, uint8_t* data) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  tx_buf[0] = static_cast<uint8_t>(address) | 0b10000000;
  tx_buf[1] = 0;

  if (enable_cs) { enable_cs(); }
  spi_transmit_and_receive(tx_buf, rx_buf, 2);
  if (disable_cs) { disable_cs(); }

  *data = rx_buf[1];

  return true;
}

bool LSM6DSV::WriteRegisters(Register start_address, uint8_t* data,
                             size_t length) {
  uint8_t tx_buf[length + 1];
  uint8_t rx_buf[length + 1];

  tx_buf[0] = static_cast<uint8_t>(start_address);
  for (size_t i = 0; i < length; ++i) { tx_buf[i + 1] = data[i]; }

  if (enable_cs) { enable_cs(); }
  spi_transmit_and_receive(tx_buf, rx_buf, length + 1);
  if (disable_cs) { disable_cs(); }

  return true;
}

bool LSM6DSV::ReadRegisters(Register start_address, uint8_t* data,
                            size_t length) {
  uint8_t tx_buf[length + 1];
  uint8_t rx_buf[length + 1];
  tx_buf[0] = static_cast<uint8_t>(start_address) | 0b10000000;

  if (enable_cs) { enable_cs(); }
  spi_transmit_and_receive(tx_buf, rx_buf, length + 1);
  if (disable_cs) { disable_cs(); }

  for (size_t i = 0; i < length; ++i) { data[i] = rx_buf[i + 1]; }

  return true;
}

uint8_t LSM6DSV::WhoAmI() {
  uint8_t who_am_i;
  if (ReadRegister(Register::WHO_AM_I, &who_am_i)) {
    // Ideally 0x6B
    // std::cout << "Who Am I: 0x" << std::hex << static_cast<int>(who_am_i)
    //           << std::dec << std::endl;
    return who_am_i;
  } else {
    return 0;
  }
}

bool LSM6DSV::WriteI3cEnabled(EnableState enable_state) {
  uint8_t if_cfg;
  ReadRegister(Register::IF_CFG, &if_cfg);

  if (enable_state == EnableState::kEnabled) {
    if_cfg |= 0b00000001;  // Set the I3C_EN bit
  } else {
    if_cfg &= ~0b00000001;  // Clear the I3C_EN bit
  }

  return WriteRegister(Register::IF_CFG, &if_cfg);
}

bool LSM6DSV::WriteAutoIncrementEnabled(EnableState enable_state) {
  uint8_t ctrl3;
  ReadRegister(Register::CTRL3, &ctrl3);

  if (enable_state == EnableState::kEnabled) {
    ctrl3 |= 0b00000100;  // Set the IF_INC bit
  } else {
    ctrl3 &= ~0b00000100;  // Clear the IF_INC bit
  }

  return WriteRegister(Register::CTRL3, &ctrl3);
}

bool LSM6DSV::EnableBlockDataUpdate(EnableState enable_state) {
  uint8_t ctrl3;
  ReadRegister(Register::CTRL3, &ctrl3);

  if (enable_state == EnableState::kEnabled) {
    ctrl3 |= 0b01000000;  // Set the BDU bit
  } else {
    ctrl3 &= ~0b01000000;  // Clear the BDU bit
  }

  return WriteRegister(Register::CTRL3, &ctrl3);
}

bool LSM6DSV::WriteTimestamp(EnableState enable_state) {
  uint8_t functions_enable;
  ReadRegister(Register::FUNCTIONS_ENABLE, &functions_enable);

  if (enable_state == EnableState::kEnabled) {
    functions_enable |= 0b01000000;  // Set the TIMESTAMP_EN bit
  } else {
    functions_enable &= ~0b01000000;  // Clear the TIMESTAMP_EN bit
  }

  return WriteRegister(Register::FUNCTIONS_ENABLE, &functions_enable);
}

bool LSM6DSV::EnableTimestampRounding(EnableState enable_state) {
  uint8_t ctrl10;
  ReadRegister(Register::CTRL10, &ctrl10);

  if (enable_state == EnableState::kEnabled) {
    ctrl10 |= 0b00000100;  // Set the ROUNDING_STATUS bit
  } else {
    ctrl10 &= ~0b00000100;  // Clear the ROUNDING_STATUS bit
  }

  return WriteRegister(Register::CTRL10, &ctrl10);
}

bool LSM6DSV::WriteGyroDataRate(LSM6DSV::GyroDataRate data_rate) {
  uint8_t ctrl2;
  ReadRegister(Register::CTRL2, &ctrl2);

  // Clear the data rate bits
  ctrl2 &= ~0b00001111;

  // Set the new data rate
  ctrl2 |= static_cast<uint8_t>(data_rate) & 0b1111;

  return WriteRegister(Register::CTRL2, &ctrl2);
}

bool LSM6DSV::WriteGyroFullScale(LSM6DSV::GyroFullScale full_scale) {
  uint8_t ctrl6;
  ReadRegister(Register::CTRL6, &ctrl6);

  // Clear the full scale bits
  ctrl6 &= ~0b00001111;

  // Set the new full scale
  ctrl6 |= static_cast<uint8_t>(full_scale) & 0b1111;

  return WriteRegister(Register::CTRL6, &ctrl6);
}

LSM6DSV::GyroFullScale LSM6DSV::ReadGyroFullScale() {
  uint8_t ctrl6;
  ReadRegister(Register::CTRL6, &ctrl6);
  return static_cast<GyroFullScale>(ctrl6 & 0b00001111);
}

void LSM6DSV::ConfigureGyroSensitivity(GyroFullScale full_scale) {
  switch (full_scale) {
    case GyroFullScale::k125dps:
      gyro_sensitivity_ = 0.004375f;  // Sensitivity for 125 dps
      break;
    case GyroFullScale::k250dps:
      gyro_sensitivity_ = 0.00875f;  // Sensitivity for 250 dps
      break;
    case GyroFullScale::k500dps:
      gyro_sensitivity_ = 0.0175f;  // Sensitivity for 500 dps
      break;
    case GyroFullScale::k1000dps:
      gyro_sensitivity_ = 0.035f;  // Sensitivity for 1000 dps
      break;
    case GyroFullScale::k2000dps:
      gyro_sensitivity_ = 0.07f;  // Sensitivity for 2000 dps
      break;
    case GyroFullScale::k4000dps:
      gyro_sensitivity_ = 0.14f;  // Sensitivity for 4000 dps
      break;
    default:
      gyro_sensitivity_ = 0.0f;  // Invalid full scale
      break;
  }
}

bool LSM6DSV::WriteGyroOpMode(GyroOpMode op_mode) {
  uint8_t ctrl2;
  ReadRegister(Register::CTRL2, &ctrl2);

  // Clear the operation mode bits
  ctrl2 &= ~0b01110000;

  // Set the new operation mode
  ctrl2 |= (static_cast<uint8_t>(op_mode) & 0b111) << 4;

  return WriteRegister(Register::CTRL2, &ctrl2);
}

bool LSM6DSV::WriteAccDataRate(AccelDataRate data_rate) {
  uint8_t ctrl1;
  ReadRegister(Register::CTRL1, &ctrl1);

  // Clear the data rate bits
  ctrl1 &= ~0b00001111;

  // Set the new data rate
  ctrl1 |= static_cast<uint8_t>(data_rate) & 0b1111;

  return WriteRegister(Register::CTRL1, &ctrl1);
}

bool LSM6DSV::WriteAccFullScale(LSM6DSV::AccelFullScale full_scale) {
  uint8_t ctrl8;
  ReadRegister(Register::CTRL8, &ctrl8);

  // Clear the full scale bits
  ctrl8 &= ~0b00000011;

  // Set the new full scale
  ctrl8 |= static_cast<uint8_t>(full_scale) & 0b11;

  return WriteRegister(Register::CTRL8, &ctrl8);
}

LSM6DSV::AccelFullScale LSM6DSV::ReadAccFullScale() {
  uint8_t ctrl8;
  ReadRegister(Register::CTRL8, &ctrl8);
  return static_cast<AccelFullScale>(ctrl8 & 0b00000011);
}

void LSM6DSV::ConfigureAccSensitivity(AccelFullScale full_scale) {
  switch (full_scale) {
    case AccelFullScale::k2g:
      acc_sensitivity_ = 0.061f;  // Sensitivity for 2g
      break;
    case AccelFullScale::k4g:
      acc_sensitivity_ = 0.122f;  // Sensitivity for 4g
      break;
    case AccelFullScale::k8g:
      acc_sensitivity_ = 0.244f;  // Sensitivity for 8g
      break;
    case AccelFullScale::k16g:
      acc_sensitivity_ = 0.488f;  // Sensitivity for 16g
      break;
    default:
      acc_sensitivity_ = 0.0f;  // Invalid full scale
      break;
  }
}

bool LSM6DSV::WriteAccOpMode(AccelOpMode op_mode) {
  uint8_t ctrl1;
  ReadRegister(Register::CTRL1, &ctrl1);

  // Clear the operation mode bits
  ctrl1 &= ~0b01110000;

  // Set the new operation mode
  ctrl1 |= (static_cast<uint8_t>(op_mode) & 0b111) << 4;

  return WriteRegister(Register::CTRL1, &ctrl1);
}

void LSM6DSV::CreateReadAccAndGyroTx(uint8_t tx_buf*) {
  tx_buf[0] = static_cast<uint8_t>(Register::OUTX_L_G) | 0b10000000;
}

bool LSM6DSV::DecodeReadAccAndGyroRx(uint8_t* rx_buf, float& gyro_x,
                                     float& gyro_y, float& gyro_z, float& acc_x,
                                     float& acc_y, float& acc_z) {
  gyro_x =
      static_cast<int16_t>((rx_buf[1] << 8) | rx_buf[0]) * gyro_sensitivity_;
  gyro_y =
      static_cast<int16_t>((rx_buf[3] << 8) | rx_buf[2]) * gyro_sensitivity_;
  gyro_z =
      static_cast<int16_t>((rx_buf[5] << 8) | rx_buf[4]) * gyro_sensitivity_;

  acc_x = static_cast<int16_t>((rx_buf[7] << 8) | rx_buf[6]) * acc_sensitivity_;
  acc_y = static_cast<int16_t>((rx_buf[9] << 8) | rx_buf[8]) * acc_sensitivity_;
  acc_z =
      static_cast<int16_t>((rx_buf[11] << 8) | rx_buf[10]) * acc_sensitivity_;
}

bool LSM6DSV::ReadAccAndGyro(float& gyro_x, float& gyro_y, float& gyro_z,
                             float& acc_x, float& acc_y, float& acc_z) {
  uint8_t data[12];
  bool status = ReadRegisters(Register::OUTX_L_G, data, 12);
  if (!status) { return status; }
  // Combine the low and high bytes for gyro and accelerometer data
  DecodeReadAccAndGyroRx(data, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

  return true;
}

bool LSM6DSV::ResetMemory() {
  uint8_t ctrl3;
  ReadRegister(Register::CTRL3, &ctrl3);

  // Set the BOOT bit
  ctrl3 |= 0b1000000;

  // Write back to the CTRL3 register
  return WriteRegister(Register::CTRL3, &ctrl3);
}

bool LSM6DSV::RebootDevice() {
  uint8_t ctrl3;
  ReadRegister(Register::CTRL3, &ctrl3);

  // Set the SW_RESET bit
  ctrl3 |= 0b00000001;

  // Write back to the CTRL3 register
  return WriteRegister(Register::CTRL3, &ctrl3);
}

bool LSM6DSV::WriteInt1Enable(uint8_t int_enable_mask) {
  uint8_t reg;
  ReadRegister(Register::INT1_CTRL, &reg);
  reg |= int_enable_mask;
  return WriteRegister(Register::INT1_CTRL, &reg);
}

bool LSM6DSV::WriteInt2Enable(uint8_t int_enable_mask) {
  uint8_t reg;
  ReadRegister(Register::INT2_CTRL, &reg);
  reg |= int_enable_mask;
  return WriteRegister(Register::INT2_CTRL, &reg);
}

// bool LSM6DSV::WriteSflpEnabled(EnableState enable_state) {
//   uint8_t ctrl10;
//   ReadRegister(Register::CTRL10, &ctrl10);

//   if (enable_state == EnableState::kEnabled) {
//     ctrl10 |= 0b00001000;  // Set SLP_EN bit
//   } else {
//     ctrl10 &= ~0b00001000;  // Clear SLP_EN bit
//   }

//   return WriteRegister(Register::CTRL10, &ctrl10);
// }

// bool LSM6DSV::WriteSflpOutput(SflpOutput output) {
//   uint8_t ctrl10;
//   ReadRegister(Register::SFLP_ODR, &ctrl10);

//   ctrl10 &= ~0b00110000;  // Clear SLP_OUT_SEL bits
//   ctrl10 |= (static_cast<uint8_t>(output) & 0b11) << 4;

//   return WriteRegister(Register::SFLP_ODR, &ctrl10);
// }

// bool LSM6DSV::WriteSflpDataRate(SflpDataRate rate) {
//   uint8_t sflp_odr;
//   ReadRegister(Register::SFLP_ODR, &sflp_odr);

//   sflp_odr &= ~0b00111000;  // Clear SLP_ODR bits
//   sflp_odr |= (static_cast<uint8_t>(rate) & 0b111) << 3;

//   return WriteRegister(Register::SFLP_ODR, &sflp_odr);
// }

// bool LSM6DSV::ReadQuaternion(float& q0, float& q1, float& q2, float& q3) {
//   uint8_t data[16];
//   if (!ReadRegisters(Register::QUATERNION_W_L, data, 16)) {
//     return false;
//   }
//   int16_t qw = static_cast<int16_t>((data[1] << 8) | data[0]);
//   int16_t qx = static_cast<int16_t>((data[3] << 8) | data[2]);
//   int16_t qy = static_cast<int16_t>((data[5] << 8) | data[4]);
//   int16_t qz = static_cast<int16_t>((data[7] << 8) | data[6]);

//   // Scale factor depends on device settings, assume 2^-14 here
//   constexpr float scale = 1.0f / 16384.0f;

//   q0 = qw * scale;
//   q1 = qx * scale;
//   q2 = qy * scale;
//   q3 = qz * scale;

//   return true;
// }

// bool LSM6DSV::ReadGravity(float& gx, float& gy, float& gz) {
//   uint8_t data[6];
//   if (!ReadRegisters(Register::GRV_X_L, data, 6)) {
//     return false;
//   }
//   int16_t x = static_cast<int16_t>((data[1] << 8) | data[0]);
//   int16_t y = static_cast<int16_t>((data[3] << 8) | data[2]);
//   int16_t z = static_cast<int16_t>((data[5] << 8) | data[4]);

//   // Scale factor depends on device settings, assume 1/1000 as example
//   constexpr float scale = 1.0f / 1000.0f;

//   gx = x * scale;
//   gy = y * scale;
//   gz = z * scale;

//   return true;
// }

// bool LSM6DSV::ReadGyroBias(float& bx, float& by, float& bz) {
//   uint8_t data[6];
//   if (!ReadRegisters(Register::GY_BIAS_X_L, data, 6)) {
//     return false;
//   }
//   int16_t x = static_cast<int16_t>((data[1] << 8) | data[0]);
//   int16_t y = static_cast<int16_t>((data[3] << 8) | data[2]);
//   int16_t z = static_cast<int16_t>((data[5] << 8) | data[4]);

//   // Scale factor depends on device settings, assume 1/1000 as example
//   constexpr float scale = 1.0f / 1000.0f;

//   bx = x * scale;
//   by = y * scale;
//   bz = z * scale;

//   return true;
// }

}  // namespace lsm6dsv_lib