#pragma once

#include <cstddef>

#include "lsm6dsv-lib/lsm6dsv_register.h"

namespace lsm6dsv_lib {
class LSM6DSV {
 public:
  LSM6DSV(void (*spi_transmit_and_receive)(uint8_t* tx_buf, uint8_t* rx_buf,
                                           size_t length),
          void (*enable_cs)(void) = nullptr, void (*disable_cs)(void) = nullptr)
      : enable_cs(enable_cs),
        disable_cs(disable_cs),
        spi_transmit_and_receive(spi_transmit_and_receive) {};
  ~LSM6DSV() = default;

  /**
   * @brief Initializes the LSM6DSV sensor.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init();

  /**
   * @brief Read accelerometer data.
   *
   * @param x Pointer to store the x-axis acceleration.
   * @param y Pointer to store the y-axis acceleration.
   * @param z Pointer to store the z-axis acceleration.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadAccel(float* x, float* y, float* z);

  /**
   * @brief Read gyroscope data.
   *
   * @param x Pointer to store the x-axis gyroscope data.
   * @param y Pointer to store the y-axis gyroscope data.
   * @param z Pointer to store the z-axis gyroscope data.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadGyro(float* x, float* y, float* z);

  /**
   * @brief Write data to one specific register.
   *
   * @param address The register address to write to.
   * @param data Pointer to the data to write.
   * @return true if writing is successful, false if writing fails.
   */
  bool WriteRegister(Register address, uint8_t* data);

  /**
   * @brief Read data from one specific register.
   *
   * @param address The register address to read from.
   * @param data Pointer to store the read data.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadRegister(Register address, uint8_t* data);

  /**
   * @brief Write data to multiple registers starting from a specific address.
   *
   * @param start_address The register address to start writing from.
   * @param data Pointer to the data to write.
   * @param length The number of registers to write.
   * @return true if writing is successful, false if writing fails.
   */
  bool WriteRegisters(Register start_address, uint8_t* data, size_t length);

  /**
   * @brief Read data from multiple registers starting from a specific address.
   *
   * @param start_address The register address to start reading from.
   * @param data Pointer to store the read data.
   * @param length The number of registers to read.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadRegisters(Register start_address, uint8_t* data, size_t length);

  /**
   * @brief Read the WHO_AM_I register to verify the device identity.
   *
   * @return The value of the WHO_AM_I register.
   */
  uint8_t WhoAmI();

  enum class EnableState : bool {
    kEnabled = true,
    kDisabled = false,
  };

  /**
   * @brief Write the I3C enabled state to the register.
   *        When using SPI or I2c, this should be set to kDisabled.
   *
   * @param enable_state The state to enable or disable I3C.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteI3cEnabled(EnableState enable_state);

  /**
   * @brief Write the auto-increment enabled state to the register.
   *        kEnabled for reading or writing multiple registers in a row,
   *
   * @param enable_state The state to enable or disable auto-increment.
   * @return true if the operation is successful.
   * @return false if the operation fails.
   */
  bool WriteAutoIncrementEnabled(EnableState enable_state);

  enum class GyroDataRate : uint8_t {
    kPowerDown = 0b0000,
    k7_5Hz = 0b0010,
    k15Hz = 0b0011,
    k30Hz = 0b0100,
    k60Hz = 0b0101,
    k120Hz = 0b0110,
    k240Hz = 0b0111,
    k480Hz = 0b1000,
    k960Hz = 0b1001,
    k1920Hz = 0b1010,
    k3840Hz = 0b1011,
    k7680Hz = 0b1100,
  };

  /**
   * @brief Write the gyroscope data rate to the register.
   *
   * @param data_rate The desired gyroscope data rate.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteGyroDataRate(GyroDataRate data_rate);

  enum class GyroFullScale : uint8_t {
    k125dps = 0b0000,
    k250dps = 0b0001,
    k500dps = 0b0010,
    k1000dps = 0b0011,
    k2000dps = 0b0100,
    k4000dps = 0b1100,
  };

  /**
   * @brief Write the gyroscope full scale to the register.
   *
   * @param full_scale The desired gyroscope full scale.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteGyroFullScale(GyroFullScale full_scale);

  /**
   * @brief Read the gyroscope full scale from the register.
   *
   * @return The current gyroscope full scale setting.
   */
  GyroFullScale ReadGyroFullScale();

  /**
   * @brief Configure the gyroscope sensitivity based on the full scale setting.
   *
   * @param full_scale The full scale setting to configure sensitivity.
   */
  void ConfigureGyroSensitivity(GyroFullScale full_scale);


  enum class GyroOpMode : uint8_t {
    kHighPerformance = 0b000,
    kHighAccuacyOdr = 0b001,
    kOdrTriggered = 0b011,
    kSleep = 0b100,
    kLowPower = 0b101,
  };

  bool WriteGyroOpMode(GyroOpMode);

  enum class AccelDataRate : uint8_t {
    kPowerDown = 0b0000,
    k1_875Hz = 0b0001,
    k7_5Hz = 0b0010,
    k15Hz = 0b0011,
    k30Hz = 0b0100,
    k60Hz = 0b0101,
    k120Hz = 0b0110,
    k240Hz = 0b0111,
    k480Hz = 0b1000,
    k960Hz = 0b1001,
    k1920Hz = 0b1010,
    k3840Hz = 0b1011,
    k7680Hz = 0b1100,
  };

  /**
   * @brief Write the accelerometer data rate to the register.
   *
   * @param data_rate The desired accelerometer data rate.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteAccDataRate(AccelDataRate data_rate);

  enum class AccelFullScale : uint8_t {
    k2g = 0b00,
    k4g = 0b01,
    k8g = 0b10,
    k16g = 0b11,
  };

  /**
   * @brief Write the accelerometer full scale to the register.
   *
   * @param full_scale The desired accelerometer full scale.
   * @return true if the operation is successful.
   * @return false if the operation fails.
   */
  bool WriteAccFullScale(AccelFullScale full_scale);

  /**
   * @brief Read the accelerometer full scale from the register.
   *
   * @return The current accelerometer full scale setting.
   */
  AccelFullScale ReadAccFullScale();

  /**
   * @brief Configure the accelerometer sensitivity based on the full scale
   *        setting.
   *
   * @param full_scale The full scale setting to configure sensitivity.
   */
  void ConfigureAccSensitivity(AccelFullScale full_scale);

  enum class AccelOpMode : uint8_t {
    kHighPerformance = 0b000,
    kHighAccuracy = 0b001,
    kOdrTriggered = 0b011,
    kLowPower2Mean = 0b100,
    kLowPower4Mean = 0b101,
    kLowPower8Mean = 0b110,
    kNormal = 0b111,
  };

  bool WriteAccOpMode(AccelOpMode op_mode);

  void CreateReadAccAndGyroTx(uint8_t tx_buf*);

  bool DecodeReadAccAndGyroRx(uint8_t* rx_buf, float& gyro_x, float& gyro_y,
                            float& gyro_z, float& acc_x, float& acc_y,
                            float& acc_z);

  /**
   * @brief Read accelerometer and gyroscope data.
   *
   * @param gyro_x pointer to store the x-axis gyroscope data.
   * @param gyro_y pointer to store the y-axis gyroscope data.
   * @param gyro_z pointer to store the z-axis gyroscope data.
   * @param acc_x pointer to store the x-axis acceleration data.
   * @param acc_y pointer to store the y-axis acceleration data.
   * @param acc_z pointer to store the z-axis acceleration data.
   * @return true if the operation is successful, false if operation fails.
   */
  bool ReadAccAndGyro(float& gyro_x, float& gyro_y, float& gyro_z, float& acc_x,
                      float& acc_y, float& acc_z);

  /**
   * @brief Reset the memory of the LSM6DSV device. All registers will be set to
   *        their default values. You sould wait for 15ms after this.
   * @return true if the reset is successful, false if it fails.
   */
  bool ResetMemory();

  /**
   * @brief Reboot the LSM6DSV device. You should wait for 50us after this.
   * @return true if the reboot is successful, false if it fails.
   */
  bool RebootDevice();

  // Interrupt enable configuration
  bool WriteInt1Enable(uint8_t value);
  bool WriteInt2Enable(uint8_t value);
  bool ReadInt1Enable(uint8_t& value);
  bool ReadInt2Enable(uint8_t& value);

  // SFLP (Sensor Fusion Low Power) enable
  bool WriteSflpEnable(EnableState enable_state);

  // SFLP output selection
  enum class SflpOutputSel : uint8_t {
    kGyro = 0,
    kAccel = 1,
    kQuaternion = 2,
    kGravity = 3,
  };
  bool WriteSflpOutputSel(SflpOutputSel sel);

  // SFLP data rate
  enum class SflpDataRate : uint8_t {
    k15Hz = 0b000,
    k30Hz = 0b001,
    k60Hz = 0b010,
    k120Hz = 0b011,
    k240Hz = 0b100,
    k480Hz = 0b101,
  };
  bool WriteSflpDataRate(SflpDataRate data_rate);

  // Read Quaternion data
  bool ReadQuaternion(float& w, float& x, float& y, float& z);

  // Read Gravity vector data
  bool ReadGravity(float& x, float& y, float& z);

  // Read Gyro Bias data
  bool ReadGyroBias(float& x, float& y, float& z);

  // Helper APIs to enable BDU, timestamp, rounding, and FIFO
  bool EnableBlockDataUpdate(EnableState enable_state);
  bool WriteTimestamp(EnableState enable_state);
  bool EnableTimestampRounding(EnableState enable_state);

  // bool WriteIntEnable(uint8_t int_enable_mask);
  // bool WriteSflpEnabled(EnableState enable_state);
  // bool WriteSflpOutput(SflpOutput output);


 private:
  float acc_sensitivity_ = 0.0f;
  float gyro_sensitivity_ = 0.0f;

  void (*enable_cs)(void);
  void (*disable_cs)(void);
  void (*spi_transmit_and_receive)(uint8_t* tx_buf, uint8_t* rx_buf,
                                   size_t length);
};

}  // namespace lsm6dsv_lib
