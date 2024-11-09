
#if defined(ARDUINO_ARCH_STM32)
#include "../../../../src/TMC51X0.hpp"
#else
#include <TMC51X0.hpp>
#endif

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#elif defined(ARDUINO_ARCH_STM32)
#include "../../../../src/EQ6IO.hpp"
//             MOSI   MISO   SCLK   SSEL
//SPIClass SPI_1(PA_7,  PA_6,  PA_5,  PB_0);    // RA
//SPIClass SPI_2(PB_15, PB_14, PB_13, PB_12);   // DEC
SPIClass RA_SPI(RA_MOSI,  RA_MISO,  RA_SCK,  RA_CSN);    // RA
SPIClass DEC_SPI(DEC_MOSI, DEC_MISO, DEC_SCK, DEC_CSN);   // DEC
#define spi RA_SPI
#else
SPIClass & spi = SPI1;
#endif

// SPI Parameters
const uint32_t SPI_CLOCK_RATE = 1000000;
const pin_size_t SPI1_CHIP_SELECT_PIN = RA_CSN;
const pin_size_t SPI2_CHIP_SELECT_PIN = DEC_CSN;

#if defined(ARDUINO_ARCH_STM32)
//                       RX    TX
HardwareSerial Serial1(PB_7, PB_6);
#endif

const pin_size_t ENABLE_VIO_PIN = PA_4;
const pin_size_t ENABLE_FAN_PIN = PA_9;

const pin_size_t LD_GREEN  = PA_9;
const pin_size_t LD_BLUE   = PA_8;
//LED_BUILTIN

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t DELAY = 4000;

// converter constants
// internal clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 12;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real position unit" in this example is one rotation of the motor shaft
constexpr int32_t MICROSTEPS_PER_REAL_POSITION_UNIT = 200 * 256; // 51200
const int32_t SECONDS_PER_REAL_VELOCITY_UNIT = 60;
// rotations/s -> rotations/min
// rotations/(s^2) -> (rotations/min)/s

// driver constants @ 48V VM
//   motor: LE-4118-048-02-02
//     rated current: 1.5A
//     200 steps/rev
//   driver:
//     0.15 current sense resistor
//     1.6 A RMS
const uint8_t GLOBAL_CURRENT_SCALAR = 255; // 128..255
const uint8_t RUN_CURRENT = 31; // 0..31
const uint8_t PWM_OFFSET = 4; // 0..255
const uint8_t PWM_GRADIENT = 42; // 0..255
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;
const uint16_t STEALTH_CHOP_THRESHOLD = 300; // rotations/min seems to be upper limit
const bool AUTOGRAD = true; // automatic gradient adaptation
const uint8_t PWM_REG = 4; // 1..15 -> slowest regulation..fastest regulation

// controller constants
const uint32_t MIN_TARGET_VELOCITY = 60;  // rotations/min
const uint32_t MAX_TARGET_VELOCITY = STEALTH_CHOP_THRESHOLD; // rotations/min
const uint32_t TARGET_VELOCITY_INC = 60;  // rotations/min
const uint32_t MAX_ACCELERATION = 60;  // (rotations/min)/s
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t target_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(ENABLE_VIO_PIN, OUTPUT);
  digitalWrite(ENABLE_VIO_PIN, HIGH);

  pinMode(ENABLE_FAN_PIN, OUTPUT);
  digitalWrite(ENABLE_FAN_PIN, HIGH);

  pinMode(LD_GREEN, OUTPUT);
  digitalWrite(LD_GREEN, HIGH);

  pinMode(LD_BLUE, OUTPUT);
  digitalWrite(LD_BLUE, HIGH);

  delay(LOOP_DELAY);

  tmc5160.printer.setup(Serial);

//#if defined(ARDUINO_ARCH_STM32)
//  SPI_1.begin(); 
//#else
  spi.begin();
//#endif

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif

  tmc51x0::SpiParameters spi_parameters(SPI_1, SPI_CLOCK_RATE, SPI1_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);

  tmc51x0::ConverterParameters converter_parameters =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_POSITION_UNIT,
      SECONDS_PER_REAL_VELOCITY_UNIT
    };
  tmc5160.converter.setup(converter_parameters);

  tmc5160.driver.writeGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR);
  tmc5160.driver.writeRunCurrent(RUN_CURRENT);
  tmc5160.driver.writePwmOffset(PWM_OFFSET);
  tmc5160.driver.writePwmGradient(PWM_GRADIENT);
  tmc5160.driver.writeMotorDirection(MOTOR_DIRECTION);

  tmc5160.driver.writeStealthChopThreshold(tmc5160.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  tmc5160.driver.enableStealthChop();

  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeRampMode(RAMP_MODE);

  tmc5160.driver.enable();

  tmc5160.controller.rampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  delay(LOOP_DELAY);

  tmc5160.driver.enableAutomaticCurrentControl(AUTOGRAD, PWM_REG);

  target_velocity = MIN_TARGET_VELOCITY;
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));
}

void loop()
{
  digitalWrite(LD_BLUE, HIGH);

  tmc5160.printer.getStoredAndPrintPwmconf();
  tmc5160.printer.readAndPrintPwmScale();
  tmc5160.printer.readAndPrintPwmAuto();

  uint8_t actual_current_scaling = tmc5160.driver.readActualCurrentScaling();
  Serial.print("actual_current_scaling: ");
  Serial.println(actual_current_scaling);
  Serial.println("--------------------------");

  Serial.print("acceleration (rotations per second per second): ");
  Serial.println(MAX_ACCELERATION);
  Serial.print("STEALTH_CHOP_THRESHOLD (rotations per minute): ");
  Serial.println(STEALTH_CHOP_THRESHOLD);
  Serial.print("target_velocity (rotations per minute): ");
  Serial.println(target_velocity);
  Serial.println("--------------------------");

  if (!tmc5160.controller.velocityReached())
  {
    Serial.println("Waiting to reach target velocity.");
    delay(LOOP_DELAY);
  }
  Serial.println("Target velocity reached!");
  Serial.println("--------------------------");

  Serial.println("--------------------------");
  
  digitalWrite(LD_BLUE, LOW);
  delay(DELAY);

  target_velocity += TARGET_VELOCITY_INC;
  if (target_velocity > MAX_TARGET_VELOCITY)
  {
    target_velocity = MIN_TARGET_VELOCITY;
  }
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));
}
