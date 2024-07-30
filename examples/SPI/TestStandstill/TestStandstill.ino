#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const uint8_t CHIP_SELECT_PIN = 10;
const uint8_t ENABLE_HARDWARE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 4000;

// driver constants
const uint8_t HOLD_CURRENT = 0;
const uint8_t HOLD_DELAY = 0;

// Instantiate TMC51X0
TMC51X0 stepper;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  tmc51x0::SPIParameters spi_parameters(spi, CHIP_SELECT_PIN);
  tmc5160.setupSPI(spi_parameters);
  stepper.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

  stepper.driver.enableStealthChop();
  stepper.driver.writeHoldCurrent(HOLD_CURRENT);
  stepper.driver.writeHoldDelay(HOLD_DELAY);
  stepper.driver.enable();

  delay(DELAY);
}

void loop()
{
  Serial.println("standstill mode = NORMAL");
  stepper.driver.writeStandstillMode(tmc51x0::Driver::NORMAL);
  delay(DELAY);

  Serial.println("standstill mode = FREEWHEELING");
  stepper.driver.writeStandstillMode(tmc51x0::Driver::FREEWHEELING);
  delay(DELAY);

  Serial.println("standstill mode = PASSIVE_BRAKING_LS");
  stepper.driver.writeStandstillMode(tmc51x0::Driver::PASSIVE_BRAKING_LS);
  delay(DELAY);

  Serial.println("standstill mode = PASSIVE_BRAKING_HS");
  stepper.driver.writeStandstillMode(tmc51x0::Driver::PASSIVE_BRAKING_HS);
  delay(DELAY);

  Serial.println("--------------------------");
}
