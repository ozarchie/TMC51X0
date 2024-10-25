//#include <TMC51X0.hpp>
#include "../../../../src/TMC51X0.hpp"

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

// SPI Parameters
const uint32_t SPI_CLOCK_RATE = 1000000;
const pin_size_t SPI_CHIP_SELECT_PIN = 10;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t register_data;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc51x0::SpiParameters spi_parameters(spi, SPI_CLOCK_RATE, SPI_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);
}

void loop()
{
  for (uint8_t register_address=0; register_address < tmc51x0::Registers::ADDRESS_COUNT; ++register_address)
  {
    if ((tmc5160.registers.readable((tmc51x0::Registers::RegisterAddress)register_address)) && (tmc5160.registers.writeable((tmc51x0::Registers::RegisterAddress)register_address)))
    {
      register_data = tmc5160.registers.read((tmc51x0::Registers::RegisterAddress)register_address);
      if (register_data != 0)
      {
        Serial.print("register_address: 0x");
        Serial.print(register_address, HEX);
        Serial.print(", register_data: 0x");
        Serial.println(register_data, HEX);
      }
    }
  }
  Serial.println("-------------------------");
  delay(DELAY);
}
