// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#include <TMC51X0.hpp>

using namespace tmc51x0;

void TMC51X0::setupSpi(SpiParameters spi_parameters)
{
  interface_spi_.setup(spi_parameters);
  registers.initialize(interface_spi_);
  initialize();
}

void TMC51X0::setupUart(UartParameters uart_parameters)
{
  interface_uart_.setup(uart_parameters);
  registers.initialize(interface_uart_);
  initialize();
}

uint8_t TMC51X0::readVersion()
{
  Registers::Ioin ioin;
  ioin.bytes = registers.read(Registers::IOIN);
  return ioin.version;
}

// private
void TMC51X0::initialize()
{
  driver.initialize(registers, converter);
  controller.initialize(registers, converter);
  encoder.initialize(registers, converter);
  printer.initialize(registers);
}
