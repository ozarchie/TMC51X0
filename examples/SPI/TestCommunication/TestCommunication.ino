/*
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             ((uint32_t)digitalPinToPinName(pin))
#define IO_REG_TYPE                     uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, pin)          digitalReadFast((PinName)pin)
#define DIRECT_WRITE_LOW(base, pin)     digitalWriteFast((PinName)pin, LOW)
#define DIRECT_WRITE_HIGH(base, pin)    digitalWriteFast((PinName)pin, HIGH)
#define DIRECT_MODE_INPUT(base, pin)    pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0))
#define DIRECT_MODE_OUTPUT(base, pin)   pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0))

#define pin_size_t                      IO_REG_TYPE
#define DIRECT_PIN_READ(base, pin)      digitalRead(pin)
*/

#if defined(ARDUINO_ARCH_STM32)
#include "../../../../src/TMC51X0.hpp"
#include "../../../../src/EQ6IO.hpp" 
#else
#include "TMC51X0.hpp"
#endif

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#elif defined(ARDUINO_ARCH_STM32)
//             MOSI   MISO   SCLK   SSEL
//SPIClass SPI_1(PA_7,  PA_6,  PA_5,  PB_0);    // RA
//SPIClass SPI_2(PB_15, PB_14, PB_13, PB_12);   // DEC
SPIClass RA_SPI(RA_MOSI,  RA_MISO,  RA_SCK,  RA_CSN);    // RA
SPIClass DEC_SPI(DEC_MOSI, DEC_MISO, DEC_SCK, DEC_CSN);   // DEC
#define spi RA_SPI
#define spi2 DEC_SPI
#else
SPIClass & spi = SPI1;
#endif

// SPI Parameters
const uint32_t SPI_CLOCK_RATE = 1000000;
const pin_size_t RA_CHIP_SELECT_PIN = RA_CSN;
const pin_size_t DEC_CHIP_SELECT_PIN = DEC_CSN;

const pin_size_t ENABLE_HARDWARE_PIN = RADEC_EN;

#if defined(ARDUINO_ARCH_STM32)
//                       RX    TX
HardwareSerial  Serial1(PB7, PB6);
#endif

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;

bool enabled;

#define LED_BLUE    DEC_DIAG0
#define LED_GREEN   DEC_DIAG1

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial1.begin(SERIAL_BAUD_RATE);
  
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

//while(1){
  Serial.println("blue");
  Serial1.println("blue");
  digitalWrite(LED_BLUE, HIGH);
  delay(1000);
  digitalWrite(LED_BLUE, LOW);
  delay(1000);
  Serial.println("green");
  Serial1.println("green");
  digitalWrite(LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(LED_GREEN, LOW);
  delay(1000);
//}

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif

  RA_SPI.begin();
  tmc51x0::SpiParameters spi_parameters(RA_SPI, SPI_CLOCK_RATE, RA_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);
  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);
  enabled = false;
}

void loop()
{
  if (enabled)
  {
    tmc5160.driver.disable();
  }
  else
  {
    tmc5160.driver.enable();
  }
  enabled = not enabled;

  tmc5160.printer.readAndPrintIoin();

  delay(DELAY);
}
