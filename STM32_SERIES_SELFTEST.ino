
/*    STM32 series digital power chip self-test program functions:
 *        1，I2C Slave 0x58 PMbus function setup
 *        2，UART serial port debug information              
 *        3，NTC temperature sensor resistor, output resistor divider detection, or current shunt ADC function   
 *        4，PWM control FAN speed or voltage adjustment
 *        5，DAC function, adjust voltage or current conversion voltage
 *        6，FAN speed TACH detection
 *        7，In-line read/write/erase SPI flash memory, including I2C EEPROM
 *        8，LED display, output connector signal voltage test  
 *  
 *   Modify the pin numbers corresponding to the chip's various function pins in the program. For specific chip information, please refer to:
 *           https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants
 *   To modify the temperature sensor:
 *     Thermistor_Fixed_R: Pull-up resistor value
 *     Sensor_Vref: ADC reference voltage
 *     Sensor_Rtop Sensor_Rbottom: Voltage divider resistor values
 *    
 *    For more information, please refer to:
 *      https://github.com/stm32duino/Arduino_Core_STM32
 *      
 *  Author Dafeng 2023
 */
#include <SPI.h>
#include <Wire.h>
// #include <IWatchdog.h>  //Hardware Watchdog Enable
#define SPIFLASH_IDREAD           0x9F
#define SPIFLASH_MACREAD          0x4B 
#define SPIFLASH_WRITEENABLE      0x06
#define SPIFLASH_BYTEPAGEPROGRAM  0x02 
#define SPIFLASH_ARRAYREADLOWFREQ 0x03
#define SPIFLASH_CHIPERASE        0x60
#define SPIFLASH_ARRAYREAD        0x0B
 
const uint8_t MOSI_PIN = XX;
const uint8_t MISO_PIN = XX;
const uint8_t SCLK_PIN = XX;
const uint8_t TX_DEBUG_PIN = XX;
//            MOSI  MISO  SCLK   SSEL
SPIClass SPI_3(MOSI_PIN, MISO_PIN, SCLK_PIN);
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
HardwareSerial Serial2(NC, TX_DEBUG_PIN);  

const uint8_t SPI_SELECT_PIN = XX;  
const uint8_t LED_AMBER_PIN = XX;  
const uint8_t LED_GREEN_PIN = XX;  
const uint8_t DAC_CONTROL_PIN = XX;  

const uint8_t FAN_PWM_PIN = XX;  
const uint8_t FAN_TACH_PIN = XX;
// const uint8_t FAN_EN_PIN = XX;  
const uint8_t TEMP_ADC_PIN = XX;  
const uint8_t VOLT_ADC_PIN = XX; 

const uint8_t fandiv = 2;  //This is the variable used to select the fan and it's the divider,Set 2 for bipole hall effect sensor

const float Thermistor_Fixed_R = XXXX;
const float Thermistor_R25 = 10000.0;
const float Thermistor_Beta =  3380.0;         //THERM_B_VALUE
const float Sensor_Vref = XXXX;
const float Sensor_Rtop = XXXX;
const float Sensor_Rbottom = XXXX;

uint32_t previousMillis = 0;
uint16_t j = 0;
uint16_t NbTopsFan = 0;
bool fanstatus = true;

volatile uint8_t  rx_command;
volatile uint8_t  rx_buff[2];
volatile bool rx_control_status = false;
volatile float fanspeed;
volatile float  voltage;
volatile float  temp;

void I2C_RxHandler(int numBytes)
{
  uint8_t RxByte;
   union
   {
     uint8_t b[2];
     uint16_t w;
   } rdata;
      rx_command = Wire.read();
      switch(rx_command){
          case 0x3B:
              rdata.b[0] = Wire.read();
              rdata.b[1] = Wire.read();
              if(rdata.w > 100) rdata.w = 4095;
              else rdata.w = map(rdata.w, 0, 100, 0, 4095); 
              analogWrite(FAN_PWM_PIN, rdata.w);  
          break;
          
          case 0xC0:
            rx_buff[0] = Wire.read();
            rx_buff[1] = Wire.read();
            rx_control_status = true;
          break;

          case 0xC1:
            rdata.b[0] = Wire.read();
            rdata.b[1] = Wire.read();
          break;

          default:
          while(Wire.available()) {RxByte = Wire.read();} 
          break;
      }
}

void I2C_TxHandler()
{
  union
   {
     uint8_t b[2];
     uint16_t w;
   } tdata;
  switch(rx_command){
      case 0:  
         Wire.write(0x00);
      break;

      case 0x88:
         tdata.w = float_to_lin11(voltage); 
         Wire.write(tdata.b[0]);
         Wire.write(tdata.b[1]); 
      break;

      case 0x8D:
         tdata.w = float_to_lin11(temp); 
         Wire.write(tdata.b[0]);
         Wire.write(tdata.b[1]); 
      break;

      case 0x90:
         tdata.w = float_to_lin11(fanspeed); 
         Wire.write(tdata.b[0]);
         Wire.write(tdata.b[1]); 
      break;

      default:
          for(int i = 0; i < 32; i++)  Wire.write(0xFF);
      break;
  }
}

void rpm ()
{ NbTopsFan++; }

void setup() {
 pinMode(SPI_SELECT_PIN, OUTPUT);
 digitalWrite(SPI_SELECT_PIN, HIGH);
 pinMode(LED_AMBER_PIN, OUTPUT);
 pinMode(LED_GREEN_PIN, OUTPUT);
 digitalWrite(LED_GREEN_PIN, LOW);
 digitalWrite(LED_AMBER_PIN, HIGH);
 // pinMode(FAN_EN_PIN, OUTPUT);
// digitalWrite(FAN_EN_PIN, LOW);
//  IWatchdog.begin(10000000);   // Init the watchdog timer with 10 seconds timeout
//  IWatchdog.reload();
//  Serial.setRx(PCx); 
//  Serial.setTx(PCx);
//  Serial.begin(38400);
 Serial2.begin(38400);
 Wire.begin(0x58); // Initialize I2C (Slave Mode: address=0x58)
 Wire.onReceive(I2C_RxHandler);
 Wire.onRequest(I2C_TxHandler);

 SPI_3.begin(); // Enable the SPI_3 instance with default SPISsettings
 analogWriteFrequency(25000); // Set FAN PMW control period to 25k Hz instead of default 1000
 analogWriteResolution(12);
 analogReadResolution(12);
 delay(50);

 Serial2.print(F("Initialising"));
  for (uint8_t i = 0; i < 6; ++i)
  {
    Serial2.print(F(". "));
    if(i%2 == 0){
      digitalWrite(LED_AMBER_PIN, HIGH);
      delay(260);
    }
    else { digitalWrite(LED_AMBER_PIN, LOW);
           delay(340); }
  }  
    attachInterrupt(FAN_TACH_PIN, rpm, FALLING);  // RISING or FALLING 
    analogWrite(FAN_PWM_PIN, 820);               //20% of fan speed 4096
    analogWrite(DAC_CONTROL_PIN, 410);
    delay(10);
}

void loop() {
  uint32_t currentMillis = millis();
    if(currentMillis - previousMillis >= 500)
    {
        previousMillis = currentMillis;
        if(j%2 == 0){
          if(fanstatus){
              NbTopsFan = 0;
              fanstatus = false;
          }
          else {
            fanspeed = float (NbTopsFan*60/fandiv);
            NbTopsFan = 0;
          }         
        }
      digitalWrite(LED_GREEN_PIN, !digitalRead(LED_GREEN_PIN));
      voltage = getVoltage(VOLT_ADC_PIN);
      temp = getTemp(TEMP_ADC_PIN);
      Serial2.printf("tempterature: %5.1f \n ", temp);
      Serial2.printf("Fan Speed: %d rpm \n", fanspeed);
      Serial2.printf("Voltage: %f \n ", voltage);
      Serial2.println(j);
      j++;
  }

  if(rx_control_status){
    rx_control_status = false;
    if(rx_buff[0] == 0 ) analogWrite(DAC_CONTROL_PIN, map(rx_buff[1], 0, 255, 0, 3500));
    else if(rx_buff[0] == 1) spi_flash_erase();
    else if(rx_buff[0] == 2) showpagedata(rx_buff[1]);
  }
}

float getTemp(uint8_t adcpin){
  float temp = float(analogRead(adcpin)) / 4095;
    // Calculate thermistor resistance
    // Fixed resistor is on the top of the voltage divider,
    // thermistor is at bottom.
    // (Vout/Vin) = Rt / (Rt + Rf)
    // ADC / 4095 = Vout/Vin = Rt / (Rt + Rf)
    // let's call ADC/4095 = "adc"
    // adc = Rt/(Rt+Rf)
    // Rt*adc + Rf*adc = Rt  which simplifies to  Rt = Rf*adc/(1-adc),  
  temp = Thermistor_Fixed_R * temp / (1-temp); 
  temp = (1.0f / 298.15f) - log(Thermistor_R25 / temp) / Thermistor_Beta; // beta = log(Rt1/Rt2) / (1/T1 - 1/T2)
  temp = 1.0f / temp;
  temp -= 273.15f; // Convert from K to degC
  return temp;
}

float getVoltage(uint8_t adcpin){
  float volt = float(analogRead(adcpin)) * Sensor_Vref / 4095;           //  Vout = Sensor_Vref * ADC / 4095
        volt = volt * (Sensor_Rbottom + Sensor_Rtop) / Sensor_Rbottom;     //   Vin = Vout * (Sensor_Rbottom + Sensor_Rtop) / Sensor_Rbottom 
  return volt;
}

uint16_t float_to_lin11(float input_val)
{
    int exponent = -16;    // set exponent to -16
    int mantissa = (int)(input_val / pow(2.0, exponent));  // extract mantissa from input value 
   do
    {
   if((mantissa >= -1024) && (mantissa <= +1023))
    {
      break; // stop if mantissa valid
    }
      exponent++;                                        // Search for an exponent that produces
      mantissa = (int)(input_val / pow(2.0, exponent));  // a valid 11-bit mantissa
    } 
  while (exponent < +15);
  uint16_t uExponent = exponent << 11;       // Format the exponent of the L11
  uint16_t uMantissa = mantissa & 0x07FF;    // Format the mantissa of the L11
  return uExponent | uMantissa;              // Compute value as exponent | mantissa
}

void spi_command(uint8_t cmd, bool isWrite = false){

  if (isWrite)
  {
    // SPI_WriteEnable(); // Write Enable
    // spi_command(SPIFLASH_WRITEENABLE); // Write Enable
    select();
    SPI_3.transfer(SPIFLASH_WRITEENABLE);
    unselect();
  }
  select();
  SPI_3.transfer(cmd);
}

void select() {
  // SPI_3.beginTransaction(settings);
  digitalWrite(SPI_SELECT_PIN, LOW);
}

void unselect() {
  digitalWrite(SPI_SELECT_PIN, HIGH);
  // SPI_3.endTransaction();
}

uint32_t readDeviceId()
{
  select();
  SPI_3.transfer(SPIFLASH_IDREAD);
  // spi_command(SPIFLASH_IDREAD);
  uint32_t jedecid = SPI_3.transfer(0) << 16;
  jedecid |= SPI_3.transfer(0) << 8;
  jedecid |= SPI_3.transfer(0);
  unselect();
  return jedecid;
}

void readUniqueId(uint8_t *values)
{
  spi_command(SPIFLASH_MACREAD);
  SPI_3.transfer(0);
  SPI_3.transfer(0);
  SPI_3.transfer(0);
  SPI_3.transfer(0);
  for (uint8_t i=0;i<8;i++)
    values[i] = SPI_3.transfer(0);
  unselect();
}

uint8_t readByte(uint32_t addr) {
  spi_command(SPIFLASH_ARRAYREADLOWFREQ);
  SPI_3.transfer(addr >> 16);
  SPI_3.transfer(addr >> 8);
  SPI_3.transfer(addr);
  uint8_t result = SPI_3.transfer(0);
  unselect();
  return result;
}

void readBytes(uint32_t addr, void* buf, uint16_t len) {
  spi_command(SPIFLASH_ARRAYREAD);
  SPI_3.transfer(addr >> 16);
  SPI_3.transfer(addr >> 8);
  SPI_3.transfer(addr);
  SPI_3.transfer(0); //"dont care"
  for (uint16_t i = 0; i < len; ++i)
    ((uint8_t*) buf)[i] = SPI_3.transfer(0);
  unselect();
}

void chipErase() {
  spi_command(SPIFLASH_CHIPERASE, true);
  unselect();
}

void showpagedata(uint16_t page){
    uint32_t addr = 0;
    uint8_t page_data[256];
    for(uint16_t i = 0; i < page; i++) {
    readBytes(addr * 256, page_data, 0x100);
    Serial2.printf("Page : 0x%04X,  Addr : 0x%06X \n", addr, addr*256);
    addr++;
    printpagedata(page_data);
    delay(20);
    Serial2.println();
  }
}

void printpagedata(uint8_t *values) {
      char c[6];   
      Serial2.print(F("    "));
      for (int i = 0; i < 16; i++) {
              sprintf(c, "%3x",  i);
              Serial2.print(c);
           }
      for (int address = 0; address < 256; address++) {   
            if (address % 16 == 0) {
                  sprintf(c, "0x%02x:", address & 0xF0);
                  Serial2.println();
                  Serial2.print(c);
                }
                sprintf(c, " %02x", values[address]);
                Serial2.print(c);
        }
    Serial2.println(F(""));
}

 void spi_flash_erase(){
  uint8_t UniqueId[8];
  Serial2.println();
  Serial2.printf("Flash_ID: 0x%06X, \n", readDeviceId());
  readUniqueId(UniqueId);
  Serial2.printf("UniqueID_ID: 0x%02X%02X%02X%02X%02X%02X%02X%02X, \n", UniqueId[0], UniqueId[1], UniqueId[2], 
  UniqueId[3], UniqueId[4], UniqueId[5], UniqueId[6], UniqueId[7]);
  chipErase();
  Serial2.println("ChipErase!!!");
  delay(10);
 } 