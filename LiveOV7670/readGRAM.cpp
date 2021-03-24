// see if 4-line mode can read anything

#include "readGRAM.h"
#include <SPI.h>

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TFT_DC  (48)     //DC=7 for HX8347
#define TFT_RESET (49)   //Backlight on HX8347
#define SS (53)
#define SCK (52)
#define MOSI (51)
#else
#define TFT_DC  (8)     //DC=7 for HX8347
#define TFT_RESET (10)   //Backlight on HX8347
#define SS (9)
#define SCK (13)
#define MOSI (11)
#endif

uint32_t GRAM_buffer[165];

void SPIwrite(uint8_t data)
{
  pinMode(MOSI, OUTPUT);
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(MOSI, (data & 0x80) != 0);
    digitalWrite(SCK, HIGH);
    digitalWrite(SCK, LOW);
    data <<= 1;
  }
}

uint8_t readSPI()
{
  uint8_t data = 0;
  pinMode(MOSI, INPUT_PULLUP);;
  for (int i = 0; i < 8; i++)
  {
    data <<= 1;
    if (digitalRead(MOSI)) data |= 1;;
    digitalWrite(SCK, HIGH);
    digitalWrite(SCK, LOW);

  }
  return data;
}

void writeData(uint8_t Data)
{
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, HIGH);
  SPIwrite(Data);
  digitalWrite(SS, HIGH);
}

void writeCmd(uint8_t cmd)
{
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPIwrite(cmd);
  digitalWrite(SS, HIGH);
}

uint8_t readData()
{
  uint8_t data = 0;
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, HIGH);
  data = readSPI();
  digitalWrite(SS, HIGH);
  return data;
}

void setXY(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1)
{
  writeCmd(0x2A);
  writeData(0x00);
  writeData(x0);
  writeData(0x00);
  writeData(x1);

  writeCmd(0x2B);
  writeData(0x00);
  writeData(y0);
  writeData(0x00);
  writeData(y1);
}

void readID()
{
  Serial.print("Read ID cmd: 0x");
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPIwrite(0x04);

  digitalWrite(TFT_DC, HIGH);
  digitalWrite(SCK, HIGH);
  digitalWrite(SCK, LOW);
  for (int i = 0; i < 3; i++)
  {
    Serial.print(readSPI(), HEX);
  }
  digitalWrite(SS, HIGH);
  Serial.println("...End ");
}

void readGRAM_Row(uint32_t *dataBuff, uint8_t Row_number)
{
  uint8_t tempValue = 0;
  setXY(0, 128, Row_number, Row_number + 1);
  // Serial.print("Read GRAM Row: ");
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPIwrite(0x2E);
  digitalWrite(TFT_DC, HIGH);
  readSPI();  // Dummy 1 byte read
  for (int i = 0; i < 128; i++)
  {
    dataBuff[i] = 0x00000000;
  //     Serial.print("0x");
    for (int j = 0; j < 3; j++)
    {
      dataBuff[i] = (dataBuff[i] << 8);
      tempValue = (readSPI() << 1);
      dataBuff[i] |= tempValue;
  //        if(tempValue < 0x10) { Serial.print("0"); }
  //        Serial.print(tempValue, HEX);
    }
   //  Serial.print(", ");
 //       if((i%8) == 0)
 //    {
 //     Serial.println("");
//    }
  }
  digitalWrite(SS, HIGH);
//    Serial.println("...End ");
}

void readGRAM_Column(uint32_t *dataBuff, uint8_t Column_number)
{
  uint8_t tempValue = 0;
  setXY(Column_number, Column_number + 1, 0, 160);
  Serial.print("Read GRAM Column: ");
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPIwrite(0x2E);
  digitalWrite(TFT_DC, HIGH);
  readSPI();  // Dummy 1 byte read
  for (int i = 0; i < 160; i++)
  {
    dataBuff[i] = 0x00000000;
    //  Serial.print("0x");
    for (int j = 0; j < 3; j++)
    {
      dataBuff[i] = (dataBuff[i] << 8);
      tempValue = (readSPI() << 1);
      dataBuff[i] |= tempValue;
      if (tempValue < 0x10) {
        Serial.print("0");
      }
      Serial.print(tempValue);
    }
    Serial.print(", ");
    if ((i % 8) == 0)
    {
      Serial.println("");
    }
  }
  digitalWrite(SS, HIGH);
  Serial.println("...End ");
}

void FillRainbow()
{
  int Row = 0, Column = 0;
  uint32_t color = 0;
  uint8_t red = 0, green = 0, blue = 0;
  setXY(0, 128, 0, 160);
  writeCmd(0x2C);
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, HIGH);

  Serial.println("Rainbow Screen Start");
  for (int Row = 0; Row < 160; Row++)
  {
    Serial.print(".");
    for (int Column = 0; Column < 128; Column++)
    {
      SPIwrite(red);
      SPIwrite(green);
      SPIwrite(blue);

      if ((Row == 0) || (Row == 1))
      {
        red += 0x02;
        green += 0x02;
        blue += 0x02;
      }
      else
      {
        red = Row * 2;
        green = Column * 2;
        blue = Row + Column;
      }
    }
  }
  digitalWrite(SS, HIGH);
  Serial.println("Rainbow Screen Ends.");
}


void fillscreen(uint8_t red, uint8_t green, uint8_t blue)
{
  setXY(0, 128, 0, 160);
  writeCmd(0x2C);
  digitalWrite(SS, LOW);
  digitalWrite(TFT_DC, HIGH);
  for (int i = 0; i < 20480; i++)
  {
    SPIwrite(red);
    SPIwrite(green);
    SPIwrite(blue);
  }
  digitalWrite(SS, HIGH);
  Serial.println("");
  Serial.print("Data Written on GRAM 0x");
  if (red < 0x10) {
    Serial.print("0");
  }
  Serial.print(red, HEX);
  if (green < 0x10) {
    Serial.print("0");
  }
  Serial.print(green, HEX);
  if (blue < 0x10) {
    Serial.print("0");
  }
  Serial.println(blue, HEX);
  Serial.println("");
}

void initiate_readGRAM()
{
  SPI.end();
  /*  Configure the PIN modes */
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_RESET, HIGH);
  pinMode(TFT_RESET, OUTPUT);

  writeCmd(0x01);            //Software Reset
  delay(100);

  writeCmd(0x11);            //Sleep Out
  delay(100);
  writeCmd(0x29);     // Dislay ON
  delay(100);
}
