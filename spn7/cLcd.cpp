// cLcd.cpp
//   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//   x  GND   EXTMODE   5v   VCOM   MOSI   3.3v  x
//   x  GND     5v     DISP   CS    SCLK    GND  x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
#include "cLcd.h"
#include "font.h"
//{{{  defines
#define VCOM_PIN    GPIO_PIN_11  // GPIO PB11  VCOM - TIM2 CH4 1Hz flip
#define CS_PIN      GPIO_PIN_12  // SPI2 PB12  CS/NSS active hi
#define SCK_PIN     GPIO_PIN_13  // SPI2 PB13  spi2 SCK
#define POWER_PIN   GPIO_PIN_14  // GPIO PB14  lcd power
#define MOSI_PIN    GPIO_PIN_15  // SPI2 PB15  spi2 MOSI

#define paddingByte 0x00
#define clearByte   0x20 // unused
#define vcomByte    0x40 // unused
#define commandByte 0x80

#define POLY_X(Z)  ((int32_t)((points + Z)->x))
#define POLY_Y(Z)  ((int32_t)((points + Z)->y))
#define ABS(X)     ((X) > 0 ? (X) : -(X))
//}}}

const uint8_t kFirstMask[8] = { 0xFF, 0x7F, 0x3F, 0x1F, 0x0f, 0x07, 0x03, 0x01 };
const uint8_t kLastMask[8] =  { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };

cLcd* cLcd::mLcd = nullptr;

extern "C" {
  void SPI2_IRQHandler() { HAL_SPI_IRQHandler (cLcd::mLcd->getSpiHandle()); }
  void DMA1_Channel5_IRQHandler() { HAL_DMA_IRQHandler (cLcd::mLcd->getSpiHandle()->hdmatx); }
  }

//{{{
bool cLcd::init() {

  mLcd = this;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  // config CS, POWER_PIN
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pin = CS_PIN | POWER_PIN;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_PULLUP;
  gpioInit.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init (GPIOB, &gpioInit);

  // POWER_PIN hi
  GPIOB->BSRR = POWER_PIN;

  //{{{  config VCOM as tim2 1Hz pwm
  __HAL_RCC_TIM2_CLK_ENABLE();

  // config VCOM GPIOB as TIM2 CH4
  gpioInit.Pin = VCOM_PIN;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  gpioInit.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init (GPIOB, &gpioInit);

  TIM_HandleTypeDef timHandle = {0};
  timHandle.Instance = TIM2;
  timHandle.Init.Period = 10000 - 1;
  timHandle.Init.Prescaler = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  timHandle.Init.ClockDivision = 0;
  timHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init (&timHandle))
    printf ("HAL_TIM_Base_Init failed\n");

  // init timOcInit
  TIM_OC_InitTypeDef timOcInit = {0};
  timOcInit.OCMode       = TIM_OCMODE_PWM1;
  timOcInit.OCPolarity   = TIM_OCPOLARITY_HIGH;
  timOcInit.OCFastMode   = TIM_OCFAST_DISABLE;
  timOcInit.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  timOcInit.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  timOcInit.OCIdleState  = TIM_OCIDLESTATE_RESET;
  timOcInit.Pulse = 10000 / 2;
  if (HAL_TIM_PWM_ConfigChannel (&timHandle, &timOcInit, TIM_CHANNEL_4))
    printf ("HAL_TIM_PWM_ConfigChannel failed\n");

  if (HAL_TIM_PWM_Start (&timHandle, TIM_CHANNEL_4))
    printf ("HAL_TIM2_PWM_Start failed\n");
  //}}}
  //{{{  config SPI2 tx
  // config SPI2 GPIOB
  gpioInit.Pin = SCK_PIN | MOSI_PIN;
  gpioInit.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init (GPIOB, &gpioInit);

  // set SPI2 master, mode0, 8bit
  __HAL_RCC_SPI2_CLK_ENABLE();
  SPI_HandleTypeDef SPI_Handle = {0};
  mSpiHandle.Instance = SPI2;
  mSpiHandle.Init.Mode              = SPI_MODE_MASTER;
  mSpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  mSpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  mSpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW; // SPI mode 0
  mSpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;  // SPI mode 0
  mSpiHandle.Init.NSS               = SPI_NSS_SOFT;
  mSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 4 = 10.5mHz
  mSpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  mSpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  HAL_SPI_Init (&mSpiHandle);

  // config tx dma
  __HAL_RCC_DMA1_CLK_ENABLE();
  mSpiTxDma = {0};
  mSpiTxDma.Instance = DMA1_Channel5;
  mSpiTxDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  mSpiTxDma.Init.PeriphInc           = DMA_PINC_DISABLE;
  mSpiTxDma.Init.MemInc              = DMA_MINC_ENABLE;
  mSpiTxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  mSpiTxDma.Init.MemDataAlignment    = DMA_PDATAALIGN_BYTE;
  mSpiTxDma.Init.Mode                = DMA_NORMAL;
  mSpiTxDma.Init.Priority            = DMA_PRIORITY_LOW;
  HAL_DMA_Init (&mSpiTxDma);
  __HAL_LINKDMA (&mSpiHandle, hdmatx, mSpiTxDma);

  HAL_NVIC_SetPriority (DMA1_Channel5_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ (DMA1_Channel5_IRQn);

  // SPI2 enable
  SPI2->CR1 |= SPI_CR1_SPE;
  //}}}

  // init frameBuf command : 240 * (lineByte:15bytes:padding) : padding
  const int frameBufLen = 1 + (((getWidth()/8) + 2) * getHeight()) + 1;
  mFrameBuf = (uint8_t*)malloc (frameBufLen);
  if (mFrameBuf) {
    printf ("cLcd::init frameBuf alloc:%d\n", frameBufLen);

    memset (mFrameBuf+1, 0, frameBufLen);
    mFrameBuf [0] = commandByte;
    for (uint16_t y = 0; y < getHeight(); y++) {
      uint8_t lineAddressByte = y+1;
      // bit reverse
      lineAddressByte = ((lineAddressByte & 0xF0) >> 4) | ((lineAddressByte & 0x0F) << 4);
      lineAddressByte = ((lineAddressByte & 0xCC) >> 2) | ((lineAddressByte & 0x33) << 2);
      lineAddressByte = ((lineAddressByte & 0xAA) >> 1) | ((lineAddressByte & 0x55) << 1);
      mFrameBuf [1 + (y*getPitch())] = lineAddressByte;
      }

    present();
    }
  else
    printf ("cLcd::init frameBuf alloc fail\n");

  return mFrameBuf;
  }
//}}}

//{{{
void cLcd::clear (eDraw draw) {

  if (draw == eInvert)
    printf ("cLcd::clear - eInvert clear\n");

  // point to first line pixel bytes
  auto framePtr = mFrameBuf + 2;
  for (int y = 0; y < getHeight(); y++) {
    memset (framePtr, draw == eOn ? 0xFF : 0, getWidth()/8);
    framePtr += getPitch();
    }
  }
//}}}
//{{{
void cLcd::fillRect (eDraw draw, cRect rect) {

  // clip x
  if (rect.left < 0)
    rect.left = 0;
  else if (rect.right > getWidth())
    rect.right = getWidth();
  if (rect.left >= rect.right)
    return;

  // clip y
  if (rect.top < 0)
    rect.top = 0;
  else if (rect.bottom > getHeight())
    rect.bottom = getHeight();
  if (rect.top >= rect.bottom)
    return;

  int16_t firstByte = rect.left / 8;
  int16_t lastByte = (rect.right-1) / 8;

  if (firstByte == lastByte) {
    // simple single x byte case
    uint8_t mask = kFirstMask[rect.left & 7] & kLastMask[(rect.right-1) & 7];
    auto framePtr = getFramePtr (rect.top) + firstByte;
    for (uint16_t y = rect.top; y < rect.bottom; y++) {
      if (draw == eOff)
        *framePtr &= ~mask;
      else if (draw == eInvert)
        *framePtr ^= mask;
      else
        *framePtr |= mask;
      framePtr += getPitch();
      }
    }
  else {
    // multiple x bytes
    uint8_t firstMask = kFirstMask[rect.left & 7];
    uint8_t lastMask = kLastMask[(rect.right-1) & 7];
    auto framePtr = getFramePtr (rect.top) + firstByte;
    if (draw == eOff) {
      //{{{  eOff
      for (uint16_t y = rect.top; y < rect.bottom; y++) {
        int16_t byte = firstByte;
        *framePtr++ &= ~firstMask;
        byte++;

        while (byte < lastByte) {
          *framePtr++ = 0x00;
          byte++;
          }

        if (byte == lastByte)
          *framePtr++ &= ~lastMask;

        framePtr += getPitch() - (lastByte - firstByte) - 1;
        }
      }
      //}}}
    else if (draw == eOn) {
      //{{{  eOn
      for (uint16_t y = rect.top; y < rect.bottom; y++) {
        int16_t byte = firstByte;
        *framePtr++ |= firstMask;
        byte++;

        while (byte < lastByte) {
          *framePtr++ = 0xFF;
          byte++;
          }

        if (byte == lastByte)
          *framePtr++ |= lastMask;

        framePtr += getPitch() - (lastByte - firstByte) - 1;
        }
      }
      //}}}
    else {
      //{{{  eInvert
      for (uint16_t y = rect.top; y < rect.bottom; y++) {
        int16_t byte = firstByte;
        *framePtr++ ^= firstMask;
        byte++;

        while (byte < lastByte) {
          *framePtr++ ^= 0xFF;
          byte++;
          }

        if (byte == lastByte)
          *framePtr++ ^= lastMask;

        framePtr += getPitch() - (lastByte - firstByte) - 1;
        }
      }
      //}}}
    }
  }
//}}}
//{{{
void cLcd::fillRect (eDraw draw, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
  fillRect (draw, cRect (x, y, x+width, y+height));
  }
//}}}
//{{{
void cLcd::drawLine (eDraw draw, cPoint p1, cPoint p2) {

  int16_t deltax = ABS(p2.x - p1.x); // The difference between the x's
  int16_t deltay = ABS(p2.y - p1.y); // The difference between the y's

  cPoint p = p1;
  cPoint inc1 ((p2.x >= p1.x) ? 1 : -1, (p2.y >= p1.y) ? 1 : -1);
  cPoint inc2 = inc1;

  int16_t numAdd = (deltax >= deltay) ? deltay : deltax;
  int16_t den = (deltax >= deltay) ? deltax : deltay;
  if (deltax >= deltay) { // There is at least one x-value for every y-value
    inc1.x = 0;            // Don't change the x when numerator >= denominator
    inc2.y = 0;            // Don't change the y for every iteration
    }
  else {                  // There is at least one y-value for every x-value
    inc2.x = 0;            // Don't change the x for every iteration
    inc1.y = 0;            // Don't change the y when numerator >= denominator
    }
  int16_t num = den / 2;
  int16_t numPixels = den;

  for (int16_t pixel = 0; pixel <= numPixels; pixel++) {
    drawPix (draw, p);
    num += numAdd;     // Increase the numerator by the top of the fraction
    if (num >= den) {   // Check if numerator >= denominator
      num -= den;       // Calculate the new numerator value
      p += inc1;
      }
    p += inc2;
    }
  }
//}}}
//{{{
void cLcd::drawPolygon (eDraw draw, cPoint* points, uint16_t numPoints) {

  int16_t x = 0, y = 0;

  if (numPoints < 2)
    return;

  drawLine (draw, *points, *(points + numPoints-1));

  while (--numPoints) {
    cPoint point = *points++;
    drawLine (draw, point, *points);
    }
  }
//}}}
//{{{
void cLcd::drawCircle (eDraw draw, cPoint centre, int16_t radius) {

  int32_t decision = 3 - (radius << 1);

  cPoint p = {0, radius};
  while (p.x <= p.y) {
    drawPix (draw, centre + cPoint (p.x, -p.y));
    drawPix (draw, centre + cPoint (-p.x, -p.y));
    drawPix (draw, centre + cPoint (p.y, -p.x));
    drawPix (draw, centre + cPoint (-p.y, -p.x));
    drawPix (draw, centre + cPoint (p.x, p.y));
    drawPix (draw, centre + cPoint (-p.x, p.y));
    drawPix (draw, centre + cPoint (p.y, p.x));
    drawPix (draw, centre + cPoint (-p.y, p.x));

    if (decision < 0)
      decision += (p.x << 2) + 6;
    else {
      decision += ((p.x - p.y) << 2) + 10;
      p.y--;
      }

    p.x++;
    }

  }
//}}}
//{{{
void cLcd::drawEllipse (eDraw draw, cPoint centre, cPoint radius) {

  int x = 0;
  int y = -radius.y;

  int err = 2 - 2 * radius.x;
  float k = (float)radius.y / (float)radius.x;

  do {
    drawPix (draw, centre + cPoint (-(int16_t)(x / k), y));
    drawPix (draw, centre + cPoint ((int16_t)(x / k), y));
    drawPix (draw, centre + cPoint ((int16_t)(x / k), -y));
    drawPix (draw, centre + cPoint (- (int16_t)(x / k), - y));

    int e2 = err;
    if (e2 <= x) {
      err += ++x * 2+ 1 ;
      if (-y == x && e2 <= y)
        e2 = 0;
      }
    if (e2 > y)
      err += ++y *2 + 1;
    } while (y <= 0);
  }
//}}}
//{{{
void cLcd::fillTriangle (eDraw draw, cPoint p1, cPoint p2, cPoint p3) {

  cPoint inc1;
  cPoint inc2;

  if (p2.x >= p1.x) {
    //{{{  x increasing
    inc1.x = 1;
    inc2.x = 1;
    }
    //}}}
  else {
    //{{{  x decreasing
    inc1.x = -1;
    inc2.x = -1;
    }
    //}}}

  if (p2.y >= p1.y) {
    //{{{  y increasing
    inc1.y = 1;
    inc2.y = 1;
    }
    //}}}
  else {
    //{{{  y decreasing
    inc1.y = -1;
    inc2.y = -1;
    }
    //}}}

  int16_t den;
  int16_t num;
  int16_t num_add;
  int16_t num_pixels;

  int16_t deltax = ABS (p2.x - p1.x);  // The difference between the x's
  int16_t deltay = ABS (p2.y - p1.y);  // The difference between the y's
  if (deltax >= deltay) {
    //{{{  at least one x-value for every y-value
    inc1.x = 0;           // Don't change the x when numerator >= denominator
    inc2.y = 0;           // Don't change the y for every iteration

    den = deltax;
    num = deltax / 2;
    num_add = deltay;
    num_pixels = deltax;  // There are more x-values than y-values
    }
    //}}}
  else {
    //{{{  at least one y-value for every x-value
    inc2.x = 0;           // Don't change the x for every iteration
    inc1.y = 0;           // Don't change the y when numerator >= denominator

    den = deltay;
    num = deltay / 2;
    num_add = deltax;
    num_pixels = deltay; // There are more y-values than x-values
    }
    //}}}

  cPoint p = p1;
  for (int16_t curpixel = 0; curpixel <= num_pixels; curpixel++) {
    drawLine (draw, p, p3);
    num += num_add;     // Increase the numerator by the top of the fraction
    if (num >= den)  {  // Check if numerator >= denominator
      num -= den;       // Calculate the new numerator value
      p += inc1;       // Change the x as appropriate
      }

    p += inc2;         // Change the x as appropriate
    }
  }
//}}}
//{{{
void cLcd::fillPolygon (eDraw draw, cPoint* points, uint16_t numPoints) {

  cPoint tl = *points;
  cPoint br = *points;

  cPoint pixel;
  for (int16_t counter = 1; counter < numPoints; counter++) {
    pixel.x = POLY_X (counter);
    if (pixel.x < tl.x)
      tl.x = pixel.x;
    if (pixel.x > br.x)
      br.x = pixel.x;

    pixel.y = POLY_Y(counter);
    if (pixel.y < tl.y)
      tl.y = pixel.y;
    if (pixel.y > br.y)
      br.y = pixel.y;
    }

  if (numPoints < 2)
    return;

  cPoint centre = (tl + br) / 2;
  cPoint first = *points;

  cPoint p2;
  while (--numPoints) {
    cPoint p1 = *points;
    points++;
    p2 = *points;

    fillTriangle (draw, p1, p2, centre);
    fillTriangle (draw, p1, centre, p2);
    fillTriangle (draw, centre, p2, p1);
    }

  fillTriangle (draw, first, p2, centre);
  fillTriangle (draw, first, centre, p2);
  fillTriangle (draw, centre, p2, first);
  }
//}}}
//{{{
void cLcd::fillCircle (eDraw draw, cPoint centre, uint16_t radius) {

  int32_t decision = 3 - (radius << 1);

  uint32_t current_x = 0;
  uint32_t current_y = radius;

  while (current_x <= current_y) {
    if (current_y > 0) {
      fillRect (draw, centre.x - current_y, centre.y + current_x, 1, 2*current_y);
      fillRect (draw, centre.x - current_y, centre.y - current_x, 1, 2*current_y);
      }
    if (current_x > 0) {
      fillRect (draw, centre.x - current_x, centre.y - current_y, 1, 2*current_x);
      fillRect (draw, centre.x - current_x, centre.y + current_y, 1, 2*current_x);
      }
    if (decision < 0)
      decision += (current_x << 2) + 6;
    else {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
      }
    current_x++;
    }

  drawCircle (draw, centre, radius);
  }
//}}}
//{{{
void cLcd::fillEllipse (eDraw draw, cPoint centre, cPoint radius) {

  int x = 0;
  int y = -radius.y;
  int err = 2 - 2 * radius.x;
  float k = (float)radius.y / (float)radius.x;

  do {
    fillRect (draw, (centre.x - (int16_t)(x/k)), (centre.y + y), 1, (2 * (int16_t)(x / k) + 1));
    fillRect (draw, (centre.x - (int16_t)(x/k)), (centre.y - y), 1, (2 * (int16_t)(x / k) + 1));

    int e2 = err;
    if (e2 <= x) {
      err += ++x * 2 + 1;
      if (-y == x && e2 <= y)
        e2 = 0;
      }
    if (e2 > y)
      err += ++y*2+1;
    } while (y <= 0);
  }
//}}}
//{{{
void cLcd::drawString (eDraw draw, eFont font, eAlign align, const std::string& str, cPoint p) {
// simple char clip to width

  const font_t* drawFont;
  if (font == eSmall)
    drawFont = &font18;
  else
    drawFont = &font36;

  switch (align) {
    //{{{
    case eCentre: {
      int16_t size = 0;
      for (auto ch : str)
        size += getCharWidth (drawFont, ch);
      p.x -= size/2;

      if (p.x < 0)
        p.x = 0;
      else if (p.x >= getWidth())
        p.x = 0;
      }
      break;
    //}}}
    //{{{
    case eRight: {
      int16_t size = 0;
      for (auto ch : str)
        size += getCharWidth (drawFont, ch);
      p.x -= size;

      if (p.x < 0)
        p.x = 0;
      else if (p.x >= getWidth())
        p.x = 0;
      }
      break;
    //}}}
    }

  for (auto ch : str) {
    if ((ch >= drawFont->firstChar) && (ch <= drawFont->lastChar)) {
      auto fontChar = (fontChar_t*)(drawFont->glyphsBase + drawFont->glyphOffsets[ch - drawFont->firstChar]);
      if (p.x + fontChar->left + fontChar->width < getWidth()) {
        auto charBytes = (uint8_t*)fontChar + 5;
        int16_t xfirst = p.x + fontChar->left;
        auto framePtr = getFramePtr (p.y + drawFont->height - fontChar->top) + xfirst/8;

        for (uint16_t y = 0; y < fontChar->height; y++) {
          int16_t x = xfirst;
          int16_t charBits = fontChar->width;
          uint8_t charByte = 0;
          while (charBits > 0) {
            uint8_t xbit = x & 7;
            charByte |= (*charBytes) >> xbit;
            if (draw == eOff)
              *framePtr++ &= ~charByte;
            else if (draw == eInvert)
              *framePtr++ ^= charByte;
            else
              *framePtr++ |= charByte;
            charBits -= 8;
            charByte = (*charBytes) << (8 - xbit);
            charBytes++;
            }
          if (draw == eOff)
            *framePtr &= ~charByte;
          else if (draw == eInvert)
            *framePtr ^= charByte;
          else
            *framePtr |= charByte;
          framePtr += getPitch() - (fontChar->width + 7)/8;
          }
        }
      p.x += fontChar->advance;
      }
    else
      p.x += drawFont->spaceWidth;
    }
  }
//}}}
//{{{
void cLcd::present() {

  // CS hi
  GPIOB->BSRR = CS_PIN;

  if (HAL_SPI_Transmit_DMA (&mSpiHandle, mFrameBuf, 1 + getHeight() * getPitch() + 1))
    printf ("HAL_SPI_Transmit failed\n");

  while (HAL_SPI_GetState (&mSpiHandle) != HAL_SPI_STATE_READY)
    HAL_Delay (1);

  // CS lo
  GPIOB->BSRR = CS_PIN << 16;

  auto ticks = HAL_GetTick();
  mTookTicks = ticks - mPresentTicks;
  mPresentTicks = ticks;
  mFrameNum++;
  }
//}}}

// private:
//{{{
uint16_t cLcd::getCharWidth (const font_t* font, char ascii) {

  if ((ascii >= font->firstChar) && (ascii <= font->lastChar)) {
    auto char8 = (uint8_t*)(font->glyphsBase + font->glyphOffsets[ascii - font->firstChar]);
    return char8[4];
    }

  return font->spaceWidth;
  }
//}}}
//{{{
void cLcd::drawPix (eDraw draw, cPoint p) {

  uint8_t mask = 0x80 >> (p.x & 7);
  auto framePtr = getFramePtr (p.y) + p.x/8;

  if (draw == eInvert)
    *framePtr++ ^= mask;
  else if (draw == eOff)
    *framePtr++ &= ~mask;
  else
    *framePtr++ |= mask;
  }
//}}}
