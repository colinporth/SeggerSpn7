// main.c
//{{{  includes
#include "stm32f3xx_nucleo.h"

#include <math.h>
#include "cPointRect.h"
#include "utils.h"
#include "font.h"

#include "ihm07m1.h"
#include "sixStepLib.h"
//}}}

const uint8_t kFirstMask[8] = { 0xFF, 0x7F, 0x3F, 0x1F, 0x0f, 0x07, 0x03, 0x01 };
const uint8_t kLastMask[8] =  { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };
//{{{
class cLcd {
public:
  //{{{  defines
  //    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //    x  GND   EXTMODE   5v   VCOM   MOSI   3.3v  x
  //    x  GND     5v     DISP   CS    SCLK    GND  x
  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

  #define SCK_PIN        GPIO_PIN_13  //  SPI2  PB13  SCK
  #define MOSI_PIN       GPIO_PIN_15  //  SPI2  PB15  MOSI
  #define CS_PIN         GPIO_PIN_12  //  SPI2  PB12  CS/NSS active hi
  #define VCOM_PIN       GPIO_PIN_11  //  GPIO  PB11  VCOM - TIM2 CH4 1Hz flip

  #define paddingByte 0x00
  #define clearByte   0x20 // unused
  #define vcomByte    0x40 // unused
  #define commandByte 0x80

  #define POLY_X(Z)  ((int32_t)((points + Z)->x))
  #define POLY_Y(Z)  ((int32_t)((points + Z)->y))
  #define ABS(X)     ((X) > 0 ? (X) : -(X))
  //}}}
  enum eDraw { eInvert, eOff, eOn };
  enum eFont { eSmall, eBig };
  enum eAlign { eLeft, eCentre, eRight };

  //{{{
  bool init() {

    // config CS, DISP, init lo
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpioInit;
    gpioInit.Pin = CS_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init (GPIOB, &gpioInit);

    //{{{  init tim2
    // config VCOM GPIOB as TIM2 CH4
    gpioInit.Pin = VCOM_PIN;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init (GPIOB, &gpioInit);

    __HAL_RCC_TIM2_CLK_ENABLE();

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

    //}}}
    if (HAL_TIM_PWM_Start (&timHandle, TIM_CHANNEL_4))
      printf ("HAL_TIM2_PWM_Start failed\n");

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

  const uint16_t getWidth() { return kWidth; }
  const uint16_t getHeight() { return kHeight; }
  const cPoint getSize() { return cPoint (kWidth, kHeight); }
  const cRect getRect() { return cRect (getSize()); }
  const cPoint getCentre() { return getSize()/2; }

  uint32_t getFrameNum() { return mFrameNum; }
  uint32_t getTookTicks() { return mTookTicks; }

  //{{{
  void clear (eDraw draw) {

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
  void fillRect (eDraw draw, cRect rect) {

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
  void fillRect (eDraw draw, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    fillRect (draw, cRect (x, y, x+width, y+height));
    }
  //}}}
  //{{{
  void drawLine (eDraw draw, cPoint p1, cPoint p2) {

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
  void drawPolygon (eDraw draw, cPoint* points, uint16_t numPoints) {

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
  void drawCircle (eDraw draw, cPoint centre, int16_t radius) {

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
  void drawEllipse (eDraw draw, cPoint centre, cPoint radius) {

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
  void fillTriangle (eDraw draw, cPoint p1, cPoint p2, cPoint p3) {

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
  void fillPolygon (eDraw draw, cPoint* points, uint16_t numPoints) {

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
  void fillCircle (eDraw draw, cPoint centre, uint16_t radius) {

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
  void fillEllipse (eDraw draw, cPoint centre, cPoint radius) {

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
  void drawString (eDraw draw, eFont font, eAlign align, const std::string& str, cPoint p) {
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
  void present() {

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

  // for irqs
  SPI_HandleTypeDef* getSpiHandle() { return &mSpiHandle; }

private:
  const uint16_t kWidth = 400;
  const uint16_t kHeight = 240;

  uint16_t getPitch() { return 2 + getWidth()/8; }
  uint8_t* getFramePtr (int16_t y) { return mFrameBuf + 2 + y*getPitch(); }
  //{{{
  uint16_t getCharWidth (const font_t* font, char ascii) {
    if ((ascii >= font->firstChar) && (ascii <= font->lastChar)) {
      auto char8 = (uint8_t*)(font->glyphsBase + font->glyphOffsets[ascii - font->firstChar]);
      return char8[4];
      }
    return font->spaceWidth;
    }
  //}}}

  //{{{
  void drawPix (eDraw draw, cPoint p) {

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

  // frameBuf - commandByte | 240 * (lineAddressByte | 50bytes,400pixels | paddingByte0) | paddingByte0
  uint8_t* mFrameBuf = nullptr;
  uint32_t mFrameNum = 0;
  uint32_t mPresentTicks = 0;
  uint32_t mTookTicks = 0;

  SPI_HandleTypeDef mSpiHandle;
  DMA_HandleTypeDef mSpiTxDma;
  };
//}}}
cLcd lcd;
//{{{
extern "C" {
  void SPI2_IRQHandler() { HAL_SPI_IRQHandler (lcd.getSpiHandle()); }
  void DMA1_Channel5_IRQHandler() { HAL_DMA_IRQHandler (lcd.getSpiHandle()->hdmatx); }
  }
//}}}

//{{{  vars
uint32_t mLastButtonPress = 0;
uint16_t mAlignTicks = 1;
uint16_t mMotorStartupCount = 1;

uint32_t mech_accel_hz = 0;               // Hz -- Mechanical acceleration rate
uint32_t constant_k = 0;                  // 1/3*mech_accel_hz

uint32_t Time_vector_tmp = 0;             // Startup variable
uint32_t Time_vector_prev_tmp = 0 ;       // Startup variable
uint32_t T_single_step = 0;               // Startup variable
uint32_t T_single_step_first_value = 0;   // Startup variable

int32_t delta = 0;                        // Startup variable
uint16_t index_array = 1;                 // Speed filter variable
int16_t speed_tmp_array[FILTER_DEEP];     // Speed filter variable
uint16_t speed_tmp_buffer[FILTER_DEEP];   // Potentiometer filter variable
bool array_completed = false;             // Speed filter variable
bool buffer_completed = false;            // Potentiometer filter variable

#define POT_VALUES_SIZE                   10
uint16_t mPotValueIndex = 0;              // High-Frequency Buffer Index
uint16_t mPotValues[POT_VALUES_SIZE];        // Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion

uint32_t ARR_LF = 0;                      // Autoreload LF TIM variable
int32_t Mech_Speed_RPM = 0;               // Mechanical motor speed
int32_t El_Speed_Hz = 0;                  // Electrical motor speed

uint16_t target_speed = TARGET_SPEED > 0 ? TARGET_SPEED : -TARGET_SPEED;

uint16_t index_ARR_step = 1;
uint32_t n_zcr_startup = 0;
uint16_t shift_n_sqrt = 14;

uint16_t mOpenLoopBemfEvent = 0;
bool mOpenLoopBemfFailure = false;
bool mSpeedFeedbackFailure = false;

int32_t speed_sum_sp_filt = 0;
int32_t speed_sum_pot_filt = 0;
uint16_t index_pot_filt = 1;
int16_t potent_filtered = 0;

uint32_t counter_ARR_Bemf = 0;
uint64_t constant_multiplier_tmp = 0;
//}}}
cSixStep sixStep;
cPiParam piParam;
std::string gStateString = "init";

#define AVERAGE_VALUES 20
#define NUM_VALUES 8000
uint32_t mValueIndex = 0;
uint8_t* mValues0 = nullptr;
uint8_t* mValues1 = nullptr;
uint8_t* mValues2 = nullptr;

//{{{
uint64_t fastSqrt (uint64_t wInput) {

  uint64_t tempRoot;
  if (wInput <= (uint64_t)((uint64_t)2097152 << shift_n_sqrt))
    tempRoot = (uint64_t)((uint64_t)128 << shift_n_sqrt);
  else
    tempRoot = (uint64_t)((uint64_t)8192 << shift_n_sqrt);

  uint8_t biter = 0u;
  uint64_t tempRootNew;
  do {
    tempRootNew = (tempRoot + wInput / tempRoot) >> 1;
    if (tempRootNew == tempRoot)
      biter = shift_n_sqrt -1 ;
    else {
      biter ++;
      tempRoot = tempRootNew;
      }
    } while (biter < (shift_n_sqrt - 1));

  return tempRootNew;
  }
//}}}
//{{{
void potSpeedTarget() {

  target_speed = sixStep.mAdcBuffer[1] * MAX_POT_SPEED / 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;

  if (target_speed > (MAX_POT_SPEED / VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);

  //printf ("potSpeedTarget %d\n", target_speed);
  }
//}}}
//{{{
void potSpeed() {

  uint32_t sum = 0;
  uint16_t max = 0;
  for (uint16_t i = 0; i < POT_VALUES_SIZE; i++) {
    uint16_t val = mPotValues[i];
    sum += val;
    if (val > max)
      max = val;
    }
  sum -= max;
  uint16_t potMean = sum / (POT_VALUES_SIZE - 1);

  if (!buffer_completed) {
    speed_tmp_buffer[index_pot_filt] = potMean;
    speed_sum_pot_filt = 0;
    for (uint16_t i = 1; i <= index_pot_filt;i++)
      speed_sum_pot_filt = speed_sum_pot_filt + speed_tmp_buffer[i];
    potent_filtered = speed_sum_pot_filt / index_pot_filt;
    index_pot_filt++;

    if (index_pot_filt >= FILTER_DEEP) {
      index_pot_filt = 1;
      buffer_completed = true;
      }
    }

  else {
     index_pot_filt++;
     if (index_pot_filt >= FILTER_DEEP)
       index_pot_filt = 1;

     speed_sum_pot_filt = 0;
     speed_tmp_buffer[index_pot_filt] = potMean;
     uint16_t speed_max = 0;
     for (uint16_t i = 1; i < FILTER_DEEP;i++) {
       uint16_t val = speed_tmp_buffer[i];
       if (val > speed_max)
         speed_max = val;
       speed_sum_pot_filt += val;
       }
     speed_sum_pot_filt -= speed_max;
     potent_filtered = speed_sum_pot_filt / (FILTER_DEEP-2);
    }

  if (potent_filtered==0)
    potent_filtered = 1;

  sixStep.Speed_Ref_filtered = potent_filtered;

  //printf ("potSpeed %d\n", potent_filtered);
  }
//}}}
//{{{
void speedFilter() {

  if (!array_completed) {
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    speed_sum_sp_filt = 0;
    for (uint16_t i = 1; i <= index_array;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / index_array;
    index_array++;

    if (index_array >= FILTER_DEEP) {
     index_array = 1;
     array_completed = true;
     }
    }

  else {
    index_array++;
    if (index_array >= FILTER_DEEP)
      index_array = 1;

    speed_sum_sp_filt = 0;
    speed_tmp_array[index_array] = sixStep.speed_fdbk;
    for (uint16_t i = 1; i < FILTER_DEEP;i++)
      speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
    sixStep.speed_fdbk_filtered = speed_sum_sp_filt / (FILTER_DEEP-1);
    }
  }
//}}}

//{{{
void sixStepTable (uint8_t step_number) {

  switch (step_number) {
    case 1:
      mcNucleoSetChanCCR (sixStep.pulse_value, 0, 0);
      mcNucleoEnableInputChan12();
      sixStep.mBemfIndex = 2;
      break;

    case 2:
      mcNucleoSetChanCCR (sixStep.pulse_value, 0, 0);
      mcNucleoEnableInputChan13();
      sixStep.mBemfIndex = 1;
      break;

    case 3:
      mcNucleoSetChanCCR (0, sixStep.pulse_value, 0);
      mcNucleoEnableInputChan23();
      sixStep.mBemfIndex = 0;
      break;

    case 4:
      mcNucleoSetChanCCR (0, sixStep.pulse_value, 0);
      mcNucleoEnableInputChan12();
      sixStep.mBemfIndex = 2;
     break;

    case 5:
      mcNucleoSetChanCCR (0, 0, sixStep.pulse_value);
      mcNucleoEnableInputChan13();
      sixStep.mBemfIndex = 1;
      break;

    case 6:
      mcNucleoSetChanCCR (0, 0, sixStep.pulse_value);
      mcNucleoEnableInputChan23();
      sixStep.mBemfIndex = 0;
      break;
     }
  }
//}}}
//{{{
void rampMotor() {

  uint32_t constant_multiplier = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (mMotorStartupCount == 1) {
    mech_accel_hz = sixStep.ACCEL * sixStep.mNumPolePair/ 60;
    constant_multiplier_tmp = (uint64_t)constant_multiplier * (uint64_t)constant_multiplier_2;
    constant_k = constant_multiplier_tmp / (3 * mech_accel_hz);
    mcNucleoCurrentRefSetValue (sixStep.mStartupCurrent);
    Time_vector_prev_tmp = 0;
    }

  if (mMotorStartupCount < NUMBER_OF_STEPS) {
    Time_vector_tmp = ((uint64_t)1000 * (uint64_t)1000 * (uint64_t) fastSqrt(((uint64_t)mMotorStartupCount * (uint64_t)constant_k)))/632455;
    delta = Time_vector_tmp - Time_vector_prev_tmp;
    if (mMotorStartupCount == 1) {
      T_single_step_first_value = (2 * 3141) * delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535);
      }
    else {
      T_single_step = (2 * 3141)* delta / 1000;
      sixStep.ARR_value = (uint32_t)(65535 * T_single_step) / T_single_step_first_value;
      }
    }
  else
    mMotorStartupCount = 1;

  if (mMotorStartupCount == 1)
    sixStep.prescaler_value = (((sixStep.SYSCLK_frequency / 1000000) * T_single_step_first_value) / 65535) - 1;

  if ((sixStep.STATUS != ALIGNMENT) && (sixStep.STATUS != START))
    mMotorStartupCount++;
  else
    Time_vector_tmp = 0;

  Time_vector_prev_tmp = Time_vector_tmp;
  }
//}}}
//{{{
void arrStep() {

  if (!sixStep.mAligning)
    sixStep.mAligning = true;

  if (sixStep.mAligned) {
    if (sixStep.VALIDATION_OK) {
      sixStep.ARR_OK = true;
      mMotorStartupCount = 1;
      index_ARR_step = 1;
      }
    else {
      sixStep.STATUS = STARTUP;
      rampMotor();
      if (index_ARR_step < sixStep.numberofitemArr) {
        hTim6.Init.Period = sixStep.ARR_value;
        hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;
        index_ARR_step++;
        }
      else if (!sixStep.ARR_OK) {
        index_ARR_step = 1;
        sixStep.ACCEL >>= 1;
        if (sixStep.ACCEL < MINIMUM_ACC)
          sixStep.ACCEL = MINIMUM_ACC;

        sixStep.STATUS = STARTUP_FAILURE;
        mcStopMotor();
        printf ("startup failure\n");
        gStateString = "startup failure";
        }
      }
    }
  }
//}}}
//{{{
void arrBemf (bool up) {

  if (sixStep.mPrevStep != sixStep.mStep) {
    sixStep.mPrevStep = sixStep.mStep;

    if (sixStep.SPEED_VALIDATED) {
      if (mOpenLoopBemfEvent > BEMF_CNT_EVENT_MAX)
        mOpenLoopBemfFailure = true;

      if (up && !sixStep.BEMF_OK) {
        n_zcr_startup++;
        mOpenLoopBemfEvent = 0;
        }
      else if (!sixStep.BEMF_OK)
        mOpenLoopBemfEvent++;

      if ((n_zcr_startup >= NUMBER_ZCR) && !sixStep.BEMF_OK) {
        sixStep.BEMF_OK = true;
        n_zcr_startup = 0;
        }
      }

    if (sixStep.VALIDATION_OK) {
      counter_ARR_Bemf = __HAL_TIM_GetCounter (&hTim6);
      __HAL_TIM_SetAutoreload (&hTim6, counter_ARR_Bemf + ARR_LF / 2);

      }
    }
  }
//}}}

//{{{
void setPiParam (cPiParam* piParam) {

  piParam->Reference = sixStep.CW_CCW ? -target_speed : target_speed;

  piParam->Kp_Gain = sixStep.KP;
  piParam->Ki_Gain = sixStep.KI;

  piParam->Lower_Limit_Output = LOWER_OUT_LIMIT;
  piParam->Upper_Limit_Output = UPPER_OUT_LIMIT;

  piParam->Max_PID_Output = false;
  piParam->Min_PID_Output = false;
  }
//}}}
//{{{
int16_t piController (cPiParam* PI_PARAM, int16_t speed_fdb) {

  int32_t Error = PI_PARAM->Reference - speed_fdb;

  // Proportional term computation
  int32_t wProportional_Term = PI_PARAM->Kp_Gain * Error;

  // Integral term computation
  int32_t wIntegral_Term = 0;
  int32_t wIntegral_sum_temp = 0;
  if (PI_PARAM->Ki_Gain == 0)
    sixStep.Integral_Term_sum = 0;
  else {
    wIntegral_Term = PI_PARAM->Ki_Gain * Error;
    wIntegral_sum_temp = sixStep.Integral_Term_sum + wIntegral_Term;
    sixStep.Integral_Term_sum = wIntegral_sum_temp;
    }

  if (sixStep.Integral_Term_sum > KI_DIV * PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = KI_DIV * PI_PARAM->Upper_Limit_Output;

  if (sixStep.Integral_Term_sum < -KI_DIV * PI_PARAM->Upper_Limit_Output)
    sixStep.Integral_Term_sum = -KI_DIV * PI_PARAM->Upper_Limit_Output;

  // WARNING: the below instruction is not MISRA compliant, user should verify
  //          that Cortex-M3 assembly instruction ASR (arithmetic shift right)
  //          is used by the compiler to perform the shifts (instead of LSR logical shift right)
  int32_t wOutput_32 = (wProportional_Term / KP_DIV) + (sixStep.Integral_Term_sum / KI_DIV);

  if (PI_PARAM->Reference > 0) {
    if (wOutput_32 > PI_PARAM->Upper_Limit_Output)
      wOutput_32 = PI_PARAM->Upper_Limit_Output;
    else if (wOutput_32 < PI_PARAM->Lower_Limit_Output)
      wOutput_32 = PI_PARAM->Lower_Limit_Output;
    }
  else {
    if (wOutput_32 < -PI_PARAM->Upper_Limit_Output)
      wOutput_32 = -PI_PARAM->Upper_Limit_Output;
    else if (wOutput_32 > -PI_PARAM->Lower_Limit_Output)
      wOutput_32 = -PI_PARAM->Lower_Limit_Output;
    }

  return (int16_t)wOutput_32;
  }
//}}}

//{{{
void bemfDelayCalc (uint16_t piReference) {

 if (piReference >= 0) {
   if (sixStep.speed_fdbk_filtered <= 12000 && sixStep.speed_fdbk_filtered > 10000)
      sixStep.demagn_value = DEMAGN_VAL_1;
    else if (sixStep.speed_fdbk_filtered <= 10000 && sixStep.speed_fdbk_filtered > 7800)
      sixStep.demagn_value = DEMAGN_VAL_2;
    else if (sixStep.speed_fdbk_filtered <= 7800 && sixStep.speed_fdbk_filtered > 6400)
      sixStep.demagn_value = DEMAGN_VAL_3;
    else if (sixStep.speed_fdbk_filtered <= 6400 && sixStep.speed_fdbk_filtered > 5400)
      sixStep.demagn_value = DEMAGN_VAL_4;
    else if (sixStep.speed_fdbk_filtered <= 5400 && sixStep.speed_fdbk_filtered > 4650)
      sixStep.demagn_value = DEMAGN_VAL_5;
    else if (sixStep.speed_fdbk_filtered <= 4650 && sixStep.speed_fdbk_filtered > 4100)
      sixStep.demagn_value = DEMAGN_VAL_6;
    else if (sixStep.speed_fdbk_filtered <= 4100 && sixStep.speed_fdbk_filtered > 3650)
      sixStep.demagn_value = DEMAGN_VAL_7;
    else if (sixStep.speed_fdbk_filtered <= 3650 && sixStep.speed_fdbk_filtered > 3300)
      sixStep.demagn_value = DEMAGN_VAL_8;
    else if (sixStep.speed_fdbk_filtered <= 3300 && sixStep.speed_fdbk_filtered > 2600)
      sixStep.demagn_value = DEMAGN_VAL_9;
    else if (sixStep.speed_fdbk_filtered <= 2600 && sixStep.speed_fdbk_filtered > 1800)
      sixStep.demagn_value = DEMAGN_VAL_10;
    else if (sixStep.speed_fdbk_filtered <= 1800 && sixStep.speed_fdbk_filtered > 1500)
      sixStep.demagn_value = DEMAGN_VAL_11;
    else if (sixStep.speed_fdbk_filtered <= 1500 && sixStep.speed_fdbk_filtered > 1300)
      sixStep.demagn_value = DEMAGN_VAL_12;
    else if (sixStep.speed_fdbk_filtered <= 1300 && sixStep.speed_fdbk_filtered > 1000)
      sixStep.demagn_value = DEMAGN_VAL_13;
    else if (sixStep.speed_fdbk_filtered <= 1000 && sixStep.speed_fdbk_filtered > 500)
      sixStep.demagn_value = DEMAGN_VAL_14;
     }
   else {
    if (sixStep.speed_fdbk_filtered >= -12000 && sixStep.speed_fdbk_filtered < -10000)
      sixStep.demagn_value = DEMAGN_VAL_1;
    else if (sixStep.speed_fdbk_filtered >= -10000 && sixStep.speed_fdbk_filtered < -7800)
      sixStep.demagn_value = DEMAGN_VAL_2;
    else if (sixStep.speed_fdbk_filtered >= -7800 && sixStep.speed_fdbk_filtered < -6400)
      sixStep.demagn_value = DEMAGN_VAL_3;
    else if (sixStep.speed_fdbk_filtered >= -6400 && sixStep.speed_fdbk_filtered < -5400)
      sixStep.demagn_value = DEMAGN_VAL_4;
    else if (sixStep.speed_fdbk_filtered >= -5400 && sixStep.speed_fdbk_filtered < -4650)
      sixStep.demagn_value = DEMAGN_VAL_5;
    else if (sixStep.speed_fdbk_filtered >= -4650 && sixStep.speed_fdbk_filtered < -4100)
      sixStep.demagn_value = DEMAGN_VAL_6;
    else if (sixStep.speed_fdbk_filtered >= -4100 && sixStep.speed_fdbk_filtered < -3650)
      sixStep.demagn_value = DEMAGN_VAL_7;
    else if (sixStep.speed_fdbk_filtered >= -3650 && sixStep.speed_fdbk_filtered < -3300)
      sixStep.demagn_value = DEMAGN_VAL_8;
    else if (sixStep.speed_fdbk_filtered >= -3300 && sixStep.speed_fdbk_filtered < -2650)
      sixStep.demagn_value = DEMAGN_VAL_9;
    else if (sixStep.speed_fdbk_filtered >= -2600 && sixStep.speed_fdbk_filtered <-1800)
      sixStep.demagn_value = DEMAGN_VAL_10;
    else if (sixStep.speed_fdbk_filtered >= -1800 && sixStep.speed_fdbk_filtered < -1500)
      sixStep.demagn_value = DEMAGN_VAL_11;
    else if (sixStep.speed_fdbk_filtered >= -1500 && sixStep.speed_fdbk_filtered < -1300)
      sixStep.demagn_value = DEMAGN_VAL_12;
    else if (sixStep.speed_fdbk_filtered >= -1300 && sixStep.speed_fdbk_filtered < -1000)
      sixStep.demagn_value = DEMAGN_VAL_13;
    else if (sixStep.speed_fdbk_filtered >= -1000 && sixStep.speed_fdbk_filtered < -500)
      sixStep.demagn_value = DEMAGN_VAL_14;
    }
  }
//}}}
//{{{
void taskSpeed() {

  if (!sixStep.VALIDATION_OK &&
      ((sixStep.speed_fdbk_filtered > (target_speed)) || (sixStep.speed_fdbk_filtered < (-target_speed)))) {
    //printf ("taskSpeed SPEED_VALIDATED\n");
    sixStep.STATUS = VALIDATION;
    sixStep.SPEED_VALIDATED = true;
    }

  if (sixStep.SPEED_VALIDATED && sixStep.BEMF_OK && !sixStep.mClosedLoopReady)
    sixStep.mClosedLoopReady = true;

  if (sixStep.VALIDATION_OK) {
    //printf ("taskSpeed VALIDATION_OK\n");
    sixStep.STATUS = RUN;
    uint16_t ref = (uint16_t)piController (&piParam,(int16_t)sixStep.speed_fdbk_filtered);
    if (piParam.Reference < 0)
      ref = -ref;

    //printf ("ref %d\n", ref);
    sixStep.Current_Reference = ref;
    mcNucleoCurrentRefSetValue (sixStep.Current_Reference);
    }

  bemfDelayCalc (piParam.Reference);
  }
//}}}

// callback interface
//{{{
void mcAdcSample (ADC_HandleTypeDef* hAdc) {

  uint16_t value = HAL_ADC_GetValue (hAdc);

  if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
    mValues0[mValueIndex++ % NUM_VALUES] = sixStep.mBemfIndex == 0 ? value/16 : 0;
    mValues1[mValueIndex++ % NUM_VALUES] = sixStep.mBemfIndex == 1 ? value/16 : 0;
    mValues2[mValueIndex++ % NUM_VALUES] = sixStep.mBemfIndex == 2 ? value/16 : 0;

    // tim1 pwm up counting
    if ((sixStep.STATUS != START) && (sixStep.STATUS != ALIGNMENT)) {
      switch (sixStep.mStep) {
        //{{{
        case 1:
          sixStep.mBemfInputBuffer[2] = value;

          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else if (value > sixStep.ADC_BEMF_threshold_UP) {
            arrBemf (1);
            sixStep.BEMF_Tdown_count = 0;
            }

          break;
        //}}}
        //{{{
        case 2:
          sixStep.mBemfInputBuffer[1] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else if (value < sixStep.ADC_BEMF_threshold_DOWN)
            arrBemf (0);

          break;
        //}}}
        //{{{
        case 3:
          sixStep.mBemfInputBuffer[0] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else if (value > sixStep.ADC_BEMF_threshold_UP) {
            arrBemf (1);
            sixStep.BEMF_Tdown_count = 0;
            }

          break;
        //}}}
        //{{{
        case 4:
          sixStep.mBemfInputBuffer[2] = value;

          if (sixStep.demagn_counter >= sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
           else if (value < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);

         break;
        //}}}
        //{{{
        case 5:
          sixStep.mBemfInputBuffer[1] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value < sixStep.ADC_BEMF_threshold_DOWN)
              arrBemf (0);
            }
          else if (value > sixStep.ADC_BEMF_threshold_UP) {
            arrBemf (1);
            sixStep.BEMF_Tdown_count = 0;
            }

         break;
        //}}}
        //{{{
        case 6:
          sixStep.mBemfInputBuffer[0] = value;
          if (sixStep.demagn_counter < sixStep.demagn_value)
            sixStep.demagn_counter++;
          else if (piParam.Reference >= 0) {
            if (value > sixStep.ADC_BEMF_threshold_UP) {
              arrBemf (1);
              sixStep.BEMF_Tdown_count = 0;
              }
            }
          else if (value < sixStep.ADC_BEMF_threshold_DOWN)
             arrBemf (0);
          break;
        //}}}
        }
      //printf ("%d adcU %d\n", HAL_GetTick(), sixStep.mStep);
      }

    // set adc chan for next curr/temp/vbus/temp reading
    mcNucleoAdcChan (sixStep.mAdcInputAdc[sixStep.mAdcIndex], sixStep.mAdcInputChan[sixStep.mAdcIndex]);
    }

  else {
    // tim1 pwm down counting
    if (sixStep.mAdcIndex == 1) {
      mPotValues[mPotValueIndex] = value;
      mPotValueIndex = (mPotValueIndex+1) % POT_VALUES_SIZE;
      }
    sixStep.mAdcBuffer[sixStep.mAdcIndex] = value;
    sixStep.mAdcIndex = (sixStep.mAdcIndex+1) % 4;

    // set adc chan for next bemf ADC reading
    mcNucleoAdcChan (sixStep.mBemfInputAdc[sixStep.mBemfIndex], sixStep.mBemfInputChan[sixStep.mBemfIndex]);
    }
  }
//}}}
//{{{
void mcTim6Tick() {

  //printf ("mcTim6Tick %d\n", HAL_GetTick());
  // nextStep
  if (!sixStep.mPmwRunning) {
    mcNucleoStartPwm();
    sixStep.mPmwRunning = true;
    }
  ARR_LF = __HAL_TIM_GetAutoreload (&hTim6);

  if (sixStep.mAligned) {
    sixStep.speed_fdbk = mcGetMechSpeedRPM();
    sixStep.demagn_counter = 1;
    if (sixStep.mPrevStep != sixStep.mStep)
      n_zcr_startup = 0;

    if (piParam.Reference >= 0) {
      sixStep.mStep++;
      if (sixStep.mStep > 6)
        sixStep.mStep = 1;
      if (sixStep.mClosedLoopReady)
        sixStep.VALIDATION_OK = true;
      }
    else {
      sixStep.mStep--;
      if (sixStep.mStep < 1)
        sixStep.mStep = 6;
      if (sixStep.mClosedLoopReady)
        sixStep.VALIDATION_OK = true;
      }
    }

  if (sixStep.VALIDATION_OK) {
    // motorStall detection
    if (sixStep.BEMF_Tdown_count++ > BEMF_CONSEC_DOWN_MAX)
      mSpeedFeedbackFailure = true;
    else
      __HAL_TIM_SetAutoreload (&hTim6, 0xFFFF);
    }

  sixStepTable (sixStep.mStep);
  if (__HAL_TIM_DIRECTION_STATUS (&hTim1)) {
    // step request during downCount, change adc Chan
    mcNucleoAdcChan (sixStep.mBemfInputAdc[sixStep.mBemfIndex], sixStep.mBemfInputChan[sixStep.mBemfIndex]);
    //printf ("step request during downCount\n");
    }

  if (!sixStep.ARR_OK) // STEP frequency changing
    arrStep();

  speedFilter();
  }
//}}}
//{{{
void mcSysTick() {

  if (sixStep.mAligning && !sixStep.mAligned) {
    //{{{  align motor
    sixStep.STATUS = ALIGNMENT;
    sixStep.mStep = 6;

    hTim6.Init.Period = sixStep.ARR_value;
    hTim6.Instance->ARR = (uint32_t)hTim6.Init.Period;

    mAlignTicks++;
    if (mAlignTicks >= TIME_FOR_ALIGN + 1) {
      printf ("%d aligned\n", HAL_GetTick());
      gStateString = "aligned";
      sixStep.mAligned = true;
      sixStep.STATUS = STARTUP;
      hTim6.Init.Prescaler = sixStep.prescaler_value;
      hTim6.Instance->PSC = hTim6.Init.Prescaler;
      }

    potSpeedTarget();
    }
    //}}}

  if (sixStep.VALIDATION_OK)
    potSpeed();
  if (sixStep.STATUS != SPEED_FEEDBACK_FAILURE)
    taskSpeed();
  if (sixStep.VALIDATION_OK)
    mcSetSpeed();

  if (mOpenLoopBemfFailure) {
    sixStep.ACCEL >>= 1;
    if (sixStep.ACCEL < MINIMUM_ACC)
      sixStep.ACCEL = MINIMUM_ACC;
    mcStopMotor();

    sixStep.STATUS = STARTUP_BEMF_FAILURE;
    printf ("startup Bemf failure\n");
    gStateString = "startup Bemf failure";
    mOpenLoopBemfEvent = 0;
    }

  if (mSpeedFeedbackFailure) {
    mcStopMotor();
    sixStep.STATUS = SPEED_FEEDBACK_FAILURE;
    printf ("speed feedback failure\n");
    gStateString = "speed feedback failure";
    }
  }
//}}}

// external interface
//{{{
void mcInit() {

  mcNucleoInit();

  sixStep.HF_TIMx_ARR = hTim1.Instance->ARR;
  sixStep.HF_TIMx_PSC = hTim1.Instance->PSC;
  sixStep.HF_TIMx_CCR = hTim1.Instance->CCR1;

  sixStep.LF_TIMx_ARR = hTim6.Instance->ARR;
  sixStep.LF_TIMx_PSC = hTim6.Instance->PSC;

  sixStep.mStartupCurrent = STARTUP_CURRENT_REFERENCE;
  sixStep.mNumPolePair = NUM_POLE_PAIR;

  sixStep.ACCEL = ACC;
  sixStep.KP = KP_GAIN;
  sixStep.KI = KI_GAIN;
  sixStep.CW_CCW = TARGET_SPEED < 0;

  mcReset();
  }
//}}}
//{{{
void mcReset() {

  mLastButtonPress = 0;

  sixStep.mMotorRunning = false;
  sixStep.mPmwRunning = false;
  sixStep.mAligning = false;
  sixStep.mAligned = false;
  sixStep.VALIDATION_OK = false;
  sixStep.ARR_OK = false;
  sixStep.BEMF_OK = false;
  sixStep.mClosedLoopReady = false;
  sixStep.SPEED_VALIDATED = false;

  sixStep.numberofitemArr = NUMBER_OF_STEPS;
  sixStep.mStartupCurrent = STARTUP_CURRENT_REFERENCE;

  sixStep.pulse_value = sixStep.HF_TIMx_CCR;
  sixStep.Speed_target_ramp = MAX_POT_SPEED;
  sixStep.Speed_Ref_filtered = 0;
  sixStep.demagn_value = INITIAL_DEMAGN_DELAY;

  hTim1.Init.Prescaler = sixStep.HF_TIMx_PSC;
  hTim1.Instance->PSC =  sixStep.HF_TIMx_PSC;
  hTim1.Init.Period =    sixStep.HF_TIMx_ARR;
  hTim1.Instance->ARR =  sixStep.HF_TIMx_ARR;
  hTim1.Instance->CCR1 = sixStep.HF_TIMx_CCR;

  hTim6.Init.Prescaler = sixStep.LF_TIMx_PSC;
  hTim6.Instance->PSC =  sixStep.LF_TIMx_PSC;
  hTim6.Init.Period =    sixStep.LF_TIMx_ARR;
  hTim6.Instance->ARR =  sixStep.LF_TIMx_ARR;

  sixStep.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

  mcNucleoSetChanCCR (0,0,0);

  // PC1 -> ADC12_IN7  curr_fdbk2 - 1shunt
  // PB1 -> ADC3_IN1   pot
  // PA1 -> ADC1_IN2   vbus
  // PC2 -> ADC12_IN8  temp
  sixStep.mAdcIndex = 0;
  sixStep.mAdcInputAdc[0] = &hAdc1;
  sixStep.mAdcInputChan[0] = ADC_CHANNEL_7;
  sixStep.mAdcInputAdc[1] = &hAdc3;
  sixStep.mAdcInputChan[1] = ADC_CHANNEL_1;
  sixStep.mAdcInputAdc[2] = &hAdc1;
  sixStep.mAdcInputChan[2] = ADC_CHANNEL_2;
  sixStep.mAdcInputAdc[3] = &hAdc1;
  sixStep.mAdcInputChan[3] = ADC_CHANNEL_8;

  // PC3 -> ADC12_IN9  bemf1
  // PB0 -> ADC3_IN12  bemf2
  // PA7 -> ADC2_IN4   bemf3
  sixStep.mBemfIndex = 0;
  sixStep.mBemfInputAdc[0] = &hAdc1;
  sixStep.mBemfInputChan[0] = ADC_CHANNEL_9;
  sixStep.mBemfInputAdc[1] = &hAdc3;
  sixStep.mBemfInputChan[1] = ADC_CHANNEL_12;
  sixStep.mBemfInputAdc[2] = &hAdc2;
  sixStep.mBemfInputChan[2] = ADC_CHANNEL_4;

  sixStep.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
  sixStep.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
  sixStep.demagn_counter = 0;

  sixStep.speed_fdbk_filtered = 0;
  sixStep.Integral_Term_sum = 0;
  sixStep.Current_Reference = 0;
  sixStep.speed_fdbk = 0;

  sixStep.BEMF_Tdown_count = 0;   // Reset of the Counter to detect Stop motor condition when a stall condition occurs

  T_single_step = 0;
  T_single_step_first_value = 0;
  Time_vector_tmp = 0;
  Time_vector_prev_tmp = 0;
  delta = 0;

  Mech_Speed_RPM = 0;
  El_Speed_Hz = 0;
  mech_accel_hz = 0;

  constant_k = 0;
  ARR_LF = 0;
  index_array = 1;

  mOpenLoopBemfEvent = 0;
  mOpenLoopBemfFailure = false;
  mSpeedFeedbackFailure = false;

  index_ARR_step = 1;
  n_zcr_startup = 0;

  mAlignTicks = 1;
  speed_sum_sp_filt = 0;
  speed_sum_pot_filt = 0;
  index_pot_filt = 1;
  potent_filtered = 0;
  counter_ARR_Bemf = 0;
  constant_multiplier_tmp = 0;

  mPotValueIndex = 0;
  for (uint16_t i = 0; i < POT_VALUES_SIZE; i++)
    mPotValues[i] = 0;

  array_completed = false;
  buffer_completed = false;
  for (uint16_t i = 0; i < FILTER_DEEP;i++) {
    speed_tmp_array[i] = 0;
    speed_tmp_buffer[i]= 0;
    }

  sixStep.mPrevStep = 0;
  if (piParam.Reference < 0)
    sixStep.mStep = 1;
  else
    sixStep.mStep = 0;

  target_speed = TARGET_SPEED;
  setPiParam (&piParam);

  mcNucleoCurrentRefStart();
  mcNucleoCurrentRefSetValue (sixStep.mStartupCurrent);

  mMotorStartupCount = 1;
  rampMotor();
  }
//}}}

//{{{
int32_t mcGetElSpeedHz() {

  if (__HAL_TIM_GetAutoreload (&hTim6) != 0xFFFF)
    El_Speed_Hz = (int32_t)((sixStep.SYSCLK_frequency) / hTim6.Instance->PSC) / (__HAL_TIM_GetAutoreload (&hTim6) * 6);
  else
    El_Speed_Hz = 0;

  return piParam.Reference < 0 ? -El_Speed_Hz : El_Speed_Hz;
  }
//}}}
//{{{
int32_t mcGetMechSpeedRPM() {

  Mech_Speed_RPM = (int32_t)(mcGetElSpeedHz() *  60 / sixStep.mNumPolePair);
  return (Mech_Speed_RPM);
  }
//}}}

//{{{
void mcStartMotor() {

  sixStep.STATUS = START;
  sixStep.mMotorRunning = true;

  HAL_TIM_Base_Start_IT (&hTim6);
  HAL_ADC_Start_IT (&hAdc1);

  mcNucleoLedOn();
  }
//}}}
//{{{
void mcStopMotor() {

  sixStep.STATUS = STOP;
  sixStep.mMotorRunning = false;

  mcNucleoStopPwm();
  hTim1.Instance->CR1 &= ~(TIM_CR1_CEN);
  hTim1.Instance->CNT = 0;
  mcNucleoDisableChan();

  HAL_TIM_Base_Stop_IT (&hTim6);
  HAL_ADC_Stop_IT (&hAdc1);

  mcNucleoCurrentRefStop();
  mcNucleoLedOff();

  mcReset();
  }
//}}}
//{{{
void mcPanic() {

  mcStopMotor();
  sixStep.STATUS = OVERCURRENT_FAILURE;
  printf ("mcPanic\n");
  gStateString = "mcPanic";
  }
//}}}

//{{{
void mcSetSpeed() {

  int16_t reference_tmp = 0;

  bool change_target_speed = false;
  if (sixStep.Speed_Ref_filtered > sixStep.Speed_target_ramp) {
    if ((sixStep.Speed_Ref_filtered - sixStep.Speed_target_ramp) > ADC_SPEED_TH)
      change_target_speed = true;
    }
  else if ((sixStep.Speed_target_ramp - sixStep.Speed_Ref_filtered) > ADC_SPEED_TH)
    change_target_speed = true;

  if (change_target_speed) {
    sixStep.Speed_target_ramp = sixStep.Speed_Ref_filtered;

    if (sixStep.CW_CCW) {
      reference_tmp = -(sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096);
      piParam.Reference = (reference_tmp >=- MIN_POT_SPEED) ? -MIN_POT_SPEED : reference_tmp;
      }
    else {
      reference_tmp = sixStep.Speed_Ref_filtered * MAX_POT_SPEED / 4096;
      piParam.Reference = (reference_tmp <= MIN_POT_SPEED) ? MIN_POT_SPEED : reference_tmp;
      }
    }
  }
//}}}
//{{{
void mcEXTbutton() {

  if (HAL_GetTick() > mLastButtonPress + 200) {
    printf ("mcEXTbutton toggle %d\n", HAL_GetTick() - mLastButtonPress);
    mLastButtonPress = HAL_GetTick();
    if (sixStep.mMotorRunning) {
      printf ("StopMotor\n");
      mcStopMotor();
      }
    else {
      printf ("StartMotor\n");
      mcStartMotor();
      }
    }
  }
//}}}

// callbacks
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) {
  mcAdcSample (hadc);
  }
//}}}
//{{{
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim) {
  if (htim == &hTim6)
    mcTim6Tick();
  }
//}}}
//{{{
void HAL_SYSTICK_Callback() {
  mcSysTick();
  }
//}}}
//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
  mcEXTbutton();
  }
//}}}

//{{{
void SystemClock_Config() {

  // init osc
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);

  // init CPU, AHB and APB busses clocks
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2);

  // init periph clocks
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_ADC12 |
                                       RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit);

  // enable Clock Security System
  HAL_RCC_EnableCSS();

  // config Systick interrupt time
  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq() / 1000);

  // config Systick
  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority (SysTick_IRQn, 4, 0);
  }
//}}}
//{{{
int main() {

  HAL_Init();
  SystemClock_Config();

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority (MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority (BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority (DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority (PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority (SysTick_IRQn, 2, 0);

  mcInit();

  lcd.init();

  mValues0 = (uint8_t*)malloc (NUM_VALUES);
  mValues1 = (uint8_t*)malloc (NUM_VALUES);
  mValues2 = (uint8_t*)malloc (NUM_VALUES);
  memset (mValues0, 0, NUM_VALUES);
  memset (mValues1, 0, NUM_VALUES);
  memset (mValues2, 0, NUM_VALUES);

  while (1) {
    lcd.clear (cLcd::eOn);
    //printf ("%d %d %d i:%d p:%d v:%d t:%d 1:%d 2:%d 3:%d\n",
    //        ARR_LF, counter_ARR_Bemf, piParam.Reference,
    //        sixStep.mAdcBuffer[0], sixStep.mAdcBuffer[1] ,sixStep.mAdcBuffer[2] ,sixStep.mAdcBuffer[3],
    //        sixStep.mBemfInputBuffer[0], sixStep.mBemfInputBuffer[1] ,sixStep.mBemfInputBuffer[2]);

    lcd.drawString (cLcd::eOff, cLcd::eBig, cLcd::eLeft,
                    dec (sixStep.mAdcBuffer[0], 4) + " " +
                    dec (sixStep.mAdcBuffer[1], 4) + " " +
                    dec (sixStep.mAdcBuffer[2], 4) + " " +
                    dec (sixStep.mAdcBuffer[3], 4),
                    cPoint(0,0));

    lcd.drawString (cLcd::eOff, cLcd::eBig, cLcd::eLeft,
                    dec (sixStep.mBemfInputBuffer[0], 4) + " " +
                    dec (sixStep.mBemfInputBuffer[1], 4) + " " +
                    dec (sixStep.mBemfInputBuffer[2], 4),
                    cPoint(0,40));

    lcd.drawString (cLcd::eOff, cLcd::eBig, cLcd::eLeft, gStateString, cPoint(0,80));

    int32_t valueIndex = mValueIndex - lcd.getWidth();
    for (int i = 0; i < lcd.getWidth(); i++) {
      int16_t height = (lcd.getHeight()/3) - 1;

      int32_t value = 0;
      for (int j = 0; j < AVERAGE_VALUES; j++)
        value += valueIndex+j > 0 ? mValues0[(valueIndex+j) % NUM_VALUES] : 0;
      value /= AVERAGE_VALUES * 2;
      lcd.fillRect (cLcd::eOff, cRect (i, height - value, i+1, height));

      height += (lcd.getHeight()/3) -1;
      value = 0;
      for (int j = 0; j < AVERAGE_VALUES; j++)
        value += valueIndex+j > 0 ? mValues1[(valueIndex+j) % NUM_VALUES] : 0;
      value /= AVERAGE_VALUES * 2;
      lcd.fillRect (cLcd::eOff, cRect (i, height - value, i+1, height));

      height += (lcd.getHeight()/3);
      value = 0;
      for (int j = 0; j < AVERAGE_VALUES; j++)
        value += valueIndex+j > 0 ? mValues2[(valueIndex+j) % NUM_VALUES] : 0;
      value /= AVERAGE_VALUES * 2;
      lcd.fillRect (cLcd::eOff, cRect (i, height - value, i+1, height));

      valueIndex += AVERAGE_VALUES;
      }
    lcd.present();
    }
  }
//}}}
