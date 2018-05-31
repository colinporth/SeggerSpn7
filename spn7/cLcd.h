// cLcd.h
#include "cPointRect.h"
#include "utils.h"
#include "stm32f3xx_hal.h"

struct font_t;
class cLcd {
public:
  enum eDraw { eInvert, eOff, eOn };
  enum eFont { eSmall, eBig };
  enum eAlign { eLeft, eCentre, eRight };

  bool init();

  const uint16_t getWidth() { return kWidth; }
  const uint16_t getHeight() { return kHeight; }
  const cPoint getSize() { return cPoint (kWidth, kHeight); }
  const cRect getRect() { return cRect (getSize()); }
  const cPoint getCentre() { return getSize()/2; }

  uint32_t getFrameNum() { return mFrameNum; }
  uint32_t getTookTicks() { return mTookTicks; }

  void clear (eDraw draw);
  void fillRect (eDraw draw, cRect rect);
  void fillRect (eDraw draw, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
  void drawLine (eDraw draw, cPoint p1, cPoint p2);
  void drawPolygon (eDraw draw, cPoint* points, uint16_t numPoints);
  void drawCircle (eDraw draw, cPoint centre, int16_t radius);
  void drawEllipse (eDraw draw, cPoint centre, cPoint radius);
  void fillTriangle (eDraw draw, cPoint p1, cPoint p2, cPoint p3);
  void fillPolygon (eDraw draw, cPoint* points, uint16_t numPoints);
  void fillCircle (eDraw draw, cPoint centre, uint16_t radius);
  void fillEllipse (eDraw draw, cPoint centre, cPoint radius);
  void drawString (eDraw draw, eFont font, eAlign align, const std::string& str, cPoint p);
  void present();

  // for irqs
  SPI_HandleTypeDef* getSpiHandle() { return &mSpiHandle; }

  static cLcd* mLcd;

private:
  const uint16_t kWidth = 400;
  const uint16_t kHeight = 240;

  uint16_t getPitch() { return 2 + getWidth()/8; }
  uint8_t* getFramePtr (int16_t y) { return mFrameBuf + 2 + y*getPitch(); }
  uint16_t getCharWidth (const font_t* font, char ascii);

  void drawPix (eDraw draw, cPoint p);

  // frameBuf - commandByte | 240 * (lineAddressByte | 50bytes,400pixels | paddingByte0) | paddingByte0
  uint8_t* mFrameBuf = nullptr;
  uint32_t mFrameNum = 0;
  uint32_t mPresentTicks = 0;
  uint32_t mTookTicks = 0;

  SPI_HandleTypeDef mSpiHandle;
  DMA_HandleTypeDef mSpiTxDma;
  };
