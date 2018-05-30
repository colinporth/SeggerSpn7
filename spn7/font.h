#pragma once
#include <stdint.h>

struct fontChar_t {
  uint8_t width;
  uint8_t height;
  int8_t  left;
  int8_t  top;
  uint8_t advance;
  };

typedef struct font {
  uint8_t fixedWidth;
  uint8_t height;
  uint8_t spaceWidth;
  uint8_t lineSpacing;
  uint8_t firstChar;
  uint8_t lastChar;
  const uint8_t* glyphsBase;
  const uint16_t* glyphOffsets;
} font_t;

extern const font_t font18;
extern const font_t font36;
extern const font_t font72;
extern const font_t font120;
