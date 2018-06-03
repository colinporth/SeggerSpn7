#pragma once
#include <stdint.h>

struct fontChar_t {
  uint8_t width;
  uint8_t height;
  int8_t  left;
  int8_t  top;
  uint8_t advance;
  };

struct font_t {
  uint8_t fixedWidth;
  uint8_t height;
  uint8_t spaceWidth;
  uint8_t lineSpacing;
  uint8_t firstChar;
  uint8_t lastChar;
  const uint8_t* glyphsBase;
  const uint16_t* glyphOffsets;
  };

extern const struct font_t font18;
extern const struct font_t font36;
