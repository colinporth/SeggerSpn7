// cTrace.h
#pragma once
#include <vector>
#include "cLcd.h"

class cTrace {
public:
  //{{{
  cTrace (int16_t numSamples, int16_t averageSamples)
     : mNumSamples(numSamples), mAverageSamples(averageSamples) {
    mSamples = (uint8_t*)malloc (numSamples);
    memset (mSamples, 0, numSamples);
    }
  //}}}

  //{{{
  void addSample (uint8_t value) {

    if (value < mMin)
      mMin = value;
    if (value > mMax)
      mMax = value;

    mSamples[mCurSample++ % mNumSamples] = value;
    }
  //}}}
  //{{{
  void draw (cLcd* lcd, int16_t y, int16_t height) {

    int16_t range = mMax - mMin;

    for (int i = 0; i < lcd->getWidth(); i++) {
      uint32_t value = 0;
      for (int j = 0; j < mAverageSamples; j++)
        value += mCurSample+j > 0 ? mSamples[(mCurSample+j) % mNumSamples] : 0;
      value = (value * height) / (range * mAverageSamples);

      lcd->fillRect (cLcd::eOff, cRect (i, y - value, i+1, y));
      }
    }
  //}}}

private:
  int16_t mNumSamples = 0;
  int16_t mAverageSamples = 1;
  uint8_t* mSamples = nullptr;
  uint32_t mCurSample = 0;
  uint8_t mMin = 255;
  uint8_t mMax = 0;
  };

class cTraceVec {
public:
  //{{{
  void addTrace (int16_t numSamples, int16_t averageSamples) {

    mTraces.push_back (new cTrace (numSamples, averageSamples));
    }
  //}}}
  //{{{
  void addSample (int16_t trace, uint8_t value) {
    mTraces[trace]->addSample (value);
    }
  //}}}
  //{{{
  void draw (cLcd* lcd) {

    int16_t height = lcd->getHeight() / mTraces.size();
    int16_t y = height;
    for (auto trace : mTraces) {
      trace->draw (lcd, y, height);
      y += height;
      }
    }
  //}}}

private:
  std::vector<cTrace*> mTraces;
  };
