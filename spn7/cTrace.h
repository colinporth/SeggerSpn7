// cTrace.h
#pragma once
#include <vector>
#include "cLcd.h"

class cTrace {
public:
  //{{{
  cTrace (int numSamples, int averageSamples)
     : mNumSamples(numSamples), mAverageSamples(averageSamples) {
    mSamples = (uint8_t*)malloc (numSamples);
    memset (mSamples, 0, numSamples);
    }
  //}}}

  //{{{
  void addSample (uint8_t value) {
    mSamples[mCurSample++ % mNumSamples] = value;
    }
  //}}}
  //{{{
  void draw (cLcd* lcd, int y, int height) {

    for (int i = 0; i < lcd->getWidth(); i++) {
      uint32_t value = 0;
      for (int j = 0; j < mAverageSamples; j++)
        value += mCurSample+j > 0 ? mSamples[(mCurSample+j) % mNumSamples] : 0;
      value = (value * height) / (255 * mAverageSamples);

      lcd->fillRect (cLcd::eOff, cRect (i, y - value, i+1, y));
      }
    }
  //}}}

private:
  int mCurSample = 0;
  int mNumSamples = 0;
  int mAverageSamples = 1;
  uint8_t* mSamples = nullptr;
  };

class cTraceVec {
public:
  //{{{
  void addTrace (int numSamples, int averageSamples) {

    mTraces.push_back (new cTrace (numSamples, averageSamples));
    }
  //}}}
  //{{{
  void addSample (int trace, uint8_t value) {
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
