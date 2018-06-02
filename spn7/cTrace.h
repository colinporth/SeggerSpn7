// cTrace.h
#pragma once
#include <vector>
#include "cLcd.h"

class cTraceVec {
public:
  //{{{
  void addTrace (int16_t numSamples, int16_t averageSamples, int16_t chans) {
    for (int i = 0; i < chans; i++)
      mTraces.push_back (new cTrace (numSamples, averageSamples));
    }
  //}}}
  //{{{
  void addSample (int16_t trace, uint8_t value) {
    if (trace < mTraces.size())
      mTraces[trace]->addSample (value);
    }
  //}}}
  //{{{
  void trigger() {
    for (auto trace : mTraces)
      trace->trigger();
    }
  //}}}
  //{{{
  void draw (cLcd* lcd, int16_t top, int16_t bottom) {

    int16_t height = (bottom - top - mTraces.size()) / mTraces.size();
    for (auto trace : mTraces) {
      trace->draw (lcd, top, top + height);
      top += height + 1;
      }
    }
  //}}}

private:
  //{{{
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
    void trigger() {

      mCurSample = 0;
      }
    //}}}
    //{{{
    void draw (cLcd* lcd, int16_t top, int16_t bottom) {

      int16_t range = mMax - mMin;
      int16_t height = bottom - top;

      int32_t sample = mCurSample - (lcd->getWidth() * mAverageSamples);
      for (int i = 0; i < lcd->getWidth(); i++) {
        uint32_t value = 0;
        for (int j = 0; j < mAverageSamples; j++)
          value += sample+j > 0 ? mSamples[(sample+j) % mNumSamples] : 0;
        value = (value * height) / (range * mAverageSamples);
        lcd->fillRect (cLcd::eOff, cRect (i, bottom - value, i+1, bottom));
        sample += mAverageSamples;
        }
      }
    //}}}

  private:
    int16_t mNumSamples = 0;
    int16_t mAverageSamples = 1;
    uint8_t* mSamples = nullptr;
    int32_t mCurSample = 0;
    uint8_t mMin = 255;
    uint8_t mMax = 0;
    };
  //}}}
  std::vector<cTrace*> mTraces;
  };
