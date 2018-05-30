// utils.h
#include "utils.h"

std::string dec (int num, int width, char fill) {

  std::string str;
  bool neg = num < 0;
  if (neg)
    num = -num;

  while ((width > 0) || num) {
    str = char((num % 10) + 0x30) + str;
    num /= 10;
    width--;
    }

  return (neg ? "-" : "") + str;
  }
