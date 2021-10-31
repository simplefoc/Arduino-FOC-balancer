#include "Arduino.h"

// pinout es32 d1 r32
#if defined(ESP_H)
  // driver pionouts
  #define MOT1_A 27    // 5
  #define MOT1_B 5     // 10
  #define MOT1_C 16    // 6
  #define MOT1_EN 12   // 8
  #define MOT2_A 18    // 3
  #define MOT2_B 25    // 9
  #define MOT2_C 13    // 11
  #define MOT2_EN 14   // 7
  // encoder pinouts
  #define ENC1_A 26    // 2
  #define ENC1_B 17    // 4
  #define ENC2_A 36    // A4
  #define ENC2_B 19    // 12
#else // nucleo/arduino pinout

// driver pionouts
  #define MOT1_A  5
  #define MOT1_B  10
  #define MOT1_C  6
  #define MOT1_EN 8
  #define MOT2_A  3
  #define MOT2_B  9
  #define MOT2_C  11
  #define MOT2_EN 7
  // encoder pinouts
  #define ENC1_A 2
  #define ENC1_B 4
  #define ENC2_A A4
  #define ENC2_B 12
  
#endif
