#pragma once

#define DEBUG 1

#if DEBUG

#define CONFIG() { \
    /* use the serial port */ \
    Serial.begin(9600); \
    /* Serial.begin(115200); */ \
    while (!Serial) { delay(500); } \
}

#define ERROR(err) { \
  Serial.println(err); \
  while (1); \
}

#define PRINT(msg) { \
    Serial.println(msg); \
}

#define ASSERTM(exp, msg) { \
    if(!(exp)) { \
       Serial.print(msg); \
       ERROR( #exp ); \
    } \
}

#else

#define CONFIG() { }
#define ERROR(msg) { }
#define PRINT(msg) { }
#define ASSERTM(exp, msg) (exp)

#endif

