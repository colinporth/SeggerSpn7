#pragma once
#include "6StepLib.h"

#define TOKEN "\r"
#define CMD_NUM 16

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE          (COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE  8

typedef struct {
  char name[10];
  void (*pCmdFunc)(void);
} CMD_T;

void CMD_STARTM( void );
void CMD_STOPMT( void );
void CMD_DIRECTION( void );
void CMD_SETSPD( void );
void CMD_GETSPD( void );
void CMD_STATUS( void );
void CMD_POTENZ( void );
void CMD_HELP(void);
void CMD_INIREF(void);
void CMD_POLESP(void);
void CMD_ACCELE(void);
void CMD_KP_PRM(void);
void CMD_KI_PRM(void);
