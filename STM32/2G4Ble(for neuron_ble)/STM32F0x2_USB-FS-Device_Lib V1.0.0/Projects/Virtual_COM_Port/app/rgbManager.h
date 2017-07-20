#ifndef RGBMANAGER_H
#define RGBMANAGER_H

typedef struct
{
  unsigned char g;
  unsigned char r;
  unsigned char b;
} RGB;

void rgbMngInit(void);
void rgbShow(void);
void rgbMngClose(void);

#endif
