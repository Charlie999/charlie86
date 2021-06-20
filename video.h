#ifndef CHARLIE86_VIDEO_H
#define CHARLIE86_VIDEO_H
#include "types.h"

#define GRAPHICS_MODE_8BPP 0b001

#define TEXT_MODE_8BPC     0b010
#define TEXT_MODE_16BPC    0b100

#define WM_MSG_KILLWITHOUTEXIT 0xE000

void initvideo(int xres, int yres, uint32_t startaddr, uint16_t flags);
void killvideo();

void waitForKey();
int getLatestKey();
int isKeyPressed();

#endif //CHARLIE86_VIDEO_H
