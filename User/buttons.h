#ifndef _BUTTONS_H
#define _BUTTONS_H

#ifdef __cplusplus
 extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////
#define BUTTON_SETTINGS   GPIO_Pin_3
#define BUTTON_NEXT       GPIO_Pin_4
#define BUTTON_UP         GPIO_Pin_5
#define BUTTON_DOWN       GPIO_Pin_6
#define BUTTONS           4

#define BUTTON_RETRY      3
#define BUTTON_DELAY      0

enum _button {
  settings,
  next,
  up,
  down
};

typedef struct {
  uint8_t cycles;
  uint8_t pressed;
} _button_t;

///////////////////////////////////////////////////////////////////////////////
void buttons_scan();

///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif
