#include <debug.h>
#include "buttons.h"
#include "display.h"

///////////////////////////////////////////////////////////////////////////////
//extern enum display_state state;

const uint16_t masks[BUTTONS] = {BUTTON_SETTINGS, BUTTON_NEXT, BUTTON_UP, BUTTON_DOWN};
_button_t buttons[BUTTONS];

/*****************************************************************************
 * @brief Scan buttons
 * 
 *****************************************************************************/
void buttons_scan() {
  uint16_t pins = GPIO_ReadInputData(GPIOC);

  for (uint8_t p = 0; p < BUTTONS; p++) {
    buttons[p].pressed = 0;

    uint16_t mask = masks[p];
    if ((pins & mask) == 0) {
      buttons[p].cycles++;

      if (buttons[p].cycles > BUTTON_RETRY) {
        buttons[p].pressed = 1;
        buttons[p].cycles = 0;
      }
    } else {
      if (buttons[p].cycles > BUTTON_DELAY) {
        buttons[p].pressed = 1;
        buttons[p].cycles = 0;
      }
    }
  }
}
