#pragma once

#include "hal.h"

hal_err_t render_init(void);

// Projects the latest fluid snapshot and pushes a whole frame to the panel.
void render_frame(void);
