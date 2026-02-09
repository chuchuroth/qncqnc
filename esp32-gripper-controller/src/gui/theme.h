#pragma once
#include <lvgl.h>

namespace Theme {
    // Dark industrial color palette
    static const lv_color_t BG_COLOR    = lv_color_hex(0x1A1A2E);
    static const lv_color_t PANEL_COLOR = lv_color_hex(0x16213E);
    static const lv_color_t ACCENT      = lv_color_hex(0x0F3460);
    static const lv_color_t TEXT_COLOR  = lv_color_hex(0xE0E0E0);
    static const lv_color_t TEXT_DIM    = lv_color_hex(0x808080);
    static const lv_color_t GREEN       = lv_color_hex(0x00C853);
    static const lv_color_t RED         = lv_color_hex(0xFF1744);
    static const lv_color_t YELLOW      = lv_color_hex(0xFFD600);
    static const lv_color_t BLUE        = lv_color_hex(0x2979FF);

    // Layout (320x240 landscape)
    static const int SCREEN_W    = 320;
    static const int SCREEN_H    = 240;
    static const int PADDING     = 4;
    static const int BAR_HEIGHT  = 14;

    // Button sizes (min 60x36 for resistive touch accuracy)
    static const int BTN_WIDTH   = 70;
    static const int BTN_HEIGHT  = 34;
}
