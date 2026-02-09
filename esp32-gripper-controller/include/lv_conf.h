#ifndef LV_CONF_H
#define LV_CONF_H

/* Color depth: 16-bit RGB565 matches ILI9341 native format */
#define LV_COLOR_DEPTH 16

/* Memory: LVGL internal heap — 32KB for single-screen dashboard */
#define LV_MEM_SIZE (32 * 1024)

/* Tick: use Arduino millis() */
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/* Fonts — only 14 to save DRAM */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/* Widgets: enable only what we use */
#define LV_USE_LABEL    1
#define LV_USE_BTN      1
#define LV_USE_BAR      1
#define LV_USE_SLIDER   1
#define LV_USE_LED      1

/* Disable everything else */
#define LV_USE_MSGBOX       0
#define LV_USE_KEYBOARD     0
#define LV_USE_TEXTAREA     0
#define LV_USE_TABLE        0
#define LV_USE_CHART        0
#define LV_USE_CALENDAR     0
#define LV_USE_COLORWHEEL   0
#define LV_USE_IMGBTN       0
#define LV_USE_SPAN         0
#define LV_USE_ROLLER       0
#define LV_USE_DROPDOWN     0
#define LV_USE_ARC          0
#define LV_USE_ANIMIMG      0
#define LV_USE_CANVAS       0
#define LV_USE_LINE         0
#define LV_USE_LIST         0
#define LV_USE_MENU         0
#define LV_USE_METER        0
#define LV_USE_SPINBOX      0
#define LV_USE_SPINNER      0
#define LV_USE_TABVIEW      0
#define LV_USE_TILEVIEW     0
#define LV_USE_WIN          0
#define LV_USE_BUTTONMATRIX 0
#define LV_USE_CHECKBOX     0
#define LV_USE_IMAGE        0
#define LV_USE_SWITCH       0
#define LV_USE_TEXTAREA     0

/* Disable draw layers & features not needed */
#define LV_USE_DRAW_MASKS       0
#define LV_USE_FREETYPE         0
#define LV_USE_TINY_TTF         0
#define LV_USE_VECTOR_GRAPHIC   0
#define LV_USE_SVG              0
#define LV_USE_SYSMON           0
#define LV_USE_PROFILER         0
#define LV_USE_SNAPSHOT         0
#define LV_USE_MONKEY           0
#define LV_USE_GRIDNAV          0
#define LV_USE_FRAGMENT         0
#define LV_USE_IMGFONT          0
#define LV_USE_OBSERVER         1
#define LV_USE_IME_PINYIN       0
#define LV_USE_FILE_EXPLORER    0

/* Disable built-in demos */
#define LV_USE_DEMO_WIDGETS         0
#define LV_USE_DEMO_BENCHMARK       0
#define LV_USE_DEMO_STRESS          0
#define LV_USE_DEMO_MUSIC           0
#define LV_USE_DEMO_KEYPAD_AND_ENCODER 0

/* Logging */
#define LV_USE_LOG 0

/* OS: None — we handle threading ourselves via FreeRTOS tasks */
#define LV_USE_OS LV_OS_NONE

#endif /* LV_CONF_H */
