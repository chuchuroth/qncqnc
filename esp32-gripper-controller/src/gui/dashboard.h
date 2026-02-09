#pragma once
#include <lvgl.h>
#include "modbus/gripper_controller.h"

class Dashboard {
public:
    Dashboard(GripperController& controller);

    void create(lv_obj_t* parent);
    void update(const GripperState& state);

private:
    GripperController& _controller;

    // Status bar
    lv_obj_t* _lbl_title;
    lv_obj_t* _led_online;
    lv_obj_t* _lbl_online;

    // State labels
    lv_obj_t* _lbl_init_state;
    lv_obj_t* _lbl_gripper_state;

    // Progress bars + value labels
    lv_obj_t* _bar_position;
    lv_obj_t* _lbl_position_val;
    lv_obj_t* _bar_force;
    lv_obj_t* _lbl_force_val;
    lv_obj_t* _bar_speed;
    lv_obj_t* _lbl_speed_val;

    // Control buttons
    lv_obj_t* _btn_open;
    lv_obj_t* _btn_close;
    lv_obj_t* _btn_init;
    lv_obj_t* _btn_setpos;

    // Position slider (toggled by SET POS button)
    lv_obj_t* _slider_container;
    lv_obj_t* _slider_pos;
    lv_obj_t* _lbl_slider_val;

    // Helpers
    lv_obj_t* createBar(lv_obj_t* parent, int y, const char* label,
                        int min_val, int max_val, lv_color_t color,
                        lv_obj_t** bar_out, lv_obj_t** val_lbl_out);
    lv_obj_t* createButton(lv_obj_t* parent, int x, int y,
                           const char* text, lv_color_t color,
                           lv_event_cb_t cb);

    // LVGL event callbacks (static C functions)
    static void onBtnOpen(lv_event_t* e);
    static void onBtnClose(lv_event_t* e);
    static void onBtnInit(lv_event_t* e);
    static void onBtnSetPos(lv_event_t* e);
    static void onSliderPos(lv_event_t* e);
};
