#include "dashboard.h"
#include "gui/theme.h"
#include "config.h"

Dashboard::Dashboard(GripperController& controller)
    : _controller(controller)
    , _lbl_title(nullptr), _led_online(nullptr), _lbl_online(nullptr)
    , _lbl_init_state(nullptr), _lbl_gripper_state(nullptr)
    , _bar_position(nullptr), _lbl_position_val(nullptr)
    , _bar_force(nullptr), _lbl_force_val(nullptr)
    , _bar_speed(nullptr), _lbl_speed_val(nullptr)
    , _btn_open(nullptr), _btn_close(nullptr)
    , _btn_init(nullptr), _btn_setpos(nullptr)
    , _slider_container(nullptr), _slider_pos(nullptr), _lbl_slider_val(nullptr)
{
}

// ============================================================================
// Layout Creation — 320x240 landscape dashboard
// ============================================================================

void Dashboard::create(lv_obj_t* parent)
{
    // Set background color
    lv_obj_set_style_bg_color(parent, Theme::BG_COLOR, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    // ---- Row 0: Title bar (y=2) ----
    _lbl_title = lv_label_create(parent);
    lv_label_set_text(_lbl_title, "GRIPPER DASHBOARD");
    lv_obj_set_style_text_font(_lbl_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(_lbl_title, Theme::TEXT_COLOR, 0);
    lv_obj_set_pos(_lbl_title, Theme::PADDING, 4);

    _led_online = lv_led_create(parent);
    lv_led_set_color(_led_online, Theme::RED);
    lv_obj_set_size(_led_online, 12, 12);
    lv_obj_set_pos(_led_online, 260, 6);
    lv_led_off(_led_online);

    _lbl_online = lv_label_create(parent);
    lv_label_set_text(_lbl_online, "OFFLINE");
    lv_obj_set_style_text_font(_lbl_online, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(_lbl_online, Theme::RED, 0);
    lv_obj_set_pos(_lbl_online, 276, 4);

    // Separator line using a thin bar
    lv_obj_t* sep = lv_obj_create(parent);
    lv_obj_set_size(sep, 312, 1);
    lv_obj_set_pos(sep, Theme::PADDING, 22);
    lv_obj_set_style_bg_color(sep, Theme::ACCENT, 0);
    lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep, 0, 0);
    lv_obj_set_style_pad_all(sep, 0, 0);

    // ---- Row 1: State labels (y=26) ----
    _lbl_init_state = lv_label_create(parent);
    lv_label_set_text(_lbl_init_state, "Init: ---");
    lv_obj_set_style_text_font(_lbl_init_state, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(_lbl_init_state, Theme::YELLOW, 0);
    lv_obj_set_pos(_lbl_init_state, Theme::PADDING, 28);

    _lbl_gripper_state = lv_label_create(parent);
    lv_label_set_text(_lbl_gripper_state, "State: ---");
    lv_obj_set_style_text_font(_lbl_gripper_state, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(_lbl_gripper_state, Theme::TEXT_COLOR, 0);
    lv_obj_set_pos(_lbl_gripper_state, 165, 28);

    // ---- Row 2: Position bar (y=48) ----
    createBar(parent, 48, "Pos", GRIPPER_POSITION_MIN, GRIPPER_POSITION_MAX,
              Theme::BLUE, &_bar_position, &_lbl_position_val);

    // ---- Row 3: Force bar (y=78) ----
    createBar(parent, 78, "Force", 0, 100, Theme::YELLOW,
              &_bar_force, &_lbl_force_val);

    // ---- Row 4: Speed bar (y=108) ----
    createBar(parent, 108, "Speed", 0, 100, Theme::GREEN,
              &_bar_speed, &_lbl_speed_val);

    // ---- Row 5: Control buttons (y=138) ----
    int btn_y = 138;
    int btn_spacing = Theme::BTN_WIDTH + 6;

    _btn_open = createButton(parent, Theme::PADDING, btn_y,
                             "OPEN", Theme::GREEN, onBtnOpen);
    _btn_close = createButton(parent, Theme::PADDING + btn_spacing, btn_y,
                              "CLOSE", Theme::RED, onBtnClose);
    _btn_init = createButton(parent, Theme::PADDING + btn_spacing * 2, btn_y,
                             "INIT", Theme::YELLOW, onBtnInit);
    _btn_setpos = createButton(parent, Theme::PADDING + btn_spacing * 3, btn_y,
                               "SET POS", Theme::BLUE, onBtnSetPos);

    // ---- Slider container (y=178, initially hidden) ----
    _slider_container = lv_obj_create(parent);
    lv_obj_set_size(_slider_container, 312, 50);
    lv_obj_set_pos(_slider_container, Theme::PADDING, 178);
    lv_obj_set_style_bg_color(_slider_container, Theme::PANEL_COLOR, 0);
    lv_obj_set_style_bg_opa(_slider_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(_slider_container, Theme::ACCENT, 0);
    lv_obj_set_style_border_width(_slider_container, 1, 0);
    lv_obj_set_style_pad_all(_slider_container, 6, 0);
    lv_obj_add_flag(_slider_container, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* slider_label = lv_label_create(_slider_container);
    lv_label_set_text(slider_label, "Position:");
    lv_obj_set_style_text_font(slider_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(slider_label, Theme::TEXT_DIM, 0);
    lv_obj_set_pos(slider_label, 0, 2);

    _slider_pos = lv_slider_create(_slider_container);
    lv_slider_set_range(_slider_pos, GRIPPER_POSITION_MIN, GRIPPER_POSITION_MAX);
    lv_slider_set_value(_slider_pos, 500, LV_ANIM_OFF);
    lv_obj_set_size(_slider_pos, 200, 14);
    lv_obj_set_pos(_slider_pos, 70, 6);
    lv_obj_set_style_bg_color(_slider_pos, Theme::ACCENT, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_slider_pos, Theme::BLUE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_slider_pos, Theme::TEXT_COLOR, LV_PART_KNOB);
    lv_obj_add_event_cb(_slider_pos, onSliderPos, LV_EVENT_VALUE_CHANGED, this);

    _lbl_slider_val = lv_label_create(_slider_container);
    lv_label_set_text(_lbl_slider_val, "500");
    lv_obj_set_style_text_font(_lbl_slider_val, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(_lbl_slider_val, Theme::TEXT_COLOR, 0);
    lv_obj_set_pos(_lbl_slider_val, 276, 2);
}

// ============================================================================
// Widget Factory Helpers
// ============================================================================

lv_obj_t* Dashboard::createBar(lv_obj_t* parent, int y, const char* label,
                               int min_val, int max_val, lv_color_t color,
                               lv_obj_t** bar_out, lv_obj_t** val_lbl_out)
{
    // Label
    lv_obj_t* lbl = lv_label_create(parent);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, Theme::TEXT_DIM, 0);
    lv_obj_set_pos(lbl, Theme::PADDING, y + 2);

    // Bar
    *bar_out = lv_bar_create(parent);
    lv_bar_set_range(*bar_out, min_val, max_val);
    lv_bar_set_value(*bar_out, 0, LV_ANIM_OFF);
    lv_obj_set_size(*bar_out, 200, Theme::BAR_HEIGHT);
    lv_obj_set_pos(*bar_out, 52, y + 2);
    lv_obj_set_style_bg_color(*bar_out, Theme::ACCENT, LV_PART_MAIN);
    lv_obj_set_style_bg_color(*bar_out, color, LV_PART_INDICATOR);

    // Value label
    *val_lbl_out = lv_label_create(parent);
    lv_label_set_text(*val_lbl_out, "---");
    lv_obj_set_style_text_font(*val_lbl_out, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(*val_lbl_out, Theme::TEXT_COLOR, 0);
    lv_obj_set_pos(*val_lbl_out, 258, y + 2);

    return lbl;
}

lv_obj_t* Dashboard::createButton(lv_obj_t* parent, int x, int y,
                                  const char* text, lv_color_t color,
                                  lv_event_cb_t cb)
{
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, Theme::BTN_WIDTH, Theme::BTN_HEIGHT);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_bg_color(btn, color, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_80, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, this);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_center(lbl);

    return btn;
}

// ============================================================================
// Update — called from GUI task with latest state snapshot
// ============================================================================

void Dashboard::update(const GripperState& state)
{
    // Online LED
    if (state.online) {
        lv_led_set_color(_led_online, Theme::GREEN);
        lv_led_on(_led_online);
        lv_label_set_text(_lbl_online, "ONLINE");
        lv_obj_set_style_text_color(_lbl_online, Theme::GREEN, 0);
    } else {
        lv_led_set_color(_led_online, Theme::RED);
        lv_led_off(_led_online);
        lv_label_set_text(_lbl_online, "OFFLINE");
        lv_obj_set_style_text_color(_lbl_online, Theme::RED, 0);
    }

    // State labels
    lv_label_set_text_fmt(_lbl_init_state, "Init: %s", initStateStr(state.init_state));
    lv_label_set_text_fmt(_lbl_gripper_state, "State: %s", gripperStateStr(state.gripper_state));

    // Position bar (0-1000 range, show as mm)
    lv_bar_set_value(_bar_position, state.current_position, LV_ANIM_ON);
    lv_label_set_text_fmt(_lbl_position_val, "%d", state.current_position);

    // Force bar
    lv_bar_set_value(_bar_force, state.force_percent, LV_ANIM_ON);
    lv_label_set_text_fmt(_lbl_force_val, "%d%%", state.force_percent);

    // Speed bar
    lv_bar_set_value(_bar_speed, state.speed_percent, LV_ANIM_ON);
    lv_label_set_text_fmt(_lbl_speed_val, "%d%%", state.speed_percent);
}

// ============================================================================
// Button Callbacks
// ============================================================================

void Dashboard::onBtnOpen(lv_event_t* e)
{
    Dashboard* self = (Dashboard*)lv_event_get_user_data(e);
    self->_controller.open();
}

void Dashboard::onBtnClose(lv_event_t* e)
{
    Dashboard* self = (Dashboard*)lv_event_get_user_data(e);
    self->_controller.close();
}

void Dashboard::onBtnInit(lv_event_t* e)
{
    Dashboard* self = (Dashboard*)lv_event_get_user_data(e);
    self->_controller.init(false);
}

void Dashboard::onBtnSetPos(lv_event_t* e)
{
    Dashboard* self = (Dashboard*)lv_event_get_user_data(e);
    // Toggle slider visibility
    if (lv_obj_has_flag(self->_slider_container, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_remove_flag(self->_slider_container, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(self->_slider_container, LV_OBJ_FLAG_HIDDEN);
    }
}

void Dashboard::onSliderPos(lv_event_t* e)
{
    Dashboard* self = (Dashboard*)lv_event_get_user_data(e);
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    int32_t pos = lv_slider_get_value(slider);
    lv_label_set_text_fmt(self->_lbl_slider_val, "%d", (int)pos);
    self->_controller.setPosition((uint16_t)pos);
}
