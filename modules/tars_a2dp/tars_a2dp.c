#include "py/runtime.h"

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_a2dp_api.h"

static bool tars_bt_ready = false;

// ========================================
// TEST
// ========================================

static mp_obj_t tars_a2dp_test(void) {
    return mp_obj_new_str("TARS A2DP READY", 15);
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


// ========================================
// BLUETOOTH INITIALIZATION
// ========================================

static mp_obj_t tars_a2dp_init(void) {

    if (tars_bt_ready) {
        return mp_obj_new_str(
            "Bluetooth already initialized",
            29
        );
    }

    esp_err_t ret;

    // Bluetooth controller
    ret = esp_bt_controller_mem_release(
        ESP_BT_MODE_BLE
    );

    // Tidak masalah jika memori sudah tidak bisa dilepas
    (void)ret;

    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);

    if (ret != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "BT controller init failed"
            )
        );
    }

    ret = esp_bt_controller_enable(
        ESP_BT_MODE_CLASSIC_BT
    );

    if (ret != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "BT controller enable failed"
            )
        );
    }

    ret = esp_bluedroid_init();

    if (ret != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluedroid init failed"
            )
        );
    }

    ret = esp_bluedroid_enable();

    if (ret != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluedroid enable failed"
            )
        );
    }

    tars_bt_ready = true;

    return mp_obj_new_str(
        "Bluetooth Classic READY",
        23
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_init_obj,
    tars_a2dp_init
);


// ========================================
// MODULE
// ========================================

static const mp_rom_map_elem_t
tars_a2dp_globals_table[] = {

    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_a2dp)
    },

    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(
            &tars_a2dp_test_obj
        )
    },

    {
        MP_ROM_QSTR(MP_QSTR_init),
        MP_ROM_PTR(
            &tars_a2dp_init_obj
        )
    },

};

static MP_DEFINE_CONST_DICT(
    tars_a2dp_globals,
    tars_a2dp_globals_table
);


const mp_obj_module_t
tars_a2dp_user_cmodule = {

    .base = {
        &mp_type_module
    },

    .globals = (
        mp_obj_dict_t *
    )&tars_a2dp_globals,

};


MP_REGISTER_MODULE(
    MP_QSTR_tars_a2dp,
    tars_a2dp_user_cmodule
);
