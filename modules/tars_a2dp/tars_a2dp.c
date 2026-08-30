#include "py/runtime.h"
#include <string.h>

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

static bool tars_bt_started = false;


/* START BLUETOOTH CLASSIC + A2DP */

static mp_obj_t tars_a2dp_start(void) {
    esp_err_t ret;

    if (tars_bt_started) {
        return mp_obj_new_str(
            "TARS BLUETOOTH ALREADY STARTED",
            strlen("TARS BLUETOOTH ALREADY STARTED")
        );
    }

    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: BT CONTROLLER INIT FAILED",
            strlen("ERROR: BT CONTROLLER INIT FAILED")
        );
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: BT CONTROLLER ENABLE FAILED",
            strlen("ERROR: BT CONTROLLER ENABLE FAILED")
        );
    }

    ret = esp_bluedroid_init();

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: BLUEDROID INIT FAILED",
            strlen("ERROR: BLUEDROID INIT FAILED")
        );
    }

    ret = esp_bluedroid_enable();

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: BLUEDROID ENABLE FAILED",
            strlen("ERROR: BLUEDROID ENABLE FAILED")
        );
    }

    ret = esp_bt_gap_set_device_name("TARS");

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: SET DEVICE NAME FAILED",
            strlen("ERROR: SET DEVICE NAME FAILED")
        );
    }

    ret = esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        ESP_BT_GENERAL_DISCOVERABLE
    );

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: SET DISCOVERABLE FAILED",
            strlen("ERROR: SET DISCOVERABLE FAILED")
        );
    }

    ret = esp_a2d_source_init();

    if (ret != ESP_OK) {
        return mp_obj_new_str(
            "ERROR: A2DP SOURCE INIT FAILED",
            strlen("ERROR: A2DP SOURCE INIT FAILED")
        );
    }

    tars_bt_started = true;

    return mp_obj_new_str(
        "TARS BLUETOOTH CLASSIC A2DP READY",
        strlen("TARS BLUETOOTH CLASSIC A2DP READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);


/* STATUS */

static mp_obj_t tars_a2dp_test(void) {
    if (tars_bt_started) {
        return mp_obj_new_str(
            "TARS A2DP STARTED",
            strlen("TARS A2DP STARTED")
        );
    }

    return mp_obj_new_str(
        "TARS A2DP READY",
        strlen("TARS A2DP READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


/* MICROPYTHON MODULE */

static const mp_rom_map_elem_t tars_a2dp_globals_table[] = {
    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_a2dp)
    },
    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(&tars_a2dp_test_obj)
    },
    {
        MP_ROM_QSTR(MP_QSTR_start),
        MP_ROM_PTR(&tars_a2dp_start_obj)
    },
};

static MP_DEFINE_CONST_DICT(
    tars_a2dp_globals,
    tars_a2dp_globals_table
);

const mp_obj_module_t tars_a2dp_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&tars_a2dp_globals,
};

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
