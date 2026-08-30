#include <string.h>

#include "py/runtime.h"

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "tars_bluetooth.h"


static bool bluetooth_started = false;


/* =========================================================
   BLUETOOTH CLASSIC + BLUEDROID
   ========================================================= */

esp_err_t tars_bluetooth_start(void) {

    if (bluetooth_started) {
        return ESP_OK;
    }


    esp_err_t ret;


    /*
     * Konfigurasi controller Bluetooth.
     */

    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    /*
     * Initialize controller.
     */

    ret = esp_bt_controller_init(
        &bt_cfg
    );


    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        return ret;
    }


    /*
     * Enable Bluetooth Classic.
     */

    ret = esp_bt_controller_enable(
        ESP_BT_MODE_CLASSIC_BT
    );


    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        return ret;
    }


    /*
     * Initialize Bluedroid.
     */

    ret = esp_bluedroid_init();


    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        return ret;
    }


    /*
     * Enable Bluedroid.
     */

    ret = esp_bluedroid_enable();


    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        return ret;
    }


    bluetooth_started = true;

    return ESP_OK;
}


/* =========================================================
   STATUS
   ========================================================= */

bool tars_bluetooth_ready(void) {

    return bluetooth_started;
}


/* =========================================================
   DEVICE NAME
   ========================================================= */

esp_err_t tars_bluetooth_set_name(
    const char *name
) {

    if (!bluetooth_started) {
        return ESP_ERR_INVALID_STATE;
    }


    return esp_bt_dev_set_device_name(
        name
    );
}


/* =========================================================
   MICROPYTHON: test()
   ========================================================= */

static mp_obj_t tars_bluetooth_test(void) {

    return mp_obj_new_str(
        "TARS BLUETOOTH READY",
        strlen("TARS BLUETOOTH READY")
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_bluetooth_test_obj,
    tars_bluetooth_test
);


/* =========================================================
   MICROPYTHON: init()
   ========================================================= */

static mp_obj_t tars_bluetooth_init(void) {

    esp_err_t ret =
        tars_bluetooth_start();


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluetooth Classic init failed"
            )
        );
    }


    ret = tars_bluetooth_set_name(
        "TARS"
    );


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluetooth name failed"
            )
        );
    }


    return mp_obj_new_str(
        "TARS BLUETOOTH CLASSIC READY",
        strlen(
            "TARS BLUETOOTH CLASSIC READY"
        )
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_bluetooth_init_obj,
    tars_bluetooth_init
);


/* =========================================================
   MICROPYTHON: status()
   ========================================================= */

static mp_obj_t tars_bluetooth_status(void) {

    return mp_obj_new_bool(
        bluetooth_started
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_bluetooth_status_obj,
    tars_bluetooth_status
);


/* =========================================================
   MODULE TABLE
   ========================================================= */

static const mp_rom_map_elem_t
tars_bluetooth_globals_table[] = {

    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_bluetooth)
    },


    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(
            &tars_bluetooth_test_obj
        )
    },


    {
        MP_ROM_QSTR(MP_QSTR_init),
        MP_ROM_PTR(
            &tars_bluetooth_init_obj
        )
    },


    {
        MP_ROM_QSTR(MP_QSTR_status),
        MP_ROM_PTR(
            &tars_bluetooth_status_obj
        )
    },

};


static MP_DEFINE_CONST_DICT(
    tars_bluetooth_globals,
    tars_bluetooth_globals_table
);


const mp_obj_module_t
tars_bluetooth_user_cmodule = {

    .base = {
        &mp_type_module
    },

    .globals =
        (mp_obj_dict_t *)
        &tars_bluetooth_globals,

};


MP_REGISTER_MODULE(
    MP_QSTR_tars_bluetooth,
    tars_bluetooth_user_cmodule
);
