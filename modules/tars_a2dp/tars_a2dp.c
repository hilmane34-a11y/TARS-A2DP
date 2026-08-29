#include "py/runtime.h"

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_a2dp_api.h"

static bool tars_bt_ready = false;
static bool tars_a2dp_started = false;


// =====================================================
// TEST
// =====================================================

static mp_obj_t tars_a2dp_test(void) {
    return mp_obj_new_str(
        "TARS A2DP MODULE READY",
        strlen("TARS A2DP MODULE READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


// =====================================================
// A2DP CALLBACK
// =====================================================

static void tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
) {

    switch (event) {

        case ESP_A2D_CONNECTION_STATE_EVT:

            mp_printf(
                &mp_plat_print,
                "[TARS A2DP] Connection state: %d\n",
                param->conn_stat.state
            );

            break;


        case ESP_A2D_AUDIO_STATE_EVT:

            mp_printf(
                &mp_plat_print,
                "[TARS A2DP] Audio state: %d\n",
                param->audio_stat.state
            );

            break;


        case ESP_A2D_PROF_STATE_EVT:

            mp_printf(
                &mp_plat_print,
                "[TARS A2DP] Profile state: %d\n",
                param->a2d_prof_stat.init_state
            );

            break;


        default:
            break;
    }
}


// =====================================================
// BLUETOOTH INITIALIZATION
// =====================================================

static mp_obj_t tars_a2dp_init(void) {

    if (tars_bt_ready) {

        return mp_obj_new_str(
            "Bluetooth already initialized",
            strlen("Bluetooth already initialized")
        );
    }


    esp_err_t ret;


    // Lepaskan memori BLE karena TARS menggunakan
    // Bluetooth Classic untuk A2DP.
    esp_bt_controller_mem_release(
        ESP_BT_MODE_BLE
    );


    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret = esp_bt_controller_init(
        &bt_cfg
    );

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {

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

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "BT controller enable failed"
            )
        );
    }


    ret = esp_bluedroid_init();

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluedroid init failed"
            )
        );
    }


    ret = esp_bluedroid_enable();

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluedroid enable failed"
            )
        );
    }


    tars_bt_ready = true;


    return mp_obj_new_str(
        "TARS Bluetooth Classic READY",
        strlen("TARS Bluetooth Classic READY")
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_init_obj,
    tars_a2dp_init
);


// =====================================================
// START A2DP SINK
// =====================================================

static mp_obj_t tars_a2dp_start(void) {

    if (!tars_bt_ready) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Run tars_a2dp.init() first"
            )
        );
    }


    if (tars_a2dp_started) {

        return mp_obj_new_str(
            "A2DP already started",
            strlen("A2DP already started")
        );
    }


    esp_err_t ret;


    ret = esp_a2d_register_callback(
        tars_a2dp_event_callback
    );

    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "A2DP callback failed"
            )
        );
    }


    ret = esp_a2d_sink_init();

    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "A2DP sink init failed"
            )
        );
    }


    tars_a2dp_started = true;


    return mp_obj_new_str(
        "TARS A2DP SINK READY",
        strlen("TARS A2DP SINK READY")
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);


// =====================================================
// STATUS
// =====================================================

static mp_obj_t tars_a2dp_status(void) {

    if (tars_a2dp_started) {

        return mp_obj_new_str(
            "A2DP STARTED",
            strlen("A2DP STARTED")
        );

    } else if (tars_bt_ready) {

        return mp_obj_new_str(
            "BLUETOOTH READY",
            strlen("BLUETOOTH READY")
        );

    } else {

        return mp_obj_new_str(
            "BLUETOOTH OFF",
            strlen("BLUETOOTH OFF")
        );
    }
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_status_obj,
    tars_a2dp_status
);


// =====================================================
// MODULE
// =====================================================

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

    {
        MP_ROM_QSTR(MP_QSTR_start),
        MP_ROM_PTR(
            &tars_a2dp_start_obj
        )
    },

    {
        MP_ROM_QSTR(MP_QSTR_status),
        MP_ROM_PTR(
            &tars_a2dp_status_obj
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
