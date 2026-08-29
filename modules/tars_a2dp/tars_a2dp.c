#include "py/runtime.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_a2dp_api.h"


static bool tars_bt_ready = false;
static bool tars_a2dp_ready = false;
static bool tars_connected = false;

static esp_bd_addr_t tars_remote_bda = {0};


/* =====================================================
   TEST
   ===================================================== */

static mp_obj_t tars_a2dp_test(void) {
    return mp_obj_new_str(
        "TARS A2DP SOURCE READY",
        strlen("TARS A2DP SOURCE READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


/* =====================================================
   A2DP EVENT CALLBACK
   ===================================================== */

static void tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
) {
    switch (event) {

        case ESP_A2D_CONNECTION_STATE_EVT:

            if (param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_CONNECTED) {

                tars_connected = true;

                memcpy(
                    tars_remote_bda,
                    param->conn_stat.remote_bda,
                    ESP_BD_ADDR_LEN
                );

                mp_printf(
                    &mp_plat_print,
                    "[TARS A2DP] SPEAKER CONNECTED\n"
                );

            } else if (
                param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_DISCONNECTED
            ) {

                tars_connected = false;

                mp_printf(
                    &mp_plat_print,
                    "[TARS A2DP] SPEAKER DISCONNECTED\n"
                );
            }

            break;


        case ESP_A2D_AUDIO_STATE_EVT:

            mp_printf(
                &mp_plat_print,
                "[TARS A2DP] AUDIO STATE: %d\n",
                param->audio_stat.state
            );

            break;


        case ESP_A2D_PROF_STATE_EVT:

            mp_printf(
                &mp_plat_print,
                "[TARS A2DP] PROFILE STATE: %d\n",
                param->a2d_prof_stat.init_state
            );

            break;


        default:
            break;
    }
}


/* =====================================================
   INITIALIZE BLUETOOTH CLASSIC
   ===================================================== */

static mp_obj_t tars_a2dp_init(void) {

    if (tars_bt_ready) {
        return mp_obj_new_str(
            "Bluetooth already initialized",
            strlen("Bluetooth already initialized")
        );
    }


    esp_err_t ret;


    /* Free BLE memory because TARS uses Classic BT */
    esp_bt_controller_mem_release(
        ESP_BT_MODE_BLE
    );


    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret = esp_bt_controller_init(&bt_cfg);

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("BT controller init failed")
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
            MP_ERROR_TEXT("BT controller enable failed")
        );
    }


    ret = esp_bluedroid_init();

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("Bluedroid init failed")
        );
    }


    ret = esp_bluedroid_enable();

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("Bluedroid enable failed")
        );
    }


    tars_bt_ready = true;


    mp_printf(
        &mp_plat_print,
        "[TARS A2DP] BLUETOOTH CLASSIC READY\n"
    );


    return mp_obj_new_str(
        "TARS Bluetooth Classic READY",
        strlen("TARS Bluetooth Classic READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_init_obj,
    tars_a2dp_init
);


/* =====================================================
   START A2DP SOURCE
   ===================================================== */

static mp_obj_t tars_a2dp_start(void) {

    if (!tars_bt_ready) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("Run init() first")
        );
    }


    if (tars_a2dp_ready) {
        return mp_obj_new_str(
            "A2DP Source already started",
            strlen("A2DP Source already started")
        );
    }


    esp_err_t ret;


    ret = esp_a2d_register_callback(
        tars_a2dp_event_callback
    );

    if (ret != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("A2DP callback failed")
        );
    }


    ret = esp_a2d_source_init();

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    ) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("A2DP Source init failed")
        );
    }


    tars_a2dp_ready = true;


    mp_printf(
        &mp_plat_print,
        "[TARS A2DP] SOURCE READY\n"
    );


    return mp_obj_new_str(
        "TARS A2DP SOURCE READY",
        strlen("TARS A2DP SOURCE READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);


/* =====================================================
   CONNECT TO SPEAKER

   MAC format:
   "AA:BB:CC:DD:EE:FF"
   ===================================================== */

static mp_obj_t tars_a2dp_connect(
    mp_obj_t mac_obj
) {

    if (!tars_a2dp_ready) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("Run start() first")
        );
    }


    const char *mac =
        mp_obj_str_get_str(mac_obj);


    unsigned int b[6];


    int parsed = sscanf(
        mac,
        "%02x:%02x:%02x:%02x:%02x:%02x",
        &b[0],
        &b[1],
        &b[2],
        &b[3],
        &b[4],
        &b[5]
    );


    if (parsed != 6) {
        mp_raise_ValueError(
            MP_ERROR_TEXT(
                "MAC format AA:BB:CC:DD:EE:FF"
            )
        );
    }


    esp_bd_addr_t remote_bda;

    for (int i = 0; i < 6; i++) {
        remote_bda[i] = (uint8_t)b[i];
    }


    esp_err_t ret =
        esp_a2d_source_connect(
            remote_bda
        );


    if (ret != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Speaker connection request failed"
            )
        );
    }


    mp_printf(
        &mp_plat_print,
        "[TARS A2DP] CONNECTING TO %s\n",
        mac
    );


    return mp_obj_new_str(
        "Connection request sent",
        strlen("Connection request sent")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_1(
    tars_a2dp_connect_obj,
    tars_a2dp_connect
);


/* =====================================================
   STATUS
   ===================================================== */

static mp_obj_t tars_a2dp_status(void) {

    if (tars_connected) {

        return mp_obj_new_str(
            "SPEAKER CONNECTED",
            strlen("SPEAKER CONNECTED")
        );
    }


    if (tars_a2dp_ready) {

        return mp_obj_new_str(
            "A2DP SOURCE READY",
            strlen("A2DP SOURCE READY")
        );
    }


    if (tars_bt_ready) {

        return mp_obj_new_str(
            "BLUETOOTH READY",
            strlen("BLUETOOTH READY")
        );
    }


    return mp_obj_new_str(
        "BLUETOOTH OFF",
        strlen("BLUETOOTH OFF")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_status_obj,
    tars_a2dp_status
);


/* =====================================================
   MODULE
   ===================================================== */

static const mp_rom_map_elem_t
tars_a2dp_globals_table[] = {

    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_a2dp)
    },

    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(&tars_a2dp_test_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_init),
        MP_ROM_PTR(&tars_a2dp_init_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_start),
        MP_ROM_PTR(&tars_a2dp_start_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_connect),
        MP_ROM_PTR(&tars_a2dp_connect_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_status),
        MP_ROM_PTR(&tars_a2dp_status_obj)
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
