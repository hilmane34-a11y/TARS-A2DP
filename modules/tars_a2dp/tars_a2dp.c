#include "py/runtime.h"

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"


static bool tars_bt_ready = false;
static bool tars_a2dp_ready = false;

static volatile bool tars_connected = false;
static volatile bool tars_audio_playing = false;

static uint32_t tars_pcm_bytes = 0;


/* =========================================================
   HELPER
   ========================================================= */

static void tars_check(
    esp_err_t err,
    const char *message
) {
    if (err != ESP_OK) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("TARS Bluetooth error")
        );
    }

    (void)message;
}


/* =========================================================
   A2DP DATA CALLBACK

   PCM data diterima di sini.

   Tahap ini belum mengirim PCM ke amplifier karena modul
   audio fisik TARS belum ditentukan.
   ========================================================= */

static void tars_a2dp_data_callback(
    const uint8_t *data,
    uint32_t len
) {
    (void)data;

    tars_pcm_bytes += len;
}


/* =========================================================
   A2DP EVENT CALLBACK
   ========================================================= */

static void tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
) {
    switch (event) {

        case ESP_A2D_CONNECTION_STATE_EVT:

            if (
                param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_CONNECTED
            ) {
                tars_connected = true;

            } else if (
                param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_DISCONNECTED
            ) {
                tars_connected = false;
                tars_audio_playing = false;
            }

            break;


        case ESP_A2D_AUDIO_STATE_EVT:

            if (
                param->audio_stat.state ==
                ESP_A2D_AUDIO_STATE_STARTED
            ) {
                tars_audio_playing = true;

            } else {
                tars_audio_playing = false;
            }

            break;


        default:
            break;
    }
}


/* =========================================================
   TEST
   ========================================================= */

static mp_obj_t tars_a2dp_test(void) {

    return mp_obj_new_str(
        "TARS A2DP READY",
        15
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


/* =========================================================
   INIT BLUETOOTH CLASSIC + A2DP SINK
   ========================================================= */

static mp_obj_t tars_a2dp_init(void) {

    if (tars_a2dp_ready) {

        return mp_obj_new_str(
            "TARS A2DP already initialized",
            29
        );
    }


    esp_err_t ret;


    /* Lepaskan BLE controller memory.

       Firmware ini memakai Bluetooth Classic untuk A2DP.
    */

    ret = esp_bt_controller_mem_release(
        ESP_BT_MODE_BLE
    );

    /*
       Bisa ESP_OK atau state tertentu tergantung kondisi.
       Jangan hentikan hanya karena memory sudah diatur.
    */

    (void)ret;


    /* Bluetooth controller */

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


    /* Bluedroid */

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


    /* Nama perangkat Bluetooth */

    ret = esp_bt_dev_set_device_name(
        "TARS-A2DP"
    );

    tars_check(
        ret,
        "device name failed"
    );


    /* A2DP callback */

    ret = esp_a2d_register_callback(
        tars_a2dp_event_callback
    );

    tars_check(
        ret,
        "A2DP callback failed"
    );


    /* Terima data PCM */

    ret =
        esp_a2d_sink_register_data_callback(
            tars_a2dp_data_callback
        );

    tars_check(
        ret,
        "A2DP data callback failed"
    );


    /* Inisialisasi A2DP Sink */

    ret = esp_a2d_sink_init();

    tars_check(
        ret,
        "A2DP sink init failed"
    );


    /* Discoverable + connectable */

    ret = esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        ESP_BT_GENERAL_DISCOVERABLE
    );

    tars_check(
        ret,
        "BT scan mode failed"
    );


    tars_bt_ready = true;
    tars_a2dp_ready = true;


    return mp_obj_new_str(
        "TARS A2DP SINK READY",
        20
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_init_obj,
    tars_a2dp_init
);


/* =========================================================
   STATUS
   ========================================================= */

static mp_obj_t tars_a2dp_status(void) {

    mp_obj_t items[4];

    items[0] =
        mp_obj_new_bool(
            tars_bt_ready
        );

    items[1] =
        mp_obj_new_bool(
            tars_connected
        );

    items[2] =
        mp_obj_new_bool(
            tars_audio_playing
        );

    items[3] =
        mp_obj_new_int_from_uint(
            tars_pcm_bytes
        );


    return mp_obj_new_tuple(
        4,
        items
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_status_obj,
    tars_a2dp_status
);


/* =========================================================
   MODULE TABLE
   ========================================================= */

static const mp_rom_map_elem_t
tars_a2dp_globals_table[] = {

    {
        MP_ROM_QSTR(
            MP_QSTR___name__
        ),
        MP_ROM_QSTR(
            MP_QSTR_tars_a2dp
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_test
        ),
        MP_ROM_PTR(
            &tars_a2dp_test_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_init
        ),
        MP_ROM_PTR(
            &tars_a2dp_init_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_status
        ),
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

    .globals =
        (mp_obj_dict_t *)
        &tars_a2dp_globals,

};


MP_REGISTER_MODULE(
    MP_QSTR_tars_a2dp,
    tars_a2dp_user_cmodule
);
