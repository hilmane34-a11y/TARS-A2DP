#include <string.h>

#include "py/runtime.h"

#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

#include "tars_bluetooth.h"


static bool a2dp_started = false;

static volatile bool connected = false;

static volatile bool audio_playing = false;

static volatile uint32_t pcm_bytes = 0;


/* =========================================================
   A2DP PCM DATA
   ========================================================= */

static void tars_a2dp_data_callback(
    const uint8_t *data,
    uint32_t len
) {

    /*
     * Audio PCM masuk ke sini.
     *
     * Untuk tahap pertama belum dikirim
     * ke DAC / amplifier.
     */

    (void)data;

    pcm_bytes += len;
}


/* =========================================================
   A2DP EVENTS
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

                connected = true;

            }


            else if (
                param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_DISCONNECTED
            ) {

                connected = false;

                audio_playing = false;
            }

            break;


        case ESP_A2D_AUDIO_STATE_EVT:

            if (
                param->audio_stat.state ==
                ESP_A2D_AUDIO_STATE_STARTED
            ) {

                audio_playing = true;

            } else {

                audio_playing = false;
            }

            break;


        default:

            break;
    }
}


/* =========================================================
   MICROPYTHON: test()
   ========================================================= */

static mp_obj_t tars_a2dp_test(void) {

    return mp_obj_new_str(
        "TARS A2DP READY",
        strlen("TARS A2DP READY")
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


/* =========================================================
   MICROPYTHON: init()
   ========================================================= */

static mp_obj_t tars_a2dp_init(void) {


    if (a2dp_started) {

        return mp_obj_new_str(
            "TARS A2DP already initialized",
            strlen(
                "TARS A2DP already initialized"
            )
        );
    }


    esp_err_t ret;


    /*
     * Start Bluetooth Classic + Bluedroid.
     */

    ret = tars_bluetooth_start();


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluetooth start failed"
            )
        );
    }


    /*
     * Nama perangkat.
     */

    ret = tars_bluetooth_set_name(
        "TARS-A2DP"
    );


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluetooth name failed"
            )
        );
    }


    /*
     * Register event callback.
     */

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


    /*
     * Register PCM callback.
     */

    ret =
        esp_a2d_sink_register_data_callback(
            tars_a2dp_data_callback
        );


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "A2DP data callback failed"
            )
        );
    }


    /*
     * Initialize A2DP Sink.
     */

    ret = esp_a2d_sink_init();


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "A2DP sink init failed"
            )
        );
    }


    /*
     * Bluetooth dapat ditemukan
     * dan dapat dihubungkan.
     */

    ret = esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        ESP_BT_GENERAL_DISCOVERABLE
    );


    if (ret != ESP_OK) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluetooth scan mode failed"
            )
        );
    }


    a2dp_started = true;


    return mp_obj_new_str(
        "TARS A2DP SINK READY",
        strlen("TARS A2DP SINK READY")
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_init_obj,
    tars_a2dp_init
);


/* =========================================================
   MICROPYTHON: status()
   ========================================================= */

static mp_obj_t tars_a2dp_status(void) {

    mp_obj_t items[5];


    items[0] =
        mp_obj_new_bool(
            tars_bluetooth_ready()
        );


    items[1] =
        mp_obj_new_bool(
            a2dp_started
        );


    items[2] =
        mp_obj_new_bool(
            connected
        );


    items[3] =
        mp_obj_new_bool(
            audio_playing
        );


    items[4] =
        mp_obj_new_int_from_uint(
            pcm_bytes
        );


    return mp_obj_new_tuple(
        5,
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

    .globals =
        (mp_obj_dict_t *)
        &tars_a2dp_globals,

};


MP_REGISTER_MODULE(
    MP_QSTR_tars_a2dp,
    tars_a2dp_user_cmodule
);
