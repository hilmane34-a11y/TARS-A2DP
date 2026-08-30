#include "py/runtime.h"
#include "py/objstr.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"


/* =========================================================
   TARS A2DP SOURCE
   Bluetooth Classic + Scan + Connect + PCM Streaming
   ========================================================= */


/* ---------------------------------------------------------
   SETTINGS
   --------------------------------------------------------- */

#define TARS_DEVICE_NAME "TARS"
#define TARS_TARGET_NAME "I7-TWS"

#define PCM_BUFFER_SIZE 16384

#define TARS_TAG "TARS_A2DP"


/* ---------------------------------------------------------
   BLUETOOTH STATE
   --------------------------------------------------------- */

static bool tars_bt_started = false;
static bool tars_scanning = false;
static bool tars_device_found = false;

static bool tars_a2dp_connected = false;
static bool tars_a2dp_connecting = false;
static bool tars_audio_started = false;


/* Target Bluetooth address */

static esp_bd_addr_t tars_target_bda = {0};


/* ---------------------------------------------------------
   PCM BUFFER
   --------------------------------------------------------- */

static uint8_t pcm_buffer[PCM_BUFFER_SIZE];

static volatile size_t pcm_read_pos = 0;
static volatile size_t pcm_write_pos = 0;
static volatile size_t pcm_used = 0;


/* ---------------------------------------------------------
   DEBUG
   --------------------------------------------------------- */

static volatile uint32_t tars_callback_count = 0;
static volatile uint32_t tars_callback_bytes = 0;


/* ---------------------------------------------------------
   STATUS TEXT
   --------------------------------------------------------- */

static const char *tars_status_text =
    "TARS A2DP READY";


/* =========================================================
   PCM BUFFER FUNCTIONS
   ========================================================= */


/* ---------------------------------------------------------
   CLEAR
   --------------------------------------------------------- */

static void tars_pcm_clear(void) {

    taskENTER_CRITICAL();

    pcm_read_pos = 0;
    pcm_write_pos = 0;
    pcm_used = 0;

    taskEXIT_CRITICAL();

}


/* ---------------------------------------------------------
   WRITE
   --------------------------------------------------------- */

static size_t tars_pcm_write(
    const uint8_t *data,
    size_t len
) {

    size_t written = 0;

    taskENTER_CRITICAL();

    while (
        written < len &&
        pcm_used < PCM_BUFFER_SIZE
    ) {

        pcm_buffer[pcm_write_pos] =
            data[written];

        pcm_write_pos++;

        if (
            pcm_write_pos >= PCM_BUFFER_SIZE
        ) {
            pcm_write_pos = 0;
        }

        pcm_used++;

        written++;

    }

    taskEXIT_CRITICAL();

    return written;

}


/* ---------------------------------------------------------
   READ
   --------------------------------------------------------- */

static size_t tars_pcm_read(
    uint8_t *data,
    size_t len
) {

    size_t read = 0;

    taskENTER_CRITICAL();

    while (
        read < len &&
        pcm_used > 0
    ) {

        data[read] =
            pcm_buffer[pcm_read_pos];

        pcm_read_pos++;

        if (
            pcm_read_pos >= PCM_BUFFER_SIZE
        ) {
            pcm_read_pos = 0;
        }

        pcm_used--;

        read++;

    }

    taskEXIT_CRITICAL();

    return read;

}


/* =========================================================
   A2DP AUDIO DATA CALLBACK
   ========================================================= */

static int32_t tars_a2dp_data_callback(
    uint8_t *data,
    int32_t len
) {

    if (
        data == NULL ||
        len <= 0
    ) {

        return 0;

    }


    /*
       IMPORTANT:

       This counter lets us verify
       that ESP-IDF is actually requesting audio.
    */

    tars_callback_count++;

    tars_callback_bytes +=
        (uint32_t)len;


    size_t requested =
        (size_t)len;


    size_t received =
        tars_pcm_read(
            data,
            requested
        );


    /*
       If PCM buffer is empty,
       send silence.
    */

    if (
        received < requested
    ) {

        memset(
            data + received,
            0,
            requested - received
        );

    }


    return len;

}


/* =========================================================
   A2DP EVENT CALLBACK
   ========================================================= */

static void tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
) {

    switch (event) {


        /* ---------------------------------------------
           CONNECTION STATE
           --------------------------------------------- */

        case ESP_A2D_CONNECTION_STATE_EVT:

            switch (
                param->conn_stat.state
            ) {

                case ESP_A2D_CONNECTION_STATE_DISCONNECTED:

                    tars_a2dp_connected =
                        false;

                    tars_a2dp_connecting =
                        false;

                    tars_audio_started =
                        false;

                    tars_status_text =
                        "A2DP DISCONNECTED";

                    ESP_LOGI(
                        TARS_TAG,
                        "A2DP DISCONNECTED"
                    );

                    break;


                case ESP_A2D_CONNECTION_STATE_CONNECTING:

                    tars_a2dp_connecting =
                        true;

                    tars_a2dp_connected =
                        false;

                    tars_audio_started =
                        false;

                    tars_status_text =
                        "A2DP CONNECTING";

                    ESP_LOGI(
                        TARS_TAG,
                        "A2DP CONNECTING"
                    );

                    break;


                case ESP_A2D_CONNECTION_STATE_CONNECTED:

                    tars_a2dp_connected =
                        true;

                    tars_a2dp_connecting =
                        false;

                    tars_audio_started =
                        false;

                    tars_status_text =
                        "A2DP CONNECTED";

                    ESP_LOGI(
                        TARS_TAG,
                        "A2DP CONNECTED"
                    );

                    break;


                case ESP_A2D_CONNECTION_STATE_DISCONNECTING:

                    tars_a2dp_connected =
                        false;

                    tars_a2dp_connecting =
                        false;

                    tars_audio_started =
                        false;

                    tars_status_text =
                        "A2DP DISCONNECTING";

                    ESP_LOGI(
                        TARS_TAG,
                        "A2DP DISCONNECTING"
                    );

                    break;


                default:

                    break;

            }

            break;


        /* ---------------------------------------------
           AUDIO STATE
           --------------------------------------------- */

        case ESP_A2D_AUDIO_STATE_EVT:

            switch (
                param->audio_stat.state
            ) {

                case ESP_A2D_AUDIO_STATE_STARTED:

                    tars_audio_started =
                        true;

                    tars_status_text =
                        "A2DP AUDIO STREAMING";

                    ESP_LOGI(
                        TARS_TAG,
                        "AUDIO STREAM STARTED"
                    );

                    break;


                case ESP_A2D_AUDIO_STATE_STOPPED:

                    tars_audio_started =
                        false;

                    if (
                        tars_a2dp_connected
                    ) {

                        tars_status_text =
                            "A2DP CONNECTED";

                    }

                    ESP_LOGI(
                        TARS_TAG,
                        "AUDIO STREAM STOPPED"
                    );

                    break;


                case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:

                    tars_audio_started =
                        false;

                    tars_status_text =
                        "A2DP AUDIO SUSPENDED";

                    ESP_LOGI(
                        TARS_TAG,
                        "AUDIO REMOTE SUSPEND"
                    );

                    break;


                default:

                    break;

            }

            break;


        default:

            break;

    }

}


/* =========================================================
   GAP CALLBACK
   ========================================================= */

static void tars_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param
) {

    switch (event) {


        /* ---------------------------------------------
           DISCOVERY RESULT
           --------------------------------------------- */

        case ESP_BT_GAP_DISC_RES_EVT: {

            uint8_t *eir =
                NULL;


            for (
                int i = 0;
                i < param->disc_res.num_prop;
                i++
            ) {

                esp_bt_gap_dev_prop_t *prop =
                    &param->disc_res.prop[i];


                if (
                    prop->type ==
                    ESP_BT_GAP_DEV_PROP_EIR
                ) {

                    eir =
                        (uint8_t *)
                        prop->val;

                }

            }


            if (
                eir != NULL
            ) {

                uint8_t name_len = 0;


                uint8_t *name =
                    esp_bt_gap_resolve_eir_data(
                        eir,
                        ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,
                        &name_len
                    );


                if (
                    name == NULL
                ) {

                    name =
                        esp_bt_gap_resolve_eir_data(
                            eir,
                            ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME,
                            &name_len
                        );

                }


                if (
                    name != NULL &&
                    name_len > 0
                ) {

                    if (
                        strlen(
                            TARS_TARGET_NAME
                        ) == name_len &&

                        memcmp(
                            name,
                            TARS_TARGET_NAME,
                            name_len
                        ) == 0
                    ) {

                        memcpy(
                            tars_target_bda,
                            param->disc_res.bda,
                            ESP_BD_ADDR_LEN
                        );


                        tars_device_found =
                            true;


                        ESP_LOGI(
                            TARS_TAG,
                            "TARGET FOUND"
                        );

                    }

                }

            }

            break;

        }


        /* ---------------------------------------------
           DISCOVERY STATE
           --------------------------------------------- */

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:

            if (
                param->disc_st_chg.state ==
                ESP_BT_GAP_DISCOVERY_STOPPED
            ) {

                tars_scanning =
                    false;

                ESP_LOGI(
                    TARS_TAG,
                    "SCAN STOPPED"
                );

            }

            break;


        default:

            break;

    }

}


/* =========================================================
   START BLUETOOTH
   ========================================================= */

static mp_obj_t tars_a2dp_start(void) {

    esp_err_t ret;


    if (
        tars_bt_started
    ) {

        return mp_obj_new_str(
            "TARS BLUETOOTH ALREADY STARTED",
            strlen(
                "TARS BLUETOOTH ALREADY STARTED"
            )
        );

    }


    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret =
        esp_bt_controller_init(
            &bt_cfg
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: BT CONTROLLER INIT FAILED",
            strlen(
                "ERROR: BT CONTROLLER INIT FAILED"
            )
        );

    }


    ret =
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: BT CONTROLLER ENABLE FAILED",
            strlen(
                "ERROR: BT CONTROLLER ENABLE FAILED"
            )
        );

    }


    ret =
        esp_bluedroid_init();


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: BLUEDROID INIT FAILED",
            strlen(
                "ERROR: BLUEDROID INIT FAILED"
            )
        );

    }


    ret =
        esp_bluedroid_enable();


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: BLUEDROID ENABLE FAILED",
            strlen(
                "ERROR: BLUEDROID ENABLE FAILED"
            )
        );

    }


    ret =
        esp_bt_gap_register_callback(
            tars_gap_callback
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: GAP CALLBACK FAILED",
            strlen(
                "ERROR: GAP CALLBACK FAILED"
            )
        );

    }


    ret =
        esp_bt_gap_set_device_name(
            TARS_DEVICE_NAME
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: SET DEVICE NAME FAILED",
            strlen(
                "ERROR: SET DEVICE NAME FAILED"
            )
        );

    }


    ret =
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: SET DISCOVERABLE FAILED",
            strlen(
                "ERROR: SET DISCOVERABLE FAILED"
            )
        );

    }


    ret =
        esp_a2d_register_callback(
            tars_a2dp_event_callback
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: A2DP CALLBACK FAILED",
            strlen(
                "ERROR: A2DP CALLBACK FAILED"
            )
        );

    }


    ret =
        esp_a2d_source_init();


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: A2DP SOURCE INIT FAILED",
            strlen(
                "ERROR: A2DP SOURCE INIT FAILED"
            )
        );

    }


    ret =
        esp_a2d_source_register_data_callback(
            tars_a2dp_data_callback
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: AUDIO CALLBACK FAILED",
            strlen(
                "ERROR: AUDIO CALLBACK FAILED"
            )
        );

    }


    tars_pcm_clear();


    tars_callback_count =
        0;

    tars_callback_bytes =
        0;


    tars_bt_started =
        true;


    tars_status_text =
        "TARS BLUETOOTH CLASSIC A2DP READY";


    return mp_obj_new_str(
        "TARS BLUETOOTH CLASSIC A2DP READY",
        strlen(
            "TARS BLUETOOTH CLASSIC A2DP READY"
        )
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_start_obj, tars_a2dp_start);


/* =========================================================
   SCAN
   ========================================================= */

static mp_obj_t tars_a2dp_scan(void) {

    if (
        !tars_bt_started
    ) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );

    }


    if (
        tars_scanning
    ) {

        return mp_obj_new_str(
            "TARS ALREADY SCANNING...",
            strlen(
                "TARS ALREADY SCANNING..."
            )
        );

    }


    tars_device_found =
        false;


    memset(
        tars_target_bda,
        0,
        ESP_BD_ADDR_LEN
    );


    esp_err_t ret =
        esp_bt_gap_start_discovery(
            ESP_BT_INQ_MODE_GENERAL_INQUIRY,
            10,
            0
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH SCAN FAILED",
            strlen(
                "ERROR: BLUETOOTH SCAN FAILED"
            )
        );

    }


    tars_scanning =
        true;


    return mp_obj_new_str(
        "TARS SCANNING FOR I7-TWS...",
        strlen(
            "TARS SCANNING FOR I7-TWS..."
        )
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_scan_obj, tars_a2dp_scan);


/* =========================================================
   FOUND
   ========================================================= */

static mp_obj_t tars_a2dp_found(void) {

    if (
        !tars_device_found
    ) {

        if (
            tars_scanning
        ) {

            return mp_obj_new_str(
                "STILL SCANNING...",
                strlen(
                    "STILL SCANNING..."
                )
            );

        }


        return mp_obj_new_str(
            "I7-TWS NOT FOUND",
            strlen(
                "I7-TWS NOT FOUND"
            )
        );

    }


    char result[64];


    snprintf(
        result,
        sizeof(result),

        "I7-TWS FOUND: %02X:%02X:%02X:%02X:%02X:%02X",

        tars_target_bda[0],
        tars_target_bda[1],
        tars_target_bda[2],
        tars_target_bda[3],
        tars_target_bda[4],
        tars_target_bda[5]
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_found_obj, tars_a2dp_found);


/* =========================================================
   CONNECT
   ========================================================= */

static mp_obj_t tars_a2dp_connect(void) {

    if (
        !tars_bt_started
    ) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );

    }


    if (
        !tars_device_found
    ) {

        return mp_obj_new_str(
            "ERROR: I7-TWS NOT FOUND",
            strlen(
                "ERROR: I7-TWS NOT FOUND"
            )
        );

    }


    if (
        tars_a2dp_connected
    ) {

        return mp_obj_new_str(
            "TARS ALREADY CONNECTED",
            strlen(
                "TARS ALREADY CONNECTED"
            )
        );

    }


    tars_callback_count =
        0;

    tars_callback_bytes =
        0;

    tars_pcm_clear();


    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: A2DP CONNECT FAILED",
            strlen(
                "ERROR: A2DP CONNECT FAILED"
            )
        );

    }


    tars_a2dp_connecting =
        true;


    tars_status_text =
        "A2DP CONNECTING";


    return mp_obj_new_str(
        "TARS CONNECTING TO I7-TWS...",
        strlen(
            "TARS CONNECTING TO I7-TWS..."
        )
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_connect_obj, tars_a2dp_connect);


/* =========================================================
   WRITE PCM
   ========================================================= */

static mp_obj_t tars_a2dp_write(
    mp_obj_t data_in
) {

    mp_buffer_info_t buffer_info;


    mp_get_buffer_raise(
        data_in,
        &buffer_info,
        MP_BUFFER_READ
    );


    if (
        !tars_a2dp_connected
    ) {

        return mp_obj_new_int(0);

    }


    size_t written =
        tars_pcm_write(
            (const uint8_t *)
            buffer_info.buf,

            buffer_info.len
        );


    return mp_obj_new_int_from_uint(
        written
    );

}

static MP_DEFINE_CONST_FUN_OBJ_1(tars_a2dp_write_obj, tars_a2dp_write);


/* =========================================================
   BUFFER STATUS
   ========================================================= */

static mp_obj_t tars_a2dp_buffer(void) {

    char result[80];

    size_t used;


    taskENTER_CRITICAL();

    used =
        pcm_used;

    taskEXIT_CRITICAL();


    snprintf(
        result,
        sizeof(result),

        "PCM BUFFER: %u / %u BYTES",

        (unsigned int)used,
        (unsigned int)PCM_BUFFER_SIZE
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_buffer_obj, tars_a2dp_buffer);


/* =========================================================
   AUDIO DEBUG
   ========================================================= */

static mp_obj_t tars_a2dp_audio_debug(void) {

    char result[128];


    snprintf(
        result,
        sizeof(result),

        "CALLBACKS=%lu BYTES=%lu AUDIO_STARTED=%d",

        (unsigned long)
        tars_callback_count,

        (unsigned long)
        tars_callback_bytes,

        tars_audio_started ? 1 : 0
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_audio_debug_obj,
    tars_a2dp_audio_debug
);


/* =========================================================
   CLEAR BUFFER
   ========================================================= */

static mp_obj_t tars_a2dp_clear(void) {

    tars_pcm_clear();


    return mp_obj_new_str(
        "PCM BUFFER CLEARED",
        strlen(
            "PCM BUFFER CLEARED"
        )
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_clear_obj, tars_a2dp_clear);


/* =========================================================
   STATUS
   ========================================================= */

static mp_obj_t tars_a2dp_status(void) {

    return mp_obj_new_str(
        tars_status_text,
        strlen(
            tars_status_text
        )
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_status_obj, tars_a2dp_status);


/* =========================================================
   TEST
   ========================================================= */

static mp_obj_t tars_a2dp_test(void) {

    if (
        tars_audio_started
    ) {

        return mp_obj_new_str(
            "TARS A2DP AUDIO STREAMING",
            strlen(
                "TARS A2DP AUDIO STREAMING"
            )
        );

    }


    if (
        tars_a2dp_connected
    ) {

        return mp_obj_new_str(
            "TARS A2DP CONNECTED",
            strlen(
                "TARS A2DP CONNECTED"
            )
        );

    }


    if (
        tars_a2dp_connecting
    ) {

        return mp_obj_new_str(
            "TARS A2DP CONNECTING",
            strlen(
                "TARS A2DP CONNECTING"
            )
        );

    }


    if (
        tars_bt_started
    ) {

        return mp_obj_new_str(
            "TARS A2DP STARTED",
            strlen(
                "TARS A2DP STARTED"
            )
        );

    }


    return mp_obj_new_str(
        "TARS A2DP READY",
        strlen(
            "TARS A2DP READY"
        )
    );

}

static MP_DEFINE_CONST_FUN_OBJ_0(tars_a2dp_test_obj, tars_a2dp_test);


/* =========================================================
   MICROPYTHON MODULE
   ========================================================= */

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

    {
        MP_ROM_QSTR(MP_QSTR_scan),
        MP_ROM_PTR(&tars_a2dp_scan_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_found),
        MP_ROM_PTR(&tars_a2dp_found_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_connect),
        MP_ROM_PTR(&tars_a2dp_connect_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_write),
        MP_ROM_PTR(&tars_a2dp_write_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_buffer),
        MP_ROM_PTR(&tars_a2dp_buffer_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_audio_debug),
        MP_ROM_PTR(&tars_a2dp_audio_debug_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_clear),
        MP_ROM_PTR(&tars_a2dp_clear_obj)
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


const mp_obj_module_t tars_a2dp_user_cmodule = {
    .base = {
        &mp_type_module
    },

    .globals =
        (mp_obj_dict_t *)
        &tars_a2dp_globals,
};


/* HARUS SATU BARIS */
MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
