#include "py/runtime.h"
#include "py/objstr.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

/* =========================================================
   TARS A2DP SOURCE

   ESP32
   Bluetooth Classic
   A2DP Source
   Scan + Connect
   PCM Streaming
   Internal Tone Test

   BLUETOOTH:
   - TARS V1 MAN dapat terlihat di HP
   - Scan I7-TWS
   - Discovery harus berhenti sebelum connect

   AUDIO:
   - PCM buffer statis
   - Tidak ada malloc di audio callback
   - Tone dibuat langsung di callback
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS V1 MAN"
#define TARS_TARGET_NAME "I7-TWS"

#define PCM_BUFFER_SIZE 8192

#define TARS_SAMPLE_RATE 44100
#define TARS_CHANNELS 2
#define TARS_BITS_PER_SAMPLE 16

#define TARS_FRAME_BYTES 4


/* =========================================================
   BLUETOOTH STATE
   ========================================================= */

static bool tars_bt_started = false;

static bool tars_scanning = false;
static bool tars_scan_stop_pending = false;

static bool tars_device_found = false;

static bool tars_a2dp_connected = false;
static bool tars_a2dp_connecting = false;

static bool tars_audio_started = false;

static bool tars_media_check_pending = false;
static bool tars_media_start_requested = false;
static bool tars_media_start_pending = false;
static bool tars_media_stop_pending = false;


/* =========================================================
   LAST CONNECTION INFORMATION
   ========================================================= */

static int tars_last_conn_state = -1;
static esp_err_t tars_last_error = ESP_OK;


/* =========================================================
   TARGET BLUETOOTH ADDRESS
   ========================================================= */

static esp_bd_addr_t tars_target_bda = {0};


/* =========================================================
   PCM RING BUFFER
   ========================================================= */

static uint8_t pcm_buffer[PCM_BUFFER_SIZE];

static volatile size_t pcm_read_pos = 0;
static volatile size_t pcm_write_pos = 0;
static volatile size_t pcm_used = 0;


/* =========================================================
   INTERNAL TONE
   ========================================================= */

static volatile bool tars_internal_tone = false;

static uint32_t tars_tone_phase = 0;
static uint32_t tars_tone_frequency = 440;


/* =========================================================
   STATUS
   ========================================================= */

static const char *tars_status_text =
    "TARS A2DP READY";


/* =========================================================
   CLEAR PCM BUFFER
   ========================================================= */

static void tars_pcm_clear(void)
{
    pcm_read_pos = 0;
    pcm_write_pos = 0;
    pcm_used = 0;
}


/* =========================================================
   PCM AVAILABLE SPACE
   ========================================================= */

static size_t tars_pcm_free(void)
{
    size_t used = pcm_used;

    if (used >= PCM_BUFFER_SIZE) {
        return 0;
    }

    return PCM_BUFFER_SIZE - used;
}


/* =========================================================
   WRITE PCM BUFFER
   ========================================================= */

static size_t tars_pcm_write(
    const uint8_t *data,
    size_t len
)
{
    size_t written = 0;

    if (data == NULL || len == 0) {
        return 0;
    }

    while (written < len) {

        if (pcm_used >= PCM_BUFFER_SIZE) {
            break;
        }

        pcm_buffer[pcm_write_pos] =
            data[written];

        pcm_write_pos++;

        if (pcm_write_pos >= PCM_BUFFER_SIZE) {
            pcm_write_pos = 0;
        }

        pcm_used++;
        written++;
    }

    return written;
}


/* =========================================================
   READ PCM BUFFER
   ========================================================= */

static size_t tars_pcm_read(
    uint8_t *data,
    size_t len
)
{
    size_t read_count = 0;

    if (data == NULL || len == 0) {
        return 0;
    }

    while (read_count < len) {

        if (pcm_used == 0) {
            break;
        }

        data[read_count] =
            pcm_buffer[pcm_read_pos];

        pcm_read_pos++;

        if (pcm_read_pos >= PCM_BUFFER_SIZE) {
            pcm_read_pos = 0;
        }

        pcm_used--;
        read_count++;
    }

    return read_count;
}


/* =========================================================
   GENERATE INTERNAL TONE
   ========================================================= */

static int32_t tars_generate_tone(
    uint8_t *data,
    int32_t len
)
{
    if (data == NULL || len <= 0) {
        return 0;
    }

    int32_t usable_len =
        len - (len % TARS_FRAME_BYTES);

    int32_t position = 0;

    uint32_t phase_increment =
        (uint32_t)(
            (
                (uint64_t)tars_tone_frequency *
                4294967296ULL
            )
            /
            TARS_SAMPLE_RATE
        );

    while (position < usable_len) {

        int16_t sample;

        if (tars_tone_phase & 0x80000000UL) {
            sample = 2500;
        } else {
            sample = -2500;
        }

        data[position + 0] =
            (uint8_t)(sample & 0xFF);

        data[position + 1] =
            (uint8_t)((sample >> 8) & 0xFF);

        data[position + 2] =
            (uint8_t)(sample & 0xFF);

        data[position + 3] =
            (uint8_t)((sample >> 8) & 0xFF);

        position += TARS_FRAME_BYTES;

        tars_tone_phase += phase_increment;
    }

    return usable_len;
}


/* =========================================================
   A2DP AUDIO DATA CALLBACK
   ========================================================= */

static int32_t tars_a2dp_data_callback(
    uint8_t *data,
    int32_t len
)
{
    if (data == NULL || len <= 0) {
        return 0;
    }

    if (!tars_audio_started) {

        memset(
            data,
            0,
            (size_t)len
        );

        return len;
    }

    if (tars_internal_tone) {

        int32_t generated =
            tars_generate_tone(
                data,
                len
            );

        if (generated < len) {

            memset(
                data + generated,
                0,
                (size_t)(len - generated)
            );
        }

        return len;
    }

    size_t received =
        tars_pcm_read(
            data,
            (size_t)len
        );

    if (received < (size_t)len) {

        memset(
            data + received,
            0,
            (size_t)len - received
        );
    }

    return len;
}


/* =========================================================
   REQUEST AUDIO START
   ========================================================= */

static esp_err_t tars_request_audio_start(void)
{
    if (
        !tars_bt_started ||
        !tars_a2dp_connected
    ) {
        return ESP_FAIL;
    }

    if (tars_audio_started) {
        return ESP_OK;
    }

    if (
        tars_media_check_pending ||
        tars_media_start_pending
    ) {
        return ESP_OK;
    }

    tars_media_start_requested = true;
    tars_media_check_pending = true;

    tars_status_text =
        "A2DP CHECKING SOURCE READY";

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
        );

    if (ret != ESP_OK) {

        tars_media_check_pending = false;
        tars_media_start_requested = false;

        tars_last_error = ret;

        tars_status_text =
            "A2DP SOURCE CHECK FAILED";
    }

    return ret;
}


/* =========================================================
   REQUEST AUDIO STOP
   ========================================================= */

static esp_err_t tars_request_audio_stop(void)
{
    if (!tars_a2dp_connected) {
        return ESP_FAIL;
    }

    if (!tars_audio_started) {
        return ESP_OK;
    }

    if (tars_media_stop_pending) {
        return ESP_OK;
    }

    tars_media_stop_pending = true;

    tars_status_text =
        "A2DP AUDIO STOPPING";

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_STOP
        );

    if (ret != ESP_OK) {

        tars_media_stop_pending = false;

        tars_last_error = ret;

        tars_status_text =
            "A2DP STOP REQUEST FAILED";
    }

    return ret;
}


/* =========================================================
   A2DP EVENT CALLBACK
   ========================================================= */

static void tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
)
{
    if (param == NULL) {
        return;
    }

    switch (event) {

        case ESP_A2D_CONNECTION_STATE_EVT:
        {
            tars_last_conn_state =
                param->conn_stat.state;

            switch (
                param->conn_stat.state
            ) {

                case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
                {
                    tars_a2dp_connected = false;
                    tars_a2dp_connecting = false;

                    tars_audio_started = false;

                    tars_media_check_pending = false;
                    tars_media_start_requested = false;
                    tars_media_start_pending = false;
                    tars_media_stop_pending = false;

                    tars_internal_tone = false;
                    tars_tone_phase = 0;

                    tars_pcm_clear();

                    tars_status_text =
                        "A2DP DISCONNECTED";

                    break;
                }


                case ESP_A2D_CONNECTION_STATE_CONNECTING:
                {
                    tars_a2dp_connected = false;
                    tars_a2dp_connecting = true;

                    tars_audio_started = false;

                    tars_status_text =
                        "A2DP CONNECTING";

                    break;
                }


                case ESP_A2D_CONNECTION_STATE_CONNECTED:
                {
                    tars_a2dp_connected = true;
                    tars_a2dp_connecting = false;

                    tars_audio_started = false;

                    tars_media_check_pending = false;
                    tars_media_start_requested = false;
                    tars_media_start_pending = false;
                    tars_media_stop_pending = false;

                    tars_status_text =
                        "A2DP CONNECTED";

                    break;
                }


                case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
                {
                    tars_a2dp_connected = false;

                    tars_status_text =
                        "A2DP DISCONNECTING";

                    break;
                }


                default:
                    break;
            }

            break;
        }


        case ESP_A2D_AUDIO_STATE_EVT:
        {
            switch (
                param->audio_stat.state
            ) {

                case ESP_A2D_AUDIO_STATE_STARTED:
                {
                    tars_audio_started = true;

                    tars_media_start_pending = false;
                    tars_media_start_requested = false;
                    tars_media_check_pending = false;
                    tars_media_stop_pending = false;

                    if (tars_internal_tone) {

                        tars_status_text =
                            "A2DP INTERNAL TONE STREAMING";

                    } else {

                        tars_status_text =
                            "A2DP AUDIO STREAMING";
                    }

                    break;
                }


                case ESP_A2D_AUDIO_STATE_STOPPED:
                {
                    tars_audio_started = false;

                    tars_media_start_pending = false;
                    tars_media_start_requested = false;
                    tars_media_check_pending = false;
                    tars_media_stop_pending = false;

                    tars_internal_tone = false;
                    tars_tone_phase = 0;

                    if (tars_a2dp_connected) {

                        tars_status_text =
                            "A2DP CONNECTED";

                    } else {

                        tars_status_text =
                            "A2DP AUDIO STOPPED";
                    }

                    break;
                }


                default:
                    break;
            }

            break;
        }


        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        {
            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
            ) {

                tars_media_check_pending = false;

                if (
                    param->media_ctrl_stat.status ==
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                ) {

                    if (
                        tars_media_start_requested &&
                        !tars_audio_started &&
                        tars_a2dp_connected
                    ) {

                        tars_media_start_pending = true;

                        tars_status_text =
                            "A2DP AUDIO STARTING";

                        esp_err_t ret =
                            esp_a2d_media_ctrl(
                                ESP_A2D_MEDIA_CTRL_START
                            );

                        if (ret != ESP_OK) {

                            tars_media_start_pending = false;
                            tars_media_start_requested = false;

                            tars_last_error = ret;

                            tars_status_text =
                                "A2DP START FAILED";
                        }
                    }

                } else {

                    tars_media_start_requested = false;
                    tars_media_start_pending = false;

                    tars_status_text =
                        "A2DP SOURCE NOT READY";
                }

                break;
            }


            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_START
            ) {

                if (
                    param->media_ctrl_stat.status !=
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                ) {

                    tars_media_start_pending = false;
                    tars_media_start_requested = false;

                    tars_status_text =
                        "A2DP START ACK FAILED";
                }

                break;
            }


            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_STOP
            ) {

                tars_media_stop_pending = false;

                if (
                    param->media_ctrl_stat.status ==
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                ) {

                    tars_status_text =
                        "A2DP STOP REQUEST ACCEPTED";

                } else {

                    tars_status_text =
                        "A2DP STOP ACK FAILED";
                }

                break;
            }

            break;
        }


        default:
            break;
    }
}


/* =========================================================
   BLUETOOTH GAP CALLBACK
   ========================================================= */

static void tars_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param
)
{
    if (param == NULL) {
        return;
    }

    switch (event) {

        case ESP_BT_GAP_DISC_RES_EVT:
        {
            if (tars_device_found) {
                break;
            }

            uint8_t *eir = NULL;

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
                        (uint8_t *)prop->val;
                }
            }

            if (eir != NULL) {

                uint8_t name_len = 0;

                uint8_t *name =
                    esp_bt_gap_resolve_eir_data(
                        eir,
                        ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,
                        &name_len
                    );

                if (name == NULL) {

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
                        strlen(TARS_TARGET_NAME) ==
                        name_len
                        &&
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

                        tars_device_found = true;

                        /*
                           JANGAN CONNECT DI CALLBACK.
                           Hanya hentikan discovery.
                        */

                        tars_scan_stop_pending =
                            true;

                        esp_bt_gap_cancel_discovery();
                    }
                }
            }

            break;
        }


        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        {
            if (
                param->disc_st_chg.state ==
                ESP_BT_GAP_DISCOVERY_STOPPED
            ) {

                tars_scanning = false;

                tars_scan_stop_pending =
                    false;

                if (tars_device_found) {

                    tars_status_text =
                        "I7-TWS FOUND - READY TO CONNECT";

                } else {

                    tars_status_text =
                        "TARS SCAN FINISHED";
                }
            }

            break;
        }


        default:
            break;
    }
}


/* =========================================================
   START BLUETOOTH
   ========================================================= */

static mp_obj_t
tars_a2dp_start(void)
{
    esp_err_t ret;

    if (tars_bt_started) {

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

    if (ret != ESP_OK) {

        tars_last_error = ret;

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

    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: BT CONTROLLER ENABLE FAILED",
            strlen(
                "ERROR: BT CONTROLLER ENABLE FAILED"
            )
        );
    }


    ret =
        esp_bluedroid_init();

    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: BLUEDROID INIT FAILED",
            strlen(
                "ERROR: BLUEDROID INIT FAILED"
            )
        );
    }


    ret =
        esp_bluedroid_enable();

    if (ret != ESP_OK) {

        tars_last_error = ret;

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

    if (ret != ESP_OK) {

        tars_last_error = ret;

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

    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: SET DEVICE NAME FAILED",
            strlen(
                "ERROR: SET DEVICE NAME FAILED"
            )
        );
    }


    /*
       TARS V1 MAN terlihat di HP
    */

    ret =
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        );

    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: SET SCAN MODE FAILED",
            strlen(
                "ERROR: SET SCAN MODE FAILED"
            )
        );
    }


    ret =
        esp_a2d_register_callback(
            tars_a2dp_event_callback
        );

    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: A2DP CALLBACK FAILED",
            strlen(
                "ERROR: A2DP CALLBACK FAILED"
            )
        );
    }


    ret =
        esp_a2d_source_init();

    if (ret != ESP_OK) {

        tars_last_error = ret;

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

    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: AUDIO CALLBACK FAILED",
            strlen(
                "ERROR: AUDIO CALLBACK FAILED"
            )
        );
    }


    tars_pcm_clear();

    tars_internal_tone = false;
    tars_tone_phase = 0;

    tars_scanning = false;
    tars_scan_stop_pending = false;

    tars_device_found = false;

    tars_a2dp_connected = false;
    tars_a2dp_connecting = false;

    tars_audio_started = false;

    tars_media_check_pending = false;
    tars_media_start_requested = false;
    tars_media_start_pending = false;
    tars_media_stop_pending = false;

    tars_last_conn_state = -1;
    tars_last_error = ESP_OK;


    memset(
        tars_target_bda,
        0,
        ESP_BD_ADDR_LEN
    );


    tars_bt_started = true;


    tars_status_text =
        "TARS BLUETOOTH CLASSIC A2DP READY";


    return mp_obj_new_str(
        "TARS BLUETOOTH CLASSIC A2DP READY",
        strlen(
            "TARS BLUETOOTH CLASSIC A2DP READY"
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);


/* =========================================================
   SCAN
   ========================================================= */

static mp_obj_t
tars_a2dp_scan(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );
    }


    if (tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: ALREADY CONNECTED",
            strlen(
                "ERROR: ALREADY CONNECTED"
            )
        );
    }


    if (tars_scanning) {

        return mp_obj_new_str(
            "TARS ALREADY SCANNING...",
            strlen(
                "TARS ALREADY SCANNING..."
            )
        );
    }


    if (tars_scan_stop_pending) {

        return mp_obj_new_str(
            "WAITING FOR SCAN TO STOP...",
            strlen(
                "WAITING FOR SCAN TO STOP..."
            )
        );
    }


    tars_device_found = false;

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


    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: BLUETOOTH SCAN FAILED",
            strlen(
                "ERROR: BLUETOOTH SCAN FAILED"
            )
        );
    }


    tars_scanning = true;

    tars_status_text =
        "TARS SCANNING";


    return mp_obj_new_str(
        "TARS SCANNING FOR I7-TWS...",
        strlen(
            "TARS SCANNING FOR I7-TWS..."
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_scan_obj,
    tars_a2dp_scan
);


/* =========================================================
   FOUND
   ========================================================= */

static mp_obj_t
tars_a2dp_found(void)
{
    if (!tars_device_found) {

        if (
            tars_scanning ||
            tars_scan_stop_pending
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


    if (
        tars_scanning ||
        tars_scan_stop_pending
    ) {

        return mp_obj_new_str(
            "I7-TWS FOUND - WAIT SCAN STOP",
            strlen(
                "I7-TWS FOUND - WAIT SCAN STOP"
            )
        );
    }


    char result[80];


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

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_found_obj,
    tars_a2dp_found
);


/* =========================================================
   CONNECT
   ========================================================= */

static mp_obj_t
tars_a2dp_connect(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );
    }


    if (!tars_device_found) {

        return mp_obj_new_str(
            "ERROR: I7-TWS NOT FOUND",
            strlen(
                "ERROR: I7-TWS NOT FOUND"
            )
        );
    }


    if (tars_a2dp_connected) {

        return mp_obj_new_str(
            "TARS ALREADY CONNECTED",
            strlen(
                "TARS ALREADY CONNECTED"
            )
        );
    }


    if (tars_a2dp_connecting) {

        return mp_obj_new_str(
            "TARS ALREADY CONNECTING",
            strlen(
                "TARS ALREADY CONNECTING"
            )
        );
    }


    /*
       PENTING:
       Jangan connect selama discovery aktif.
    */

    if (
        tars_scanning ||
        tars_scan_stop_pending
    ) {

        if (tars_scanning) {

            esp_bt_gap_cancel_discovery();

            tars_scan_stop_pending =
                true;
        }


        return mp_obj_new_str(
            "WAIT FOR SCAN TO FULLY STOP THEN CONNECT AGAIN",
            strlen(
                "WAIT FOR SCAN TO FULLY STOP THEN CONNECT AGAIN"
            )
        );
    }


    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );


    if (ret != ESP_OK) {

        tars_last_error = ret;

        return mp_obj_new_str(
            "ERROR: A2DP CONNECT REQUEST FAILED",
            strlen(
                "ERROR: A2DP CONNECT REQUEST FAILED"
            )
        );
    }


    tars_a2dp_connecting = true;

    tars_status_text =
        "A2DP CONNECTING";


    return mp_obj_new_str(
        "TARS CONNECTING TO I7-TWS...",
        strlen(
            "TARS CONNECTING TO I7-TWS..."
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_connect_obj,
    tars_a2dp_connect
);


/* =========================================================
   PLAY PCM
   ========================================================= */

static mp_obj_t
tars_a2dp_play(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );
    }


    if (!tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }


    if (tars_audio_started) {

        return mp_obj_new_str(
            "A2DP AUDIO ALREADY STREAMING",
            strlen(
                "A2DP AUDIO ALREADY STREAMING"
            )
        );
    }


    if (pcm_used == 0) {

        return mp_obj_new_str(
            "ERROR: PCM BUFFER EMPTY - WRITE AUDIO FIRST",
            strlen(
                "ERROR: PCM BUFFER EMPTY - WRITE AUDIO FIRST"
            )
        );
    }


    tars_internal_tone = false;


    esp_err_t ret =
        tars_request_audio_start();


    if (ret != ESP_OK) {

        return mp_obj_new_str(
            "ERROR: A2DP SOURCE NOT READY",
            strlen(
                "ERROR: A2DP SOURCE NOT READY"
            )
        );
    }


    return mp_obj_new_str(
        "TARS A2DP START REQUESTED - WAITING FOR ACK",
        strlen(
            "TARS A2DP START REQUESTED - WAITING FOR ACK"
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_play_obj,
    tars_a2dp_play
);


/* =========================================================
   INTERNAL TONE
   ========================================================= */

static mp_obj_t
tars_a2dp_tone(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );
    }


    if (!tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }


    tars_internal_tone = true;

    tars_tone_phase = 0;

    tars_tone_frequency = 440;


    if (tars_audio_started) {

        tars_status_text =
            "A2DP INTERNAL TONE STREAMING";


        return mp_obj_new_str(
            "TARS INTERNAL 440HZ TONE ACTIVE",
            strlen(
                "TARS INTERNAL 440HZ TONE ACTIVE"
            )
        );
    }


    if (
        tars_media_check_pending ||
        tars_media_start_pending
    ) {

        return mp_obj_new_str(
            "TARS TONE START ALREADY PENDING",
            strlen(
                "TARS TONE START ALREADY PENDING"
            )
        );
    }


    esp_err_t ret =
        tars_request_audio_start();


    if (ret != ESP_OK) {

        tars_internal_tone = false;

        return mp_obj_new_str(
            "ERROR: TONE SOURCE NOT READY",
            strlen(
                "ERROR: TONE SOURCE NOT READY"
            )
        );
    }


    return mp_obj_new_str(
        "TARS INTERNAL TONE REQUESTED - WAITING FOR ACK",
        strlen(
            "TARS INTERNAL TONE REQUESTED - WAITING FOR ACK"
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tone_obj,
    tars_a2dp_tone
);


/* =========================================================
   STOP INTERNAL TONE
   ========================================================= */

static mp_obj_t
tars_a2dp_tone_stop(void)
{
    tars_internal_tone = false;

    tars_tone_phase = 0;


    if (
        tars_audio_started &&
        tars_a2dp_connected
    ) {

        if (!tars_media_stop_pending) {

            tars_request_audio_stop();
        }
    }


    return mp_obj_new_str(
        "TARS INTERNAL TONE STOPPED",
        strlen(
            "TARS INTERNAL TONE STOPPED"
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tone_stop_obj,
    tars_a2dp_tone_stop
);


/* =========================================================
   STOP AUDIO STREAM
   ========================================================= */

static mp_obj_t
tars_a2dp_stop(void)
{
    if (!tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }


    tars_internal_tone = false;

    tars_tone_phase = 0;


    if (!tars_audio_started) {

        return mp_obj_new_str(
            "A2DP AUDIO ALREADY STOPPED",
            strlen(
                "A2DP AUDIO ALREADY STOPPED"
            )
        );
    }


    if (tars_media_stop_pending) {

        return mp_obj_new_str(
            "A2DP STOP ALREADY PENDING",
            strlen(
                "A2DP STOP ALREADY PENDING"
            )
        );
    }


    esp_err_t ret =
        tars_request_audio_stop();


    if (ret != ESP_OK) {

        return mp_obj_new_str(
            "ERROR: A2DP STOP FAILED",
            strlen(
                "ERROR: A2DP STOP FAILED"
            )
        );
    }


    return mp_obj_new_str(
        "TARS A2DP STOP REQUESTED",
        strlen(
            "TARS A2DP STOP REQUESTED"
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_stop_obj,
    tars_a2dp_stop
);


/* =========================================================
   WRITE PCM
   ========================================================= */

static mp_obj_t
tars_a2dp_write(
    mp_obj_t data_in
)
{
    mp_buffer_info_t buffer_info;


    mp_get_buffer_raise(
        data_in,
        &buffer_info,
        MP_BUFFER_READ
    );


    if (!tars_a2dp_connected) {

        return mp_obj_new_int(0);
    }


    tars_internal_tone = false;


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

static MP_DEFINE_CONST_FUN_OBJ_1(
    tars_a2dp_write_obj,
    tars_a2dp_write
);


/* =========================================================
   BUFFER STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_buffer(void)
{
    char result[96];


    size_t used =
        pcm_used;


    size_t free_space =
        tars_pcm_free();


    snprintf(
        result,
        sizeof(result),
        "PCM BUFFER: %u / %u BYTES | FREE: %u",
        (unsigned int)used,
        (unsigned int)PCM_BUFFER_SIZE,
        (unsigned int)free_space
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_buffer_obj,
    tars_a2dp_buffer
);


/* =========================================================
   CLEAR BUFFER
   ========================================================= */

static mp_obj_t
tars_a2dp_clear(void)
{
    tars_pcm_clear();


    return mp_obj_new_str(
        "PCM BUFFER CLEARED",
        strlen(
            "PCM BUFFER CLEARED"
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_clear_obj,
    tars_a2dp_clear
);


/* =========================================================
   STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_status(void)
{
    return mp_obj_new_str(
        tars_status_text,
        strlen(
            tars_status_text
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_status_obj,
    tars_a2dp_status
);


/* =========================================================
   DIAGNOSTIC
   ========================================================= */

static mp_obj_t
tars_a2dp_diag(void)
{
    char result[160];


    snprintf(
        result,
        sizeof(result),
        "BT=%d SCAN=%d STOPPING=%d FOUND=%d CONNECTING=%d CONNECTED=%d AUDIO=%d CONN_STATE=%d LAST_ERR=%d",
        (int)tars_bt_started,
        (int)tars_scanning,
        (int)tars_scan_stop_pending,
        (int)tars_device_found,
        (int)tars_a2dp_connecting,
        (int)tars_a2dp_connected,
        (int)tars_audio_started,
        tars_last_conn_state,
        (int)tars_last_error
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_diag_obj,
    tars_a2dp_diag
);


/* =========================================================
   TEST
   ========================================================= */

static mp_obj_t
tars_a2dp_test(void)
{
    if (tars_media_stop_pending) {

        return mp_obj_new_str(
            "TARS A2DP STOPPING",
            strlen(
                "TARS A2DP STOPPING"
            )
        );
    }


    if (
        tars_internal_tone &&
        tars_audio_started
    ) {

        return mp_obj_new_str(
            "TARS INTERNAL TONE STREAMING",
            strlen(
                "TARS INTERNAL TONE STREAMING"
            )
        );
    }


    if (tars_media_start_pending) {

        return mp_obj_new_str(
            "TARS A2DP START PENDING",
            strlen(
                "TARS A2DP START PENDING"
            )
        );
    }


    if (tars_media_check_pending) {

        return mp_obj_new_str(
            "TARS A2DP CHECKING SOURCE",
            strlen(
                "TARS A2DP CHECKING SOURCE"
            )
        );
    }


    if (tars_audio_started) {

        return mp_obj_new_str(
            "TARS A2DP AUDIO STREAMING",
            strlen(
                "TARS A2DP AUDIO STREAMING"
            )
        );
    }


    if (tars_a2dp_connected) {

        return mp_obj_new_str(
            "TARS A2DP CONNECTED",
            strlen(
                "TARS A2DP CONNECTED"
            )
        );
    }


    if (tars_a2dp_connecting) {

        return mp_obj_new_str(
            "TARS A2DP CONNECTING",
            strlen(
                "TARS A2DP CONNECTING"
            )
        );
    }


    if (
        tars_scanning ||
        tars_scan_stop_pending
    ) {

        return mp_obj_new_str(
            "TARS A2DP SCANNING",
            strlen(
                "TARS A2DP SCANNING"
            )
        );
    }


    if (
        tars_device_found &&
        !tars_scanning
    ) {

        return mp_obj_new_str(
            "TARS I7-TWS FOUND READY TO CONNECT",
            strlen(
                "TARS I7-TWS FOUND READY TO CONNECT"
            )
        );
    }


    if (tars_bt_started) {

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

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


/* =========================================================
   MICROPYTHON MODULE
   ========================================================= */

static const mp_rom_map_elem_t
tars_a2dp_globals_table[] =
{
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
            MP_QSTR_start
        ),

        MP_ROM_PTR(
            &tars_a2dp_start_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_scan
        ),

        MP_ROM_PTR(
            &tars_a2dp_scan_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_found
        ),

        MP_ROM_PTR(
            &tars_a2dp_found_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_connect
        ),

        MP_ROM_PTR(
            &tars_a2dp_connect_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_play
        ),

        MP_ROM_PTR(
            &tars_a2dp_play_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_tone
        ),

        MP_ROM_PTR(
            &tars_a2dp_tone_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_tone_stop
        ),

        MP_ROM_PTR(
            &tars_a2dp_tone_stop_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_stop
        ),

        MP_ROM_PTR(
            &tars_a2dp_stop_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_write
        ),

        MP_ROM_PTR(
            &tars_a2dp_write_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_buffer
        ),

        MP_ROM_PTR(
            &tars_a2dp_buffer_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_clear
        ),

        MP_ROM_PTR(
            &tars_a2dp_clear_obj
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


    {
        MP_ROM_QSTR(
            MP_QSTR_diag
        ),

        MP_ROM_PTR(
            &tars_a2dp_diag_obj
        )
    }
};


static MP_DEFINE_CONST_DICT(
    tars_a2dp_globals,
    tars_a2dp_globals_table
);


/* =========================================================
   MODULE
   ========================================================= */

const mp_obj_module_t
tars_a2dp_user_cmodule =
{
    .base =
    {
        &mp_type_module
    },

    .globals =
        (mp_obj_dict_t *)
        &tars_a2dp_globals
};


/* =========================================================
   REGISTER MODULE
   ========================================================= */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
