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
   Classic BT Pairing Diagnostics
   PCM Streaming
   Internal Tone Test
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS"
#define TARS_TARGET_NAME "I7-TWS"

#define PCM_BUFFER_SIZE 2048

#define TARS_SAMPLE_RATE 44100
#define TARS_CHANNELS 2
#define TARS_BITS_PER_SAMPLE 16


/* =========================================================
   BLUETOOTH STATE
   ========================================================= */

static bool tars_bt_started = false;
static bool tars_scanning = false;
static bool tars_device_found = false;

static bool tars_a2dp_connected = false;
static bool tars_a2dp_connecting = false;

static bool tars_connect_requested = false;
static bool tars_connect_after_scan_stop = false;

static bool tars_audio_started = false;

static bool tars_media_check_pending = false;
static bool tars_media_start_requested = false;
static bool tars_media_start_pending = false;
static bool tars_media_stop_pending = false;


/* =========================================================
   PAIRING / AUTH STATE
   ========================================================= */

static bool tars_auth_success = false;
static bool tars_auth_failed = false;

static char tars_auth_status[96] =
    "AUTH NOT STARTED";


/* =========================================================
   TARGET BLUETOOTH ADDRESS
   ========================================================= */

static esp_bd_addr_t tars_target_bda = {0};


/* =========================================================
   LAST CONNECTION DIAGNOSTIC
   ========================================================= */

static char tars_last_diag[128] =
    "TARS A2DP READY";


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

static bool tars_internal_tone = false;

static uint32_t tars_tone_phase = 0;
static uint32_t tars_tone_frequency = 440;


/* =========================================================
   STATUS
   ========================================================= */

static const char *tars_status_text =
    "TARS A2DP READY";


/* =========================================================
   HELPER: SET DIAGNOSTIC
   ========================================================= */

static void tars_set_diag(
    const char *text
)
{
    if (
        text == NULL
    ) {
        return;
    }

    snprintf(
        tars_last_diag,
        sizeof(tars_last_diag),
        "%s",
        text
    );
}


/* =========================================================
   HELPER: FORMAT BDA
   ========================================================= */

static void tars_bda_to_string(
    const esp_bd_addr_t bda,
    char *result,
    size_t result_size
)
{
    if (
        result == NULL ||
        result_size == 0
    ) {
        return;
    }

    snprintf(
        result,
        result_size,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        bda[0],
        bda[1],
        bda[2],
        bda[3],
        bda[4],
        bda[5]
    );
}


/* =========================================================
   HELPER: A2DP DISCONNECT REASON
   ========================================================= */

static const char *tars_a2dp_disc_reason_text(
    esp_a2d_disc_rsn_t reason
)
{
    switch (
        reason
    ) {
        case ESP_A2D_DISC_RSN_NORMAL:
            return "NORMAL";

        case ESP_A2D_DISC_RSN_ABNORMAL:
            return "ABNORMAL";

        default:
            return "UNKNOWN";
    }
}


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
   WRITE PCM BUFFER
   ========================================================= */

static size_t tars_pcm_write(
    const uint8_t *data,
    size_t len
)
{
    size_t written = 0;

    if (
        data == NULL ||
        len == 0
    ) {
        return 0;
    }

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
    size_t read = 0;

    if (
        data == NULL ||
        len == 0
    ) {
        return 0;
    }

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

    return read;
}


/* =========================================================
   GENERATE INTERNAL TONE
   ========================================================= */

static int32_t tars_generate_tone(
    uint8_t *data,
    int32_t len
)
{
    if (
        data == NULL ||
        len <= 0
    ) {
        return 0;
    }

    int32_t usable_len =
        len - (len % 4);

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

    while (
        position < usable_len
    ) {
        int16_t sample;

        if (
            tars_tone_phase &
            0x80000000UL
        ) {
            sample = 5000;
        }
        else {
            sample = -5000;
        }

        data[position + 0] =
            (uint8_t)(
                sample & 0xFF
            );

        data[position + 1] =
            (uint8_t)(
                (sample >> 8) & 0xFF
            );

        data[position + 2] =
            (uint8_t)(
                sample & 0xFF
            );

        data[position + 3] =
            (uint8_t)(
                (sample >> 8) & 0xFF
            );

        position += 4;

        tars_tone_phase +=
            phase_increment;
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
    if (
        data == NULL
    ) {
        return 0;
    }

    /*
       ESP-IDF dapat mengirim len = -1
       untuk meminta flush buffer.
    */

    if (
        len == -1
    ) {
        tars_pcm_clear();

        return 0;
    }

    if (
        len <= 0
    ) {
        return 0;
    }

    if (
        tars_internal_tone
    ) {
        return tars_generate_tone(
            data,
            len
        );
    }

    size_t received =
        tars_pcm_read(
            data,
            (size_t)len
        );

    if (
        received < (size_t)len
    ) {
        memset(
            data + received,
            0,
            (size_t)len - received
        );
    }

    return len;
}


/* =========================================================
   INTERNAL CONNECT REQUEST
   ========================================================= */

static esp_err_t tars_start_a2dp_connection(void)
{
    if (
        !tars_bt_started
    ) {
        return ESP_FAIL;
    }

    if (
        !tars_device_found
    ) {
        return ESP_FAIL;
    }

    if (
        tars_a2dp_connected
    ) {
        return ESP_OK;
    }

    if (
        tars_a2dp_connecting
    ) {
        return ESP_OK;
    }

    char bda_text[32];

    tars_bda_to_string(
        tars_target_bda,
        bda_text,
        sizeof(bda_text)
    );

    char diag[128];

    snprintf(
        diag,
        sizeof(diag),
        "A2DP CONNECT REQUEST %s",
        bda_text
    );

    tars_set_diag(
        diag
    );

    tars_status_text =
        "A2DP CONNECTING";

    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );

    if (
        ret == ESP_OK
    ) {
        tars_a2dp_connecting =
            true;

        tars_connect_requested =
            false;

        tars_connect_after_scan_stop =
            false;
    }
    else {
        snprintf(
            diag,
            sizeof(diag),
            "A2DP CONNECT API ERROR %d",
            (int)ret
        );

        tars_set_diag(
            diag
        );

        tars_status_text =
            "A2DP CONNECT API FAILED";

        tars_a2dp_connecting =
            false;

        tars_connect_requested =
            false;

        tars_connect_after_scan_stop =
            false;
    }

    return ret;
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

    if (
        tars_audio_started
    ) {
        return ESP_OK;
    }

    if (
        tars_media_check_pending ||
        tars_media_start_pending
    ) {
        return ESP_OK;
    }

    tars_media_start_requested =
        true;

    tars_media_check_pending =
        true;

    tars_status_text =
        "A2DP CHECKING SOURCE READY";

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
        );

    if (
        ret != ESP_OK
    ) {
        tars_media_check_pending =
            false;

        tars_media_start_requested =
            false;

        tars_status_text =
            "A2DP SOURCE CHECK FAILED";

        char diag[96];

        snprintf(
            diag,
            sizeof(diag),
            "MEDIA CHECK API ERROR %d",
            (int)ret
        );

        tars_set_diag(
            diag
        );
    }

    return ret;
}


/* =========================================================
   REQUEST AUDIO STOP
   ========================================================= */

static esp_err_t tars_request_audio_stop(void)
{
    if (
        !tars_a2dp_connected
    ) {
        return ESP_FAIL;
    }

    if (
        !tars_audio_started
    ) {
        return ESP_OK;
    }

    if (
        tars_media_stop_pending
    ) {
        return ESP_OK;
    }

    tars_media_stop_pending =
        true;

    tars_status_text =
        "A2DP AUDIO STOPPING";

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_STOP
        );

    if (
        ret != ESP_OK
    ) {
        tars_media_stop_pending =
            false;

        char diag[96];

        snprintf(
            diag,
            sizeof(diag),
            "MEDIA STOP API ERROR %d",
            (int)ret
        );

        tars_set_diag(
            diag
        );
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
    if (
        param == NULL
    ) {
        return;
    }

    switch (
        event
    ) {

        case ESP_A2D_CONNECTION_STATE_EVT:
        {
            char bda_text[32];

            tars_bda_to_string(
                param->conn_stat.remote_bda,
                bda_text,
                sizeof(bda_text)
            );

            switch (
                param->conn_stat.state
            ) {

                case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
                {
                    tars_a2dp_connected =
                        false;

                    tars_a2dp_connecting =
                        false;

                    tars_audio_started =
                        false;

                    tars_media_check_pending =
                        false;

                    tars_media_start_requested =
                        false;

                    tars_media_start_pending =
                        false;

                    tars_media_stop_pending =
                        false;

                    tars_internal_tone =
                        false;

                    tars_connect_requested =
                        false;

                    tars_connect_after_scan_stop =
                        false;

                    tars_pcm_clear();

                    char diag[128];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "A2DP DISCONNECTED %s RSN=%s(%d)",
                        bda_text,
                        tars_a2dp_disc_reason_text(
                            param->conn_stat.disc_rsn
                        ),
                        (int)param->conn_stat.disc_rsn
                    );

                    tars_set_diag(
                        diag
                    );

                    tars_status_text =
                        "A2DP DISCONNECTED";

                    break;
                }


                case ESP_A2D_CONNECTION_STATE_CONNECTING:
                {
                    tars_a2dp_connected =
                        false;

                    tars_a2dp_connecting =
                        true;

                    tars_audio_started =
                        false;

                    char diag[128];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "A2DP CONNECTING %s",
                        bda_text
                    );

                    tars_set_diag(
                        diag
                    );

                    tars_status_text =
                        "A2DP CONNECTING";

                    break;
                }


                case ESP_A2D_CONNECTION_STATE_CONNECTED:
                {
                    tars_a2dp_connected =
                        true;

                    tars_a2dp_connecting =
                        false;

                    tars_connect_requested =
                        false;

                    tars_connect_after_scan_stop =
                        false;

                    tars_audio_started =
                        false;

                    tars_media_check_pending =
                        false;

                    tars_media_start_requested =
                        false;

                    tars_media_start_pending =
                        false;

                    tars_media_stop_pending =
                        false;

                    char diag[128];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "A2DP CONNECTED %s MTU=%u",
                        bda_text,
                        (unsigned int)
                        param->conn_stat.audio_mtu
                    );

                    tars_set_diag(
                        diag
                    );

                    tars_status_text =
                        "A2DP CONNECTED";

                    break;
                }


                case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
                {
                    tars_a2dp_connected =
                        false;

                    tars_a2dp_connecting =
                        false;

                    tars_audio_started =
                        false;

                    char diag[128];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "A2DP DISCONNECTING %s",
                        bda_text
                    );

                    tars_set_diag(
                        diag
                    );

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
                    tars_audio_started =
                        true;

                    tars_media_start_pending =
                        false;

                    tars_media_start_requested =
                        false;

                    tars_media_check_pending =
                        false;

                    tars_media_stop_pending =
                        false;

                    if (
                        tars_internal_tone
                    ) {
                        tars_status_text =
                            "A2DP INTERNAL TONE STREAMING";
                    }
                    else {
                        tars_status_text =
                            "A2DP AUDIO STREAMING";
                    }

                    tars_set_diag(
                        "A2DP AUDIO STARTED"
                    );

                    break;
                }


                case ESP_A2D_AUDIO_STATE_STOPPED:
                {
                    tars_audio_started =
                        false;

                    tars_media_start_pending =
                        false;

                    tars_media_start_requested =
                        false;

                    tars_media_check_pending =
                        false;

                    tars_media_stop_pending =
                        false;

                    if (
                        tars_a2dp_connected
                    ) {
                        tars_status_text =
                            "A2DP CONNECTED";
                    }
                    else {
                        tars_status_text =
                            "A2DP AUDIO STOPPED";
                    }

                    tars_set_diag(
                        "A2DP AUDIO STOPPED"
                    );

                    break;
                }


                case ESP_A2D_AUDIO_STATE_SUSPEND:
                {
                    tars_audio_started =
                        false;

                    tars_status_text =
                        "A2DP AUDIO SUSPENDED";

                    tars_set_diag(
                        "A2DP AUDIO SUSPENDED"
                    );

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
                tars_media_check_pending =
                    false;

                if (
                    param->media_ctrl_stat.status ==
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                ) {
                    if (
                        tars_media_start_requested &&
                        !tars_audio_started &&
                        tars_a2dp_connected
                    ) {
                        tars_media_start_pending =
                            true;

                        tars_status_text =
                            "A2DP AUDIO STARTING";

                        esp_err_t ret =
                            esp_a2d_media_ctrl(
                                ESP_A2D_MEDIA_CTRL_START
                            );

                        if (
                            ret != ESP_OK
                        ) {
                            tars_media_start_pending =
                                false;

                            tars_media_start_requested =
                                false;

                            tars_status_text =
                                "A2DP START FAILED";

                            char diag[96];

                            snprintf(
                                diag,
                                sizeof(diag),
                                "MEDIA START API ERROR %d",
                                (int)ret
                            );

                            tars_set_diag(
                                diag
                            );
                        }
                    }
                }
                else {
                    tars_media_start_requested =
                        false;

                    tars_media_start_pending =
                        false;

                    tars_status_text =
                        "A2DP SOURCE NOT READY";

                    char diag[96];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "MEDIA CHECK ACK FAILED %d",
                        (int)param->media_ctrl_stat.status
                    );

                    tars_set_diag(
                        diag
                    );
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
                    tars_media_start_pending =
                        false;

                    tars_media_start_requested =
                        false;

                    tars_status_text =
                        "A2DP START ACK FAILED";

                    char diag[96];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "MEDIA START ACK FAILED %d",
                        (int)param->media_ctrl_stat.status
                    );

                    tars_set_diag(
                        diag
                    );
                }

                break;
            }


            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_STOP
            ) {
                tars_media_stop_pending =
                    false;

                if (
                    param->media_ctrl_stat.status ==
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                ) {
                    tars_status_text =
                        "A2DP STOP REQUEST ACCEPTED";

                    tars_set_diag(
                        "MEDIA STOP ACK SUCCESS"
                    );
                }
                else {
                    tars_status_text =
                        "A2DP STOP ACK FAILED";

                    char diag[96];

                    snprintf(
                        diag,
                        sizeof(diag),
                        "MEDIA STOP ACK FAILED %d",
                        (int)param->media_ctrl_stat.status
                    );

                    tars_set_diag(
                        diag
                    );
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
    if (
        param == NULL
    ) {
        return;
    }

    switch (
        event
    ) {

        case ESP_BT_GAP_DISC_RES_EVT:
        {
            if (
                tars_device_found
            ) {
                break;
            }

            uint8_t *eir =
                NULL;

            uint8_t *name_prop =
                NULL;

            int name_prop_len =
                0;

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

                if (
                    prop->type ==
                    ESP_BT_GAP_DEV_PROP_BDNAME
                ) {
                    name_prop =
                        (uint8_t *)prop->val;

                    name_prop_len =
                        prop->len;
                }
            }

            uint8_t *name =
                NULL;

            uint8_t name_len =
                0;

            if (
                eir != NULL
            ) {
                name =
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
            }

            bool target_match =
                false;

            if (
                name != NULL &&
                name_len > 0
            ) {
                if (
                    strlen(
                        TARS_TARGET_NAME
                    ) == name_len
                    &&
                    memcmp(
                        name,
                        TARS_TARGET_NAME,
                        name_len
                    ) == 0
                ) {
                    target_match =
                        true;
                }
            }

            if (
                !target_match &&
                name_prop != NULL &&
                name_prop_len > 0
            ) {
                size_t target_len =
                    strlen(
                        TARS_TARGET_NAME
                    );

                if (
                    (size_t)name_prop_len ==
                    target_len
                    &&
                    memcmp(
                        name_prop,
                        TARS_TARGET_NAME,
                        target_len
                    ) == 0
                ) {
                    target_match =
                        true;
                }
            }

            if (
                target_match
            ) {
                memcpy(
                    tars_target_bda,
                    param->disc_res.bda,
                    ESP_BD_ADDR_LEN
                );

                tars_device_found =
                    true;

                char bda_text[32];

                tars_bda_to_string(
                    tars_target_bda,
                    bda_text,
                    sizeof(bda_text)
                );

                char diag[128];

                snprintf(
                    diag,
                    sizeof(diag),
                    "TARGET FOUND I7-TWS %s",
                    bda_text
                );

                tars_set_diag(
                    diag
                );

                tars_status_text =
                    "TARS TARGET FOUND";

                /*
                   Jangan langsung connect dari callback.
                   Minta discovery dihentikan dulu.
                */

                if (
                    tars_scanning
                ) {
                    esp_err_t ret =
                        esp_bt_gap_cancel_discovery();

                    if (
                        ret == ESP_OK
                    ) {
                        tars_connect_after_scan_stop =
                            tars_connect_requested;
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
                tars_scanning =
                    false;

                if (
                    tars_connect_after_scan_stop &&
                    tars_device_found &&
                    !tars_a2dp_connected &&
                    !tars_a2dp_connecting
                ) {
                    tars_start_a2dp_connection();
                }
            }

            break;
        }


        case ESP_BT_GAP_AUTH_CMPL_EVT:
        {
            char bda_text[32];

            tars_bda_to_string(
                param->auth_cmpl.bda,
                bda_text,
                sizeof(bda_text)
            );

            if (
                param->auth_cmpl.stat ==
                ESP_BT_STATUS_SUCCESS
            ) {
                tars_auth_success =
                    true;

                tars_auth_failed =
                    false;

                snprintf(
                    tars_auth_status,
                    sizeof(tars_auth_status),
                    "AUTH SUCCESS %s",
                    bda_text
                );

                tars_set_diag(
                    tars_auth_status
                );
            }
            else {
                tars_auth_success =
                    false;

                tars_auth_failed =
                    true;

                snprintf(
                    tars_auth_status,
                    sizeof(tars_auth_status),
                    "AUTH FAILED %s STATUS=%d",
                    bda_text,
                    (int)param->auth_cmpl.stat
                );

                tars_set_diag(
                    tars_auth_status
                );
            }

            break;
        }


        case ESP_BT_GAP_PIN_REQ_EVT:
        {
            esp_bt_pin_code_t pin_code;

            memset(
                pin_code,
                0,
                sizeof(pin_code)
            );

            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';

            esp_bt_gap_pin_reply(
                param->pin_req.bda,
                true,
                4,
                pin_code
            );

            snprintf(
                tars_auth_status,
                sizeof(tars_auth_status),
                "PIN REPLY SENT 1234"
            );

            tars_set_diag(
                tars_auth_status
            );

            break;
        }


        case ESP_BT_GAP_CFM_REQ_EVT:
        {
            esp_bt_gap_ssp_confirm_reply(
                param->cfm_req.bda,
                true
            );

            snprintf(
                tars_auth_status,
                sizeof(tars_auth_status),
                "SSP CONFIRM ACCEPTED"
            );

            tars_set_diag(
                tars_auth_status
            );

            break;
        }


        case ESP_BT_GAP_KEY_NOTIF_EVT:
        {
            snprintf(
                tars_auth_status,
                sizeof(tars_auth_status),
                "SSP PASSKEY %06u",
                (unsigned int)param->key_notif.passkey
            );

            tars_set_diag(
                tars_auth_status
            );

            break;
        }


        case ESP_BT_GAP_KEY_REQ_EVT:
        {
            snprintf(
                tars_auth_status,
                sizeof(tars_auth_status),
                "SSP PASSKEY REQUEST"
            );

            tars_set_diag(
                tars_auth_status
            );

            break;
        }


        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        {
            char bda_text[32];

            tars_bda_to_string(
                param->acl_conn_cmpl_stat.bda,
                bda_text,
                sizeof(bda_text)
            );

            char diag[128];

            snprintf(
                diag,
                sizeof(diag),
                "ACL CONNECT STATUS=%d %s",
                (int)param->acl_conn_cmpl_stat.stat,
                bda_text
            );

            tars_set_diag(
                diag
            );

            break;
        }


        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        {
            char bda_text[32];

            tars_bda_to_string(
                param->acl_disconn_cmpl_stat.bda,
                bda_text,
                sizeof(bda_text)
            );

            char diag[128];

            snprintf(
                diag,
                sizeof(diag),
                "ACL DISCONNECT REASON=%d %s",
                (int)param->acl_disconn_cmpl_stat.reason,
                bda_text
            );

            tars_set_diag(
                diag
            );

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
        char diag[96];

        snprintf(
            diag,
            sizeof(diag),
            "BT CONTROLLER INIT ERROR %d",
            (int)ret
        );

        tars_set_diag(
            diag
        );

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

    /*
       TARS dibuat discoverable dan connectable.
    */

    ret =
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        );

    if (
        ret != ESP_OK
    ) {
        return mp_obj_new_str(
            "ERROR: SET SCAN MODE FAILED",
            strlen(
                "ERROR: SET SCAN MODE FAILED"
            )
        );
    }

    /*
       Aktifkan SSP.
       Jika perangkat lama menggunakan PIN,
       PIN_REQ_EVT tetap akan ditangani.
    */

    ret =
        esp_bt_gap_set_security_param(
            ESP_BT_SP_IOCAP_MODE,
            &(esp_bt_io_cap_t){
                ESP_BT_IO_CAP_NONE
            },
            sizeof(esp_bt_io_cap_t)
        );

    if (
        ret != ESP_OK
    ) {
        tars_set_diag(
            "WARNING: SSP IOCAP CONFIG FAILED"
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

    tars_internal_tone =
        false;

    tars_scanning =
        false;

    tars_device_found =
        false;

    tars_a2dp_connected =
        false;

    tars_a2dp_connecting =
        false;

    tars_connect_requested =
        false;

    tars_connect_after_scan_stop =
        false;

    tars_audio_started =
        false;

    tars_media_check_pending =
        false;

    tars_media_start_requested =
        false;

    tars_media_start_pending =
        false;

    tars_media_stop_pending =
        false;

    tars_auth_success =
        false;

    tars_auth_failed =
        false;

    snprintf(
        tars_auth_status,
        sizeof(tars_auth_status),
        "AUTH READY"
    );

    tars_bt_started =
        true;

    tars_status_text =
        "TARS BLUETOOTH CLASSIC A2DP READY";

    tars_set_diag(
        "TARS BT READY AND DISCOVERABLE"
    );

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
        tars_a2dp_connected
    ) {
        return mp_obj_new_str(
            "ERROR: ALREADY CONNECTED",
            strlen(
                "ERROR: ALREADY CONNECTED"
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

    tars_connect_requested =
        false;

    tars_connect_after_scan_stop =
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
        char diag[96];

        snprintf(
            diag,
            sizeof(diag),
            "BT SCAN API ERROR %d",
            (int)ret
        );

        tars_set_diag(
            diag
        );

        return mp_obj_new_str(
            "ERROR: BLUETOOTH SCAN FAILED",
            strlen(
                "ERROR: BLUETOOTH SCAN FAILED"
            )
        );
    }

    tars_scanning =
        true;

    tars_status_text =
        "TARS SCANNING";

    tars_set_diag(
        "SCANNING FOR I7-TWS"
    );

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

    if (
        tars_a2dp_connecting
    ) {
        return mp_obj_new_str(
            "TARS ALREADY CONNECTING",
            strlen(
                "TARS ALREADY CONNECTING"
            )
        );
    }

    tars_auth_success =
        false;

    tars_auth_failed =
        false;

    tars_connect_requested =
        true;

    if (
        tars_scanning
    ) {
        tars_connect_after_scan_stop =
            true;

        tars_status_text =
            "WAITING SCAN STOP";

        tars_set_diag(
            "CONNECT WAITING FOR SCAN STOP"
        );

        esp_err_t ret =
            esp_bt_gap_cancel_discovery();

        if (
            ret != ESP_OK
        ) {
            char diag[96];

            snprintf(
                diag,
                sizeof(diag),
                "CANCEL SCAN ERROR %d",
                (int)ret
            );

            tars_set_diag(
                diag
            );

            tars_connect_after_scan_stop =
                false;

            tars_connect_requested =
                false;

            return mp_obj_new_str(
                "ERROR: STOP SCAN FAILED",
                strlen(
                    "ERROR: STOP SCAN FAILED"
                )
            );
        }

        return mp_obj_new_str(
            "TARS WAITING FOR SCAN TO STOP...",
            strlen(
                "TARS WAITING FOR SCAN TO STOP..."
            )
        );
    }

    esp_err_t ret =
        tars_start_a2dp_connection();

    if (
        ret != ESP_OK
    ) {
        return mp_obj_new_str(
            "ERROR: A2DP CONNECT API FAILED",
            strlen(
                "ERROR: A2DP CONNECT API FAILED"
            )
        );
    }

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
        !tars_a2dp_connected
    ) {
        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }

    if (
        tars_audio_started
    ) {
        return mp_obj_new_str(
            "A2DP AUDIO ALREADY STREAMING",
            strlen(
                "A2DP AUDIO ALREADY STREAMING"
            )
        );
    }

    if (
        pcm_used == 0
    ) {
        return mp_obj_new_str(
            "ERROR: PCM BUFFER EMPTY - WRITE AUDIO FIRST",
            strlen(
                "ERROR: PCM BUFFER EMPTY - WRITE AUDIO FIRST"
            )
        );
    }

    tars_internal_tone =
        false;

    esp_err_t ret =
        tars_request_audio_start();

    if (
        ret != ESP_OK
    ) {
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
        !tars_a2dp_connected
    ) {
        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }

    tars_internal_tone =
        true;

    tars_tone_phase =
        0;

    tars_tone_frequency =
        440;

    if (
        tars_audio_started
    ) {
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

    if (
        ret != ESP_OK
    ) {
        tars_internal_tone =
            false;

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
    tars_internal_tone =
        false;

    tars_tone_phase =
        0;

    if (
        tars_audio_started
    ) {
        tars_status_text =
            "A2DP AUDIO STREAMING";
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
    if (
        !tars_a2dp_connected
    ) {
        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }

    tars_internal_tone =
        false;

    if (
        !tars_audio_started
    ) {
        return mp_obj_new_str(
            "A2DP AUDIO ALREADY STOPPED",
            strlen(
                "A2DP AUDIO ALREADY STOPPED"
            )
        );
    }

    esp_err_t ret =
        tars_request_audio_stop();

    if (
        ret != ESP_OK
    ) {
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

    if (
        !tars_a2dp_connected
    ) {
        return mp_obj_new_int(0);
    }

    tars_internal_tone =
        false;

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
    char result[80];

    snprintf(
        result,
        sizeof(result),
        "PCM BUFFER: %u / %u BYTES",
        (unsigned int)pcm_used,
        (unsigned int)PCM_BUFFER_SIZE
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
    return mp_obj_new_str(
        tars_last_diag,
        strlen(
            tars_last_diag
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_diag_obj,
    tars_a2dp_diag
);


/* =========================================================
   AUTH STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_auth(void)
{
    return mp_obj_new_str(
        tars_auth_status,
        strlen(
            tars_auth_status
        )
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_auth_obj,
    tars_a2dp_auth
);


/* =========================================================
   TEST
   ========================================================= */

static mp_obj_t
tars_a2dp_test(void)
{
    if (
        tars_media_stop_pending
    ) {
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

    if (
        tars_media_start_pending
    ) {
        return mp_obj_new_str(
            "TARS A2DP START PENDING",
            strlen(
                "TARS A2DP START PENDING"
            )
        );
    }

    if (
        tars_media_check_pending
    ) {
        return mp_obj_new_str(
            "TARS A2DP CHECKING SOURCE",
            strlen(
                "TARS A2DP CHECKING SOURCE"
            )
        );
    }

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
        tars_connect_after_scan_stop
    ) {
        return mp_obj_new_str(
            "TARS WAITING SCAN STOP",
            strlen(
                "TARS WAITING SCAN STOP"
            )
        );
    }

    if (
        tars_scanning
    ) {
        return mp_obj_new_str(
            "TARS A2DP SCANNING",
            strlen(
                "TARS A2DP SCANNING"
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
    },

    {
        MP_ROM_QSTR(
            MP_QSTR_auth
        ),

        MP_ROM_PTR(
            &tars_a2dp_auth_obj
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
