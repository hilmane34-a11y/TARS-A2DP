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

#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include "esp_heap_caps.h"
#include "esp_partition.h"


/* =========================================================
   TARS V1 MANZ

   ESP32 + MICROPYTHON USER C MODULE
   BLUETOOTH CLASSIC A2DP SOURCE
   CLOUDFLARE TTS

   ALUR:

   WiFi + HTTPS
          ↓
   DOWNLOAD AUDIO BERTAHAP
          ↓
   LANGSUNG TULIS KE FLASH PARTITION "ttsdata"
          ↓
   WIFI OFF
          ↓
   BLUETOOTH ON
          ↓
   BACA AUDIO DARI FLASH
          ↓
   PUTAR KE I7-TWS

   AUDIO TIDAK DISIMPAN PENUH DI RAM.
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS V1 MANZ"
#define TARS_TARGET_NAME "I7-TWS"

#define TARS_SAMPLE_RATE 44100
#define TARS_TTS_SOURCE_RATE 24000

/* PENTING: JANGAN ADA ":" DI DEPAN HOSTNAME */
#define TARS_CLOUD_TTS_URL "https://tars-cloud-v3.hilmane34.workers.dev/tts"

#define TARS_TTS_MAX_TEXT_LENGTH 30

#define TARS_HTTP_BUFFER_SIZE 512
#define TARS_FLASH_READ_BUFFER_SIZE 512

#define TARS_TTS_PARTITION_NAME "ttsdata"


/* =========================================================
   MODE STATE
   ========================================================= */

static bool tars_bt_started = false;
static bool tars_bt_stopping = false;
static volatile bool tars_tts_loading = false;


/* =========================================================
   BLUETOOTH STATE
   ========================================================= */

static bool tars_scanning = false;
static bool tars_device_found = false;
static bool tars_a2dp_connected = false;
static bool tars_a2dp_connecting = false;
static bool tars_audio_started = false;


/* =========================================================
   MEDIA CONTROL STATE
   ========================================================= */

static bool tars_media_check_pending = false;
static bool tars_media_start_requested = false;
static bool tars_media_start_pending = false;
static bool tars_media_stop_pending = false;


/* =========================================================
   TARGET BLUETOOTH ADDRESS
   ========================================================= */

static esp_bd_addr_t tars_target_bda = {0};


/* =========================================================
   INTERNAL TONE
   ========================================================= */

static bool tars_tone_enabled = false;
static uint32_t tars_tone_phase = 0;
static uint32_t tars_tone_frequency = 440;


/* =========================================================
   TTS FLASH STATE
   ========================================================= */

static const esp_partition_t *tars_tts_partition = NULL;

static size_t tars_tts_flash_size = 0;
static volatile size_t tars_tts_read_pos = 0;

static volatile bool tars_tts_playing = false;
static bool tars_tts_ready = false;

static uint64_t tars_tts_resample_phase = 0;


/* BUFFER FLASH KECIL */

static uint8_t
tars_flash_read_buffer[TARS_FLASH_READ_BUFFER_SIZE];

static size_t tars_flash_buffer_start = 0;
static size_t tars_flash_buffer_length = 0;


/* ERROR */

static const char *tars_tts_error = "";


/* STATUS */

static const char *tars_status_text = "TARS READY";


/* =========================================================
   FIND TTS FLASH PARTITION
   ========================================================= */

static bool
tars_find_tts_partition(void)
{
    if (tars_tts_partition != NULL) {
        return true;
    }

    tars_tts_partition =
        esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_ANY,
            TARS_TTS_PARTITION_NAME
        );

    if (tars_tts_partition == NULL) {
        tars_tts_error =
            "TTS FLASH PARTITION NOT FOUND";

        return false;
    }

    return true;
}


/* =========================================================
   CLEAR PLAY STATE
   ========================================================= */

static void
tars_clear_tts_play_state(void)
{
    tars_tts_read_pos = 0;
    tars_tts_resample_phase = 0;

    tars_flash_buffer_start = 0;
    tars_flash_buffer_length = 0;

    tars_tts_playing = false;
}


/* =========================================================
   CLEAR FULL TTS STATE
   ========================================================= */

static void
tars_clear_tts_state(void)
{
    tars_tts_flash_size = 0;
    tars_tts_read_pos = 0;

    tars_tts_resample_phase = 0;

    tars_flash_buffer_start = 0;
    tars_flash_buffer_length = 0;

    tars_tts_ready = false;
    tars_tts_playing = false;
}


/* =========================================================
   ERASE TTS FLASH
   ========================================================= */

static bool
tars_erase_tts_flash(void)
{
    if (!tars_find_tts_partition()) {
        return false;
    }

    esp_err_t ret =
        esp_partition_erase_range(
            tars_tts_partition,
            0,
            tars_tts_partition->size
        );

    if (ret != ESP_OK) {
        tars_tts_error =
            "TTS FLASH ERASE FAILED";

        return false;
    }

    return true;
}


/* =========================================================
   LOAD FLASH BUFFER
   ========================================================= */

static bool
tars_load_flash_buffer(size_t position)
{
    if (tars_tts_partition == NULL) {
        return false;
    }

    if (position >= tars_tts_flash_size) {
        return false;
    }

    size_t aligned_position =
        position -
        (position % TARS_FLASH_READ_BUFFER_SIZE);

    size_t remaining =
        tars_tts_flash_size -
        aligned_position;

    size_t read_length = remaining;

    if (read_length > TARS_FLASH_READ_BUFFER_SIZE) {
        read_length =
            TARS_FLASH_READ_BUFFER_SIZE;
    }

    esp_err_t ret =
        esp_partition_read(
            tars_tts_partition,
            aligned_position,
            tars_flash_read_buffer,
            read_length
        );

    if (ret != ESP_OK) {
        return false;
    }

    tars_flash_buffer_start =
        aligned_position;

    tars_flash_buffer_length =
        read_length;

    return true;
}


/* =========================================================
   READ PCM SAMPLE
   PCM 16 BIT LITTLE ENDIAN MONO
   ========================================================= */

static bool
tars_read_pcm_sample(
    size_t byte_position,
    int16_t *sample
)
{
    if (sample == NULL) {
        return false;
    }

    if (
        byte_position + 1 >=
        tars_tts_flash_size
    ) {
        return false;
    }

    bool inside_buffer =
        (byte_position >= tars_flash_buffer_start)
        &&
        (
            byte_position + 1 <
            tars_flash_buffer_start +
            tars_flash_buffer_length
        );

    if (!inside_buffer) {

        if (
            !tars_load_flash_buffer(
                byte_position
            )
        ) {
            return false;
        }
    }

    size_t local_position =
        byte_position -
        tars_flash_buffer_start;

    uint16_t value =
        ((uint16_t)
        tars_flash_read_buffer[
            local_position
        ])
        |
        (
            ((uint16_t)
            tars_flash_read_buffer[
                local_position + 1
            ])
            << 8
        );

    *sample = (int16_t)value;

    return true;
}


/* =========================================================
   GENERATE INTERNAL TONE
   ========================================================= */

static int32_t
tars_generate_tone(
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
        (uint32_t)
        (
            (
                ((uint64_t)
                tars_tone_frequency *
                4294967296ULL)
                /
                TARS_SAMPLE_RATE
            )
        );

    while (position < usable_len) {

        int16_t sample;

        if (
            tars_tone_phase &
            0x80000000UL
        ) {
            sample = 3500;
        }
        else {
            sample = -3500;
        }

        data[position + 0] =
            (uint8_t)(sample & 0xFF);

        data[position + 1] =
            (uint8_t)
            (
                (sample >> 8) & 0xFF
            );

        data[position + 2] =
            (uint8_t)(sample & 0xFF);

        data[position + 3] =
            (uint8_t)
            (
                (sample >> 8) & 0xFF
            );

        position += 4;

        tars_tone_phase +=
            phase_increment;
    }

    return usable_len;
}


/* =========================================================
   HTTP RECEIVE CALLBACK

   DATA LANGSUNG DITULIS KE FLASH
   ========================================================= */

static esp_err_t
tars_tts_http_event(
    esp_http_client_event_t *evt
)
{
    if (evt == NULL) {
        return ESP_FAIL;
    }

    if (
        evt->event_id ==
        HTTP_EVENT_ON_DATA
    ) {
        if (
            evt->data == NULL ||
            evt->data_len <= 0
        ) {
            return ESP_OK;
        }

        if (tars_bt_started) {

            tars_tts_error =
                "BLUETOOTH ACTIVE DURING DOWNLOAD";

            return ESP_FAIL;
        }

        if (!tars_find_tts_partition()) {
            return ESP_FAIL;
        }

        size_t incoming =
            (size_t)
            evt->data_len;

        if (
            tars_tts_flash_size +
            incoming >
            tars_tts_partition->size
        ) {
            tars_tts_error =
                "TTS FLASH FULL";

            return ESP_FAIL;
        }

        esp_err_t ret =
            esp_partition_write(
                tars_tts_partition,
                tars_tts_flash_size,
                evt->data,
                incoming
            );

        if (ret != ESP_OK) {

            tars_tts_error =
                "TTS FLASH WRITE FAILED";

            return ESP_FAIL;
        }

        tars_tts_flash_size +=
            incoming;
    }

    return ESP_OK;
}


/* =========================================================
   JSON ESCAPE
   ========================================================= */

static char *
tars_json_escape(
    const char *text,
    size_t length,
    size_t *out_length
)
{
    size_t capacity =
        (length * 2) + 16;

    char *result =
        heap_caps_malloc(
            capacity,
            MALLOC_CAP_8BIT
        );

    if (result == NULL) {
        return NULL;
    }

    size_t pos = 0;

    result[pos++] = '{';
    result[pos++] = '"';
    result[pos++] = 't';
    result[pos++] = 'e';
    result[pos++] = 'x';
    result[pos++] = 't';
    result[pos++] = '"';
    result[pos++] = ':';
    result[pos++] = '"';

    for (
        size_t i = 0;
        i < length;
        i++
    ) {

        char c = text[i];

        if (
            c == '"' ||
            c == '\\'
        ) {

            result[pos++] = '\\';
            result[pos++] = c;
        }

        else if (c == '\n') {

            result[pos++] = '\\';
            result[pos++] = 'n';
        }

        else if (c == '\r') {

            result[pos++] = '\\';
            result[pos++] = 'r';
        }

        else if (c == '\t') {

            result[pos++] = '\\';
            result[pos++] = 't';
        }

        else if (
            (unsigned char)c < 32
        ) {

            continue;
        }

        else {

            result[pos++] = c;
        }
    }

    result[pos++] = '"';
    result[pos++] = '}';

    result[pos] = '\0';

    if (out_length != NULL) {
        *out_length = pos;
    }

    return result;
}


/* =========================================================
   A2DP AUDIO CALLBACK
   PRIORITAS:
   1. TTS FLASH
   2. TONE
   3. SILENCE
   ========================================================= */

static int32_t
tars_a2dp_data_callback(
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

    if (!tars_audio_started) {

        memset(
            data,
            0,
            usable_len
        );

        return usable_len;
    }


    /* TTS FLASH */

    if (
        tars_tts_playing &&
        tars_tts_ready &&
        tars_tts_flash_size >= 2
    ) {

        int32_t position = 0;

        const uint64_t
        phase_increment =
            (
                ((uint64_t)
                TARS_TTS_SOURCE_RATE
                << 32)
                /
                TARS_SAMPLE_RATE
            );

        while (
            position <
            usable_len
        ) {

            size_t source_sample =
                (size_t)
                (
                    tars_tts_resample_phase
                    >> 32
                );

            size_t source_offset =
                source_sample * 2;

            tars_tts_read_pos =
                source_offset;

            int16_t sample = 0;

            if (
                !tars_read_pcm_sample(
                    source_offset,
                    &sample
                )
            ) {

                tars_tts_playing =
                    false;

                tars_tts_read_pos =
                    tars_tts_flash_size;

                memset(
                    data + position,
                    0,
                    usable_len - position
                );

                tars_status_text =
                    "TTS FINISHED";

                break;
            }


            /* MONO -> STEREO */

            data[position + 0] =
                (uint8_t)
                (sample & 0xFF);

            data[position + 1] =
                (uint8_t)
                (
                    (sample >> 8)
                    & 0xFF
                );

            data[position + 2] =
                (uint8_t)
                (sample & 0xFF);

            data[position + 3] =
                (uint8_t)
                (
                    (sample >> 8)
                    & 0xFF
                );

            position += 4;

            tars_tts_resample_phase +=
                phase_increment;
        }

        return usable_len;
    }


    /* INTERNAL TONE */

    if (tars_tone_enabled) {

        return
            tars_generate_tone(
                data,
                usable_len
            );
    }


    /* SILENCE */

    memset(
        data,
        0,
        usable_len
    );

    return usable_len;
}


/* =========================================================
   REQUEST AUDIO START
   ========================================================= */

static esp_err_t
tars_request_audio_start(void)
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

    tars_media_start_requested =
        true;

    tars_media_check_pending =
        true;

    tars_status_text =
        "A2DP CHECKING SOURCE";

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
        );

    if (ret != ESP_OK) {

        tars_media_check_pending =
            false;

        tars_media_start_requested =
            false;

        tars_status_text =
            "A2DP SOURCE CHECK FAILED";
    }

    return ret;
}


/* =========================================================
   REQUEST AUDIO STOP
   ========================================================= */

static esp_err_t
tars_request_audio_stop(void)
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

    tars_media_stop_pending =
        true;

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_STOP
        );

    if (ret != ESP_OK) {

        tars_media_stop_pending =
            false;
    }

    return ret;
}


/* =========================================================
   A2DP EVENT CALLBACK
   ========================================================= */

static void
tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
)
{
    if (param == NULL) {
        return;
    }

    switch (event) {


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

            tars_tone_enabled =
                false;

            tars_tts_playing =
                false;

            tars_media_check_pending =
                false;

            tars_media_start_requested =
                false;

            tars_media_start_pending =
                false;

            tars_media_stop_pending =
                false;

            if (!tars_bt_stopping) {

                tars_status_text =
                    "A2DP DISCONNECTED";
            }

            break;


        case ESP_A2D_CONNECTION_STATE_CONNECTING:

            tars_a2dp_connected =
                false;

            tars_a2dp_connecting =
                true;

            tars_audio_started =
                false;

            tars_status_text =
                "A2DP CONNECTING";

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

            break;


        default:
            break;
        }

        break;


    case ESP_A2D_AUDIO_STATE_EVT:

        switch (
            param->audio_stat.state
        ) {

        case ESP_A2D_AUDIO_STATE_STARTED:

            tars_audio_started =
                true;

            tars_media_check_pending =
                false;

            tars_media_start_requested =
                false;

            tars_media_start_pending =
                false;

            tars_media_stop_pending =
                false;

            if (tars_tts_playing) {

                tars_status_text =
                    "TTS PLAYING FROM FLASH";
            }
            else {

                tars_status_text =
                    "A2DP AUDIO STREAMING";
            }

            break;


        case ESP_A2D_AUDIO_STATE_STOPPED:

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

            if (
                tars_a2dp_connected &&
                !tars_bt_stopping
            ) {

                tars_status_text =
                    "A2DP CONNECTED";
            }

            break;


        default:
            break;
        }

        break;


    case ESP_A2D_MEDIA_CTRL_ACK_EVT:

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

                    esp_a2d_media_ctrl(
                        ESP_A2D_MEDIA_CTRL_START
                    );
                }
            }

            else {

                tars_media_start_requested =
                    false;

                tars_status_text =
                    "A2DP SOURCE NOT READY";
            }
        }

        break;


    default:
        break;
    }
}


/* =========================================================
   BLUETOOTH GAP CALLBACK
   ========================================================= */

static void
tars_gap_callback(
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
            i <
            param->disc_res.num_prop;
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
                    strlen(
                        TARS_TARGET_NAME
                    )
                    ==
                    name_len
                    &&
                    memcmp(
                        name,
                        TARS_TARGET_NAME,
                        name_len
                    )
                    == 0
                ) {

                    memcpy(
                        tars_target_bda,
                        param->disc_res.bda,
                        ESP_BD_ADDR_LEN
                    );

                    tars_device_found =
                        true;

                    tars_status_text =
                        "I7-TWS FOUND";

                    esp_bt_gap_cancel_discovery();
                }
            }
        }

        break;
    }


    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:

        if (
            param->disc_st_chg.state ==
            ESP_BT_GAP_DISCOVERY_STOPPED
        ) {

            tars_scanning =
                false;
        }

        break;


    default:
        break;
    }
}


/* =========================================================
   RESET BLUETOOTH STATE
   ========================================================= */

static void
tars_reset_bluetooth_state(void)
{
    tars_scanning = false;

    tars_device_found = false;

    tars_a2dp_connected = false;

    tars_a2dp_connecting = false;

    tars_audio_started = false;

    tars_tone_enabled = false;

    tars_tone_phase = 0;

    tars_media_check_pending = false;

    tars_media_start_requested = false;

    tars_media_start_pending = false;

    tars_media_stop_pending = false;

    memset(
        tars_target_bda,
        0,
        ESP_BD_ADDR_LEN
    );
}


/* =========================================================
   START BLUETOOTH
   ========================================================= */

static mp_obj_t
tars_a2dp_start(void)
{
    esp_err_t ret;

    if (tars_tts_loading) {

        return mp_obj_new_str(
            "ERROR: TTS DOWNLOAD STILL RUNNING",
            strlen(
                "ERROR: TTS DOWNLOAD STILL RUNNING"
            )
        );
    }


    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            strlen(
                "ERROR: BLUETOOTH STOPPING"
            )
        );
    }


    if (tars_bt_started) {

        return mp_obj_new_str(
            "TARS BLUETOOTH ALREADY STARTED",
            strlen(
                "TARS BLUETOOTH ALREADY STARTED"
            )
        );
    }


    tars_reset_bluetooth_state();


    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret =
        esp_bt_controller_init(
            &bt_cfg
        );

    if (ret != ESP_OK) {

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

        esp_bt_controller_deinit();

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

        esp_bt_controller_disable();

        esp_bt_controller_deinit();

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

        esp_bluedroid_deinit();

        esp_bt_controller_disable();

        esp_bt_controller_deinit();

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

        esp_bluedroid_disable();

        esp_bluedroid_deinit();

        esp_bt_controller_disable();

        esp_bt_controller_deinit();

        return mp_obj_new_str(
            "ERROR: GAP CALLBACK FAILED",
            strlen(
                "ERROR: GAP CALLBACK FAILED"
            )
        );
    }


    esp_bt_gap_set_device_name(
        TARS_DEVICE_NAME
    );

    esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        ESP_BT_NON_CONNECTABLE
    );


    ret =
        esp_a2d_register_callback(
            tars_a2dp_event_callback
        );

    if (ret != ESP_OK) {

        esp_bluedroid_disable();

        esp_bluedroid_deinit();

        esp_bt_controller_disable();

        esp_bt_controller_deinit();

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

        esp_bluedroid_disable();

        esp_bluedroid_deinit();

        esp_bt_controller_disable();

        esp_bt_controller_deinit();

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

        esp_a2d_source_deinit();

        esp_bluedroid_disable();

        esp_bluedroid_deinit();

        esp_bt_controller_disable();

        esp_bt_controller_deinit();

        return mp_obj_new_str(
            "ERROR: AUDIO CALLBACK FAILED",
            strlen(
                "ERROR: AUDIO CALLBACK FAILED"
            )
        );
    }


    tars_bt_started = true;

    tars_bt_stopping = false;

    tars_status_text =
        "TARS BLUETOOTH READY";


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


    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            strlen(
                "ERROR: BLUETOOTH STOPPING"
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
            "TARS ALREADY SCANNING",
            strlen(
                "TARS ALREADY SCANNING"
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

        return mp_obj_new_str(
            "ERROR: BLUETOOTH SCAN FAILED",
            strlen(
                "ERROR: BLUETOOTH SCAN FAILED"
            )
        );
    }


    tars_scanning = true;

    tars_status_text =
        "SCANNING FOR I7-TWS";


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
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH NOT STARTED",
            strlen(
                "ERROR: BLUETOOTH NOT STARTED"
            )
        );
    }


    if (tars_device_found) {

        return mp_obj_new_str(
            "I7-TWS FOUND",
            strlen(
                "I7-TWS FOUND"
            )
        );
    }


    if (tars_scanning) {

        return mp_obj_new_str(
            "STILL SCANNING",
            strlen(
                "STILL SCANNING"
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


    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            strlen(
                "ERROR: BLUETOOTH STOPPING"
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


    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );


    if (ret != ESP_OK) {

        return mp_obj_new_str(
            "ERROR: A2DP CONNECT FAILED",
            strlen(
                "ERROR: A2DP CONNECT FAILED"
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
   TTS DOWNLOAD
   ========================================================= */

static mp_obj_t
tars_a2dp_tts_download(
    mp_obj_t text_obj
)
{
    if (tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: STOP BLUETOOTH BEFORE DOWNLOAD",
            strlen(
                "ERROR: STOP BLUETOOTH BEFORE DOWNLOAD"
            )
        );
    }


    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STILL STOPPING",
            strlen(
                "ERROR: BLUETOOTH STILL STOPPING"
            )
        );
    }


    if (tars_tts_loading) {

        return mp_obj_new_str(
            "ERROR: TTS ALREADY DOWNLOADING",
            strlen(
                "ERROR: TTS ALREADY DOWNLOADING"
            )
        );
    }


    size_t text_length = 0;

    const char *text =
        mp_obj_str_get_data(
            text_obj,
            &text_length
        );


    if (
        text == NULL ||
        text_length == 0
    ) {

        return mp_obj_new_str(
            "ERROR: TTS TEXT EMPTY",
            strlen(
                "ERROR: TTS TEXT EMPTY"
            )
        );
    }


    if (
        text_length >
        TARS_TTS_MAX_TEXT_LENGTH
    ) {

        return mp_obj_new_str(
            "ERROR: TTS TEXT TOO LONG MAX 30",
            strlen(
                "ERROR: TTS TEXT TOO LONG MAX 30"
            )
        );
    }


    if (!tars_find_tts_partition()) {

        return mp_obj_new_str(
            "ERROR: TTS FLASH PARTITION NOT FOUND",
            strlen(
                "ERROR: TTS FLASH PARTITION NOT FOUND"
            )
        );
    }


    /* RESET */

    tars_clear_tts_state();

    tars_tts_error = "";

    tars_tts_loading = true;

    tars_status_text =
        "TTS DOWNLOADING TO FLASH";


    /* HAPUS AUDIO LAMA */

    if (!tars_erase_tts_flash()) {

        tars_tts_loading = false;

        return mp_obj_new_str(
            "ERROR: TTS FLASH ERASE FAILED",
            strlen(
                "ERROR: TTS FLASH ERASE FAILED"
            )
        );
    }


    /* BUAT JSON */

    size_t json_length = 0;

    char *json =
        tars_json_escape(
            text,
            text_length,
            &json_length
        );


    if (json == NULL) {

        tars_tts_loading = false;

        return mp_obj_new_str(
            "ERROR: TTS JSON MEMORY FAILED",
            strlen(
                "ERROR: TTS JSON MEMORY FAILED"
            )
        );
    }


    /* HTTP CONFIG */

    esp_http_client_config_t config = {

    .host =
        TARS_CLOUD_TTS_HOST,

    .path =
        TARS_CLOUD_TTS_PATH,

    .port =
        TARS_CLOUD_TTS_PORT,

    .transport_type =
        HTTP_TRANSPORT_OVER_SSL,

    .method =
        HTTP_METHOD_POST,

    .event_handler =
        tars_tts_http_event,

    .timeout_ms =
        15000,

    .buffer_size =
        TARS_HTTP_BUFFER_SIZE,

    .buffer_size_tx =
        TARS_HTTP_BUFFER_SIZE,

    .crt_bundle_attach =
        esp_crt_bundle_attach
    };


    esp_http_client_handle_t client =
        esp_http_client_init(
            &config
        );


    if (client == NULL) {

        heap_caps_free(json);

        tars_tts_loading = false;

        return mp_obj_new_str(
            "ERROR: HTTP CLIENT INIT FAILED",
            strlen(
                "ERROR: HTTP CLIENT INIT FAILED"
            )
        );
    }


    esp_http_client_set_header(
        client,
        "Content-Type",
        "application/json"
    );


    /*
       FORMAT YANG DIHARAPKAN:

       PCM 16 BIT
       LITTLE ENDIAN
       MONO
       24000 Hz
    */

    esp_http_client_set_header(
        client,
        "Accept",
        "audio/L16"
    );


    esp_http_client_set_post_field(
        client,
        json,
        (int)json_length
    );


    /* DOWNLOAD */

    esp_err_t ret =
        esp_http_client_perform(
            client
        );


    int status_code =
        esp_http_client_get_status_code(
            client
        );


    esp_http_client_cleanup(
        client
    );


    heap_caps_free(
        json
    );


    tars_tts_loading =
        false;


    /* ERROR INTERNAL */

    if (
        tars_tts_error != NULL &&
        tars_tts_error[0] != '\0'
    ) {

        const char *error =
            tars_tts_error;

        tars_clear_tts_state();

        return mp_obj_new_str(
            error,
            strlen(error)
        );
    }


    /* HTTP ERROR */

    if (ret != ESP_OK) {

        tars_clear_tts_state();

        tars_status_text =
            "TTS HTTP FAILED";

        return mp_obj_new_str(
            "ERROR: TTS HTTP REQUEST FAILED",
            strlen(
                "ERROR: TTS HTTP REQUEST FAILED"
            )
        );
    }


    if (
        status_code < 200 ||
        status_code >= 300
    ) {

        tars_clear_tts_state();

        tars_status_text =
            "TTS HTTP ERROR";

        char result[64];

        snprintf(
            result,
            sizeof(result),
            "ERROR: TTS HTTP %d",
            status_code
        );

        return mp_obj_new_str(
            result,
            strlen(result)
        );
    }


    /* CEK AUDIO */

    if (
        tars_tts_flash_size <
        2
    ) {

        tars_clear_tts_state();

        tars_status_text =
            "TTS EMPTY";

        return mp_obj_new_str(
            "ERROR: TTS EMPTY AUDIO",
            strlen(
                "ERROR: TTS EMPTY AUDIO"
            )
        );
    }


    /* SIAP */

    tars_tts_ready =
        true;

    tars_status_text =
        "TTS SAVED TO FLASH";


    char result[80];

    snprintf(
        result,
        sizeof(result),
        "TTS SAVED: %u BYTES",
        (unsigned int)
        tars_tts_flash_size
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );
}


static MP_DEFINE_CONST_FUN_OBJ_1(
    tars_a2dp_tts_download_obj,
    tars_a2dp_tts_download
);


/* =========================================================
   TTS PLAY
   ========================================================= */

static mp_obj_t
tars_a2dp_tts_play(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            strlen(
                "ERROR: START BLUETOOTH FIRST"
            )
        );
    }


    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            strlen(
                "ERROR: BLUETOOTH STOPPING"
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


    if (!tars_tts_ready) {

        return mp_obj_new_str(
            "ERROR: NO TTS IN FLASH",
            strlen(
                "ERROR: NO TTS IN FLASH"
            )
        );
    }


    if (
        tars_tts_flash_size <
        2
    ) {

        return mp_obj_new_str(
            "ERROR: TTS FLASH EMPTY",
            strlen(
                "ERROR: TTS FLASH EMPTY"
            )
        );
    }


    tars_tone_enabled =
        false;


    tars_clear_tts_play_state();


    tars_tts_playing =
        true;


    tars_status_text =
        "TTS PLAYING FROM FLASH";


    if (!tars_audio_started) {

        esp_err_t ret =
            tars_request_audio_start();


        if (ret != ESP_OK) {

            tars_tts_playing =
                false;

            return mp_obj_new_str(
                "ERROR: A2DP AUDIO START FAILED",
                strlen(
                    "ERROR: A2DP AUDIO START FAILED"
                )
            );
        }
    }


    return mp_obj_new_str(
        "TTS PLAY REQUESTED",
        strlen(
            "TTS PLAY REQUESTED"
        )
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tts_play_obj,
    tars_a2dp_tts_play
);


/* =========================================================
   STOP AUDIO
   ========================================================= */

static mp_obj_t
tars_a2dp_stop(void)
{
    tars_tone_enabled =
        false;

    tars_tts_playing =
        false;

    tars_tone_phase =
        0;

    tars_tts_resample_phase =
        0;


    if (!tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }


    if (!tars_audio_started) {

        return mp_obj_new_str(
            "A2DP AUDIO ALREADY STOPPED",
            strlen(
                "A2DP AUDIO ALREADY STOPPED"
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


    tars_status_text =
        "A2DP STOP REQUESTED";


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
   BLUETOOTH FULL STOP
   ========================================================= */

static mp_obj_t
tars_a2dp_bluetooth_stop(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "BLUETOOTH ALREADY STOPPED",
            strlen(
                "BLUETOOTH ALREADY STOPPED"
            )
        );
    }


    if (tars_tts_loading) {

        return mp_obj_new_str(
            "ERROR: TTS DOWNLOAD ACTIVE",
            strlen(
                "ERROR: TTS DOWNLOAD ACTIVE"
            )
        );
    }


    tars_bt_stopping =
        true;

    tars_status_text =
        "BLUETOOTH STOPPING";


    tars_tone_enabled =
        false;

    tars_tts_playing =
        false;

    tars_tone_phase =
        0;

    tars_tts_resample_phase =
        0;


    if (tars_scanning) {

        esp_bt_gap_cancel_discovery();

        tars_scanning =
            false;
    }


    /*
       DEINIT A2DP

       TIDAK MENUNGGU EVENT
       DISCONNECT TERLALU LAMA
       AGAR BT BISA SEGERA
       DIGUNAKAN BERGANTIAN
       DENGAN WIFI.
    */

    if (
        esp_bluedroid_get_status() ==
        ESP_BLUEDROID_STATUS_ENABLED
    ) {

        esp_a2d_source_deinit();
    }


    if (
        esp_bluedroid_get_status() ==
        ESP_BLUEDROID_STATUS_ENABLED
    ) {

        esp_bluedroid_disable();
    }


    if (
        esp_bluedroid_get_status() ==
        ESP_BLUEDROID_STATUS_INITIALIZED
    ) {

        esp_bluedroid_deinit();
    }


    if (
        esp_bt_controller_get_status() ==
        ESP_BT_CONTROLLER_STATUS_ENABLED
    ) {

        esp_bt_controller_disable();
    }


    if (
        esp_bt_controller_get_status() ==
        ESP_BT_CONTROLLER_STATUS_INITED
    ) {

        esp_bt_controller_deinit();
    }


    tars_bt_started =
        false;

    tars_bt_stopping =
        false;


    tars_reset_bluetooth_state();


    tars_status_text =
        "BLUETOOTH STOPPED";


    return mp_obj_new_str(
        "BLUETOOTH FULLY STOPPED - WIFI MODE READY",
        strlen(
            "BLUETOOTH FULLY STOPPED - WIFI MODE READY"
        )
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_bluetooth_stop_obj,
    tars_a2dp_bluetooth_stop
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
   MEMORY
   ========================================================= */

static mp_obj_t
tars_a2dp_memory(void)
{
    size_t free_8bit =
        heap_caps_get_free_size(
            MALLOC_CAP_8BIT
        );

    size_t largest_8bit =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_8BIT
        );

    size_t partition_size =
        0;


    if (
        tars_find_tts_partition()
    ) {

        partition_size =
            tars_tts_partition->size;
    }


    char result[220];


    snprintf(
        result,
        sizeof(result),

        "HEAP: %u | "
        "LARGEST: %u | "
        "FLASH TTS: %u / %u | "
        "BT: %s",

        (unsigned int)
        free_8bit,

        (unsigned int)
        largest_8bit,

        (unsigned int)
        tars_tts_flash_size,

        (unsigned int)
        partition_size,

        tars_bt_started
        ?
        "ON"
        :
        "OFF"
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_memory_obj,
    tars_a2dp_memory
);


/* =========================================================
   TTS INFO
   ========================================================= */

static mp_obj_t
tars_a2dp_tts_info(void)
{
    char result[160];


    snprintf(
        result,
        sizeof(result),

        "TTS READY: %s | "
        "SIZE: %u BYTES | "
        "PLAYING: %s | "
        "POSITION: %u",

        tars_tts_ready
        ?
        "YES"
        :
        "NO",

        (unsigned int)
        tars_tts_flash_size,

        tars_tts_playing
        ?
        "YES"
        :
        "NO",

        (unsigned int)
        tars_tts_read_pos
    );


    return mp_obj_new_str(
        result,
        strlen(result)
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tts_info_obj,
    tars_a2dp_tts_info
);


/* =========================================================
   MODULE FUNCTIONS
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
            MP_QSTR_tts_download
        ),

        MP_ROM_PTR(
            &tars_a2dp_tts_download_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_tts_play
        ),

        MP_ROM_PTR(
            &tars_a2dp_tts_play_obj
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
            MP_QSTR_bluetooth_stop
        ),

        MP_ROM_PTR(
            &tars_a2dp_bluetooth_stop_obj
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
            MP_QSTR_memory
        ),

        MP_ROM_PTR(
            &tars_a2dp_memory_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_tts_info
        ),

        MP_ROM_PTR(
            &tars_a2dp_tts_info_obj
        )
    }
};


/* =========================================================
   MICROPYTHON DICTIONARY
   ========================================================= */

static MP_DEFINE_CONST_DICT(
    tars_a2dp_globals,
    tars_a2dp_globals_table
);


/* =========================================================
   MICROPYTHON MODULE
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
   MODULE REGISTER

   HARUS TETAP SATU BARIS
   ========================================================= */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
