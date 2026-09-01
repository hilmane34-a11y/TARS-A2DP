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


/* =========================================================
   TARS A2DP SOURCE + SHORT CLOUDFLARE TTS

   ESP32
   Bluetooth Classic
   A2DP Source

   TTS MODE:
   SHORT SENTENCES
   MEMORY SAVING

   INPUT:
   {
       "text": "Halo"
   }

   OUTPUT:
   RAW PCM
   Linear16
   24000 Hz
   Mono

   A2DP OUTPUT:
   44100 Hz
   Stereo
   16-bit signed
   PCM little endian
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS V1 MANZ"

#define TARS_TARGET_NAME "I7-TWS"

#define TARS_SAMPLE_RATE 44100

#define TARS_TTS_SOURCE_RATE 24000

#define TARS_CLOUD_TTS_URL \
    "https://tars-cloud-v3.hilmane34.workers.dev/tts"


/*
   TARS hanya untuk kalimat pendek.

   80 karakter sudah cukup untuk:
   "Halo, saya TARS. Ada yang bisa saya bantu?"

   Jika server menghasilkan audio terlalu besar,
   request akan dihentikan agar ESP32 tidak kehabisan RAM.
*/

#define TARS_TTS_MAX_TEXT_LENGTH 80


/*
   Buffer audio maksimum.

   Dibuat lebih kecil agar aman saat:
   Bluetooth + WiFi aktif bersamaan.
*/

#define TARS_TTS_MAX_BYTES (16 * 1024)


/*
   HTTP buffer kecil untuk menghemat heap.
*/

#define TARS_HTTP_BUFFER_SIZE 1024


/* =========================================================
   BLUETOOTH STATE
   ========================================================= */

static bool
tars_bt_started = false;

static bool
tars_scanning = false;

static bool
tars_device_found = false;

static bool
tars_a2dp_connected = false;

static bool
tars_a2dp_connecting = false;

static bool
tars_audio_started = false;


/* =========================================================
   MEDIA CONTROL STATE
   ========================================================= */

static bool
tars_media_check_pending = false;

static bool
tars_media_start_requested = false;

static bool
tars_media_start_pending = false;

static bool
tars_media_stop_pending = false;


/* =========================================================
   TARGET BLUETOOTH ADDRESS
   ========================================================= */

static esp_bd_addr_t
tars_target_bda = {0};


/* =========================================================
   INTERNAL TONE
   ========================================================= */

static bool
tars_tone_enabled = false;

static uint32_t
tars_tone_phase = 0;

static uint32_t
tars_tone_frequency = 440;


/* =========================================================
   CLOUDFLARE TTS PCM
   ========================================================= */

static uint8_t *
tars_tts_pcm = NULL;

static size_t
tars_tts_pcm_size = 0;

static size_t
tars_tts_pcm_capacity = 0;

static volatile size_t
tars_tts_read_pos = 0;

static volatile bool
tars_tts_playing = false;

static volatile bool
tars_tts_loading = false;

static uint64_t
tars_tts_resample_phase = 0;

static const char *
tars_tts_error = "";


/* =========================================================
   STATUS
   ========================================================= */

static const char *
tars_status_text =
    "TARS A2DP READY";


/* =========================================================
   FREE TTS BUFFER
   ========================================================= */

static void
tars_free_tts_buffer(void)
{
    if (
        tars_tts_pcm != NULL
    ) {
        heap_caps_free(
            tars_tts_pcm
        );

        tars_tts_pcm =
            NULL;
    }


    tars_tts_pcm_size =
        0;

    tars_tts_pcm_capacity =
        0;

    tars_tts_read_pos =
        0;

    tars_tts_resample_phase =
        0;
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
        len -
        (
            len %
            4
        );


    int32_t position =
        0;


    uint32_t phase_increment =
        (uint32_t)
        (
            (
                (
                    uint64_t
                )
                tars_tone_frequency *
                4294967296ULL
            )
            /
            TARS_SAMPLE_RATE
        );


    while (
        position <
        usable_len
    ) {
        int16_t sample;


        if (
            tars_tone_phase &
            0x80000000UL
        ) {
            sample =
                3500;
        }

        else {
            sample =
                -3500;
        }


        data[
            position + 0
        ] =
            (
                uint8_t
            )
            (
                sample &
                0xFF
            );


        data[
            position + 1
        ] =
            (
                uint8_t
            )
            (
                (
                    sample >>
                    8
                )
                &
                0xFF
            );


        data[
            position + 2
        ] =
            (
                uint8_t
            )
            (
                sample &
                0xFF
            );


        data[
            position + 3
        ] =
            (
                uint8_t
            )
            (
                (
                    sample >>
                    8
                )
                &
                0xFF
            );


        position +=
            4;


        tars_tone_phase +=
            phase_increment;
    }


    return usable_len;
}


/* =========================================================
   HTTP RECEIVE CALLBACK

   Buffer audio dibatasi agar RAM ESP32 aman.
   ========================================================= */

static esp_err_t
tars_tts_http_event(
    esp_http_client_event_t *evt
)
{
    if (
        evt == NULL
    ) {
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


        size_t needed =
            tars_tts_pcm_size +
            (
                size_t
            )
            evt->data_len;


        if (
            needed >
            TARS_TTS_MAX_BYTES
        ) {
            tars_tts_error =
                "TTS AUDIO TOO LARGE";

            return ESP_FAIL;
        }


        if (
            needed >
            tars_tts_pcm_capacity
        ) {
            size_t new_capacity =
                tars_tts_pcm_capacity;


            if (
                new_capacity ==
                0
            ) {
                new_capacity =
                    4096;
            }


            while (
                new_capacity <
                needed
            ) {
                new_capacity *=
                    2;
            }


            if (
                new_capacity >
                TARS_TTS_MAX_BYTES
            ) {
                new_capacity =
                    TARS_TTS_MAX_BYTES;
            }


            uint8_t *new_buffer =
                heap_caps_realloc(
                    tars_tts_pcm,
                    new_capacity,
                    MALLOC_CAP_8BIT
                );


            if (
                new_buffer ==
                NULL
            ) {
                tars_tts_error =
                    "TTS OUT OF MEMORY";

                return ESP_FAIL;
            }


            tars_tts_pcm =
                new_buffer;


            tars_tts_pcm_capacity =
                new_capacity;
        }


        memcpy(
            tars_tts_pcm +
            tars_tts_pcm_size,

            evt->data,

            (
                size_t
            )
            evt->data_len
        );


        tars_tts_pcm_size +=
            (
                size_t
            )
            evt->data_len;
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
        (
            length *
            2
        )
        +
        32;


    char *result =
        heap_caps_malloc(
            capacity,
            MALLOC_CAP_8BIT
        );


    if (
        result ==
        NULL
    ) {
        return NULL;
    }


    size_t pos =
        0;


    result[pos++] =
        '{';

    result[pos++] =
        '"';

    result[pos++] =
        't';

    result[pos++] =
        'e';

    result[pos++] =
        'x';

    result[pos++] =
        't';

    result[pos++] =
        '"';

    result[pos++] =
        ':';

    result[pos++] =
        '"';


    for (
        size_t i = 0;

        i < length;

        i++
    ) {
        char c =
            text[i];


        if (
            c == '"' ||
            c == '\\'
        ) {
            result[pos++] =
                '\\';

            result[pos++] =
                c;
        }

        else if (
            c == '\n'
        ) {
            result[pos++] =
                '\\';

            result[pos++] =
                'n';
        }

        else if (
            c == '\r'
        ) {
            result[pos++] =
                '\\';

            result[pos++] =
                'r';
        }

        else if (
            c == '\t'
        ) {
            result[pos++] =
                '\\';

            result[pos++] =
                't';
        }

        else if (
            (
                unsigned char
            )
            c <
            32
        ) {
            continue;
        }

        else {
            result[pos++] =
                c;
        }
    }


    result[pos++] =
        '"';

    result[pos++] =
        '}';

    result[pos] =
        '\0';


    if (
        out_length !=
        NULL
    ) {
        *out_length =
            pos;
    }


    return result;
}


/* =========================================================
   A2DP AUDIO DATA CALLBACK

   PRIORITY:

   1. CLOUDFLARE TTS
   2. INTERNAL TONE
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
        len -
        (
            len %
            4
        );


    if (
        !tars_audio_started
    ) {
        memset(
            data,
            0,
            (
                size_t
            )
            usable_len
        );

        return usable_len;
    }


    if (
        tars_tts_playing &&
        tars_tts_pcm != NULL &&
        tars_tts_pcm_size >= 2
    ) {
        int32_t position =
            0;


        /*
           24000 Hz -> 44100 Hz

           phase menggunakan 32-bit fractional.
        */

        const uint64_t phase_increment =
            (
                (
                    (
                        uint64_t
                    )
                    TARS_TTS_SOURCE_RATE
                    <<
                    32
                )
                /
                TARS_SAMPLE_RATE
            );


        while (
            position <
            usable_len
        ) {
            size_t source_sample =
                (
                    size_t
                )
                (
                    tars_tts_resample_phase >>
                    32
                );


            size_t source_offset =
                source_sample *
                2;


            tars_tts_read_pos =
                source_offset;


            if (
                source_offset +
                1 >=
                tars_tts_pcm_size
            ) {
                tars_tts_playing =
                    false;


                tars_tts_read_pos =
                    tars_tts_pcm_size;


                memset(
                    data +
                    position,

                    0,

                    (
                        size_t
                    )
                    (
                        usable_len -
                        position
                    )
                );


                tars_status_text =
                    "TTS FINISHED";


                break;
            }


            int16_t sample =
                (
                    int16_t
                )
                (
                    (
                        (
                            uint16_t
                        )
                        tars_tts_pcm[
                            source_offset
                        ]
                    )
                    |
                    (
                        (
                            uint16_t
                        )
                        tars_tts_pcm[
                            source_offset +
                            1
                        ]
                        <<
                        8
                    )
                );


            data[
                position + 0
            ] =
                (
                    uint8_t
                )
                (
                    sample &
                    0xFF
                );


            data[
                position + 1
            ] =
                (
                    uint8_t
                )
                (
                    (
                        sample >>
                        8
                    )
                    &
                    0xFF
                );


            data[
                position + 2
            ] =
                (
                    uint8_t
                )
                (
                    sample &
                    0xFF
                );


            data[
                position + 3
            ] =
                (
                    uint8_t
                )
                (
                    (
                        sample >>
                        8
                    )
                    &
                    0xFF
                );


            position +=
                4;


            tars_tts_resample_phase +=
                phase_increment;
        }


        return usable_len;
    }


    if (
        tars_tone_enabled
    ) {
        return tars_generate_tone(
            data,
            usable_len
        );
    }


    memset(
        data,
        0,
        (
            size_t
        )
        usable_len
    );


    return usable_len;
}


/* =========================================================
   REQUEST AUDIO START
   ========================================================= */

static esp_err_t
tars_request_audio_start(
    void
)
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
        ret !=
        ESP_OK
    ) {
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
tars_request_audio_stop(
    void
)
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
        ret !=
        ESP_OK
    ) {
        tars_media_stop_pending =
            false;

        tars_status_text =
            "A2DP STOP FAILED";
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
    if (
        param ==
        NULL
    ) {
        return;
    }


    switch (
        event
    ) {
        case ESP_A2D_CONNECTION_STATE_EVT:
        {
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

                    tars_tts_playing =
                        false;


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

                    tars_media_check_pending =
                        false;

                    tars_media_start_requested =
                        false;

                    tars_media_start_pending =
                        false;

                    tars_media_stop_pending =
                        false;


                    if (
                        tars_tts_playing
                    ) {
                        tars_status_text =
                            "TTS STREAMING";
                    }

                    else if (
                        tars_tone_enabled
                    ) {
                        tars_status_text =
                            "A2DP TONE STREAMING";
                    }

                    else {
                        tars_status_text =
                            "A2DP AUDIO STREAMING";
                    }


                    break;
                }


                case ESP_A2D_AUDIO_STATE_STOPPED:
                {
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
                        tars_a2dp_connected
                    ) {
                        tars_status_text =
                            "A2DP CONNECTED";
                    }

                    else {
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
                            ret !=
                            ESP_OK
                        ) {
                            tars_media_start_pending =
                                false;

                            tars_media_start_requested =
                                false;

                            tars_status_text =
                                "A2DP START FAILED";
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
                }

                else {
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

static void
tars_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param
)
{
    if (
        param ==
        NULL
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
                        (
                            uint8_t *
                        )
                        prop->val;
                }
            }


            if (
                eir !=
                NULL
            ) {
                uint8_t name_len =
                    0;


                uint8_t *name =
                    esp_bt_gap_resolve_eir_data(
                        eir,
                        ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,
                        &name_len
                    );


                if (
                    name ==
                    NULL
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
                        )
                        ==
                        name_len

                        &&

                        memcmp(
                            name,
                            TARS_TARGET_NAME,
                            name_len
                        )
                        ==
                        0
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
        {
            if (
                param->disc_st_chg.state ==
                ESP_BT_GAP_DISCOVERY_STOPPED
            ) {
                tars_scanning =
                    false;


                if (
                    !tars_device_found
                ) {
                    tars_status_text =
                        "SCAN STOPPED - I7-TWS NOT FOUND";
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
tars_a2dp_start(
    void
)
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
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
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
            ESP_BT_NON_CONNECTABLE
        );


    if (
        ret !=
        ESP_OK
    ) {
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


    if (
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
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
        ret !=
        ESP_OK
    ) {
        return mp_obj_new_str(
            "ERROR: AUDIO CALLBACK FAILED",
            strlen(
                "ERROR: AUDIO CALLBACK FAILED"
            )
        );
    }


    tars_scanning =
        false;

    tars_device_found =
        false;

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

    tars_tone_enabled =
        false;

    tars_tts_playing =
        false;

    tars_tts_loading =
        false;

    tars_tone_phase =
        0;

    tars_tts_resample_phase =
        0;

    tars_tone_frequency =
        440;


    tars_free_tts_buffer();


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


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);


/* =========================================================
   SCAN
   ========================================================= */

static mp_obj_t
tars_a2dp_scan(
    void
)
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
        ret !=
        ESP_OK
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
tars_a2dp_found(
    void
)
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
        sizeof(
            result
        ),

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
        strlen(
            result
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
tars_a2dp_connect(
    void
)
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


    if (
        tars_scanning
    ) {
        esp_bt_gap_cancel_discovery();

        tars_scanning =
            false;
    }


    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );


    if (
        ret !=
        ESP_OK
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


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_connect_obj,
    tars_a2dp_connect
);


/* =========================================================
   PLAY TONE
   ========================================================= */

static mp_obj_t
tars_a2dp_play(
    void
)
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


    tars_tts_playing =
        false;


    tars_tone_enabled =
        true;


    tars_tone_phase =
        0;


    tars_tone_frequency =
        440;


    if (
        tars_audio_started
    ) {
        tars_status_text =
            "A2DP TONE STREAMING";


        return mp_obj_new_str(
            "TARS TONE ACTIVE",
            strlen(
                "TARS TONE ACTIVE"
            )
        );
    }


    esp_err_t ret =
        tars_request_audio_start();


    if (
        ret !=
        ESP_OK
    ) {
        tars_tone_enabled =
            false;


        return mp_obj_new_str(
            "ERROR: A2DP SOURCE NOT READY",
            strlen(
                "ERROR: A2DP SOURCE NOT READY"
            )
        );
    }


    return mp_obj_new_str(
        "TARS TONE START REQUESTED",
        strlen(
            "TARS TONE START REQUESTED"
        )
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_play_obj,
    tars_a2dp_play
);


/* =========================================================
   TONE ALIAS
   ========================================================= */

static mp_obj_t
tars_a2dp_tone(
    void
)
{
    return tars_a2dp_play();
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tone_obj,
    tars_a2dp_tone
);


/* =========================================================
   CLOUDFLARE SHORT TTS
   ========================================================= */

static mp_obj_t
tars_a2dp_tts(
    mp_obj_t text_obj
)
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
        tars_tts_loading
    ) {
        return mp_obj_new_str(
            "ERROR: TTS ALREADY LOADING",
            strlen(
                "ERROR: TTS ALREADY LOADING"
            )
        );
    }


    size_t text_length =
        0;


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


    /*
       Batasi teks agar TTS singkat.
    */

    if (
        text_length >
        TARS_TTS_MAX_TEXT_LENGTH
    ) {
        return mp_obj_new_str(
            "ERROR: TTS TEXT TOO LONG (MAX 80)",
            strlen(
                "ERROR: TTS TEXT TOO LONG (MAX 80)"
            )
        );
    }


    /*
       Hentikan audio sebelumnya.
    */

    tars_tone_enabled =
        false;

    tars_tts_playing =
        false;

    tars_tts_loading =
        true;


    /*
       Pastikan buffer lama dibersihkan
       sebelum download baru.
    */

    tars_free_tts_buffer();


    tars_tts_error =
        "";


    tars_status_text =
        "TTS DOWNLOADING";


    size_t json_length =
        0;


    char *json =
        tars_json_escape(
            text,
            text_length,
            &json_length
        );


    if (
        json ==
        NULL
    ) {
        tars_tts_loading =
            false;


        return mp_obj_new_str(
            "ERROR: TTS JSON MEMORY FAILED",
            strlen(
                "ERROR: TTS JSON MEMORY FAILED"
            )
        );
    }


    esp_http_client_config_t config =
{
    .url =
        TARS_CLOUD_TTS_URL,

    .method =
        HTTP_METHOD_POST,

    .event_handler =
        tars_tts_http_event,

    .timeout_ms =
        30000,

    .buffer_size =
        1024,

    .buffer_size_tx =
        1024,

    .crt_bundle_attach =
        esp_crt_bundle_attach
};


    esp_http_client_handle_t client =
        esp_http_client_init(
            &config
        );


    if (
        client ==
        NULL
    ) {
        heap_caps_free(
            json
        );


        tars_tts_loading =
            false;


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


    esp_http_client_set_header(
        client,
        "Accept",
        "audio/L16"
    );


    esp_http_client_set_post_field(
        client,
        json,
        (
            int
        )
        json_length
    );


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


    /*
       Jika callback menemukan error,
       buffer dibersihkan.
    */

    if (
        tars_tts_error[0] !=
        '\0'
    ) {
        const char *error_text =
            tars_tts_error;


        tars_free_tts_buffer();


        tars_status_text =
            error_text;


        return mp_obj_new_str(
            error_text,
            strlen(
                error_text
            )
        );
    }


    if (
        ret !=
        ESP_OK
    ) {
        tars_free_tts_buffer();


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
        status_code !=
        200
    ) {
        tars_free_tts_buffer();


        tars_status_text =
            "TTS SERVER ERROR";


        char result[80];


        snprintf(
            result,
            sizeof(
                result
            ),

            "ERROR: TTS HTTP %d",

            status_code
        );


        return mp_obj_new_str(
            result,
            strlen(
                result
            )
        );
    }


    if (
        tars_tts_pcm_size <
        2
    ) {
        tars_free_tts_buffer();


        tars_status_text =
            "TTS EMPTY AUDIO";


        return mp_obj_new_str(
            "ERROR: TTS EMPTY AUDIO",
            strlen(
                "ERROR: TTS EMPTY AUDIO"
            )
        );
    }


    tars_tts_read_pos =
        0;


    tars_tts_resample_phase =
        0;


    tars_tts_playing =
        true;


    tars_status_text =
        "TTS PCM READY";


    if (
        !tars_audio_started
    ) {
        esp_err_t start_ret =
            tars_request_audio_start();


        if (
            start_ret !=
            ESP_OK
        ) {
            tars_tts_playing =
                false;


            tars_free_tts_buffer();


            return mp_obj_new_str(
                "ERROR: A2DP AUDIO START FAILED",
                strlen(
                    "ERROR: A2DP AUDIO START FAILED"
                )
            );
        }
    }


    char result[96];


    snprintf(
        result,
        sizeof(
            result
        ),

        "TTS READY: %u BYTES",

        (
            unsigned int
        )
        tars_tts_pcm_size
    );


    return mp_obj_new_str(
        result,
        strlen(
            result
        )
    );
}


static MP_DEFINE_CONST_FUN_OBJ_1(
    tars_a2dp_tts_obj,
    tars_a2dp_tts
);


/* =========================================================
   STOP AUDIO
   ========================================================= */

static mp_obj_t
tars_a2dp_stop(
    void
)
{
    tars_tone_enabled =
        false;

    tars_tts_playing =
        false;

    tars_tone_phase =
        0;

    tars_tts_resample_phase =
        0;


    if (
        !tars_a2dp_connected
    ) {
        tars_free_tts_buffer();


        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            strlen(
                "ERROR: A2DP NOT CONNECTED"
            )
        );
    }


    if (
        !tars_audio_started
    ) {
        tars_free_tts_buffer();


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
        ret !=
        ESP_OK
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
   STOP TONE ALIAS
   ========================================================= */

static mp_obj_t
tars_a2dp_tone_stop(
    void
)
{
    return tars_a2dp_stop();
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tone_stop_obj,
    tars_a2dp_tone_stop
);


/* =========================================================
   STREAMING STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_streaming(
    void
)
{
    if (
        tars_audio_started
    ) {
        return mp_const_true;
    }


    return mp_const_false;
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_streaming_obj,
    tars_a2dp_streaming
);


/* =========================================================
   TTS PLAYING STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_tts_playing(
    void
)
{
    if (
        tars_tts_playing
    ) {
        return mp_const_true;
    }


    /*
       Setelah selesai,
       buffer tidak langsung dibebaskan dari
       Bluetooth callback untuk menghindari
       masalah sinkronisasi.

       Buffer akan dibersihkan sebelum TTS berikutnya
       atau saat stop().
    */

    return mp_const_false;
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tts_playing_obj,
    tars_a2dp_tts_playing
);


/* =========================================================
   STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_status(
    void
)
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
   TEST
   ========================================================= */

static mp_obj_t
tars_a2dp_test(
    void
)
{
    if (
        tars_tts_loading
    ) {
        return mp_obj_new_str(
            "TARS TTS DOWNLOADING",
            strlen(
                "TARS TTS DOWNLOADING"
            )
        );
    }


    if (
        tars_tts_playing
    ) {
        return mp_obj_new_str(
            "TARS TTS STREAMING",
            strlen(
                "TARS TTS STREAMING"
            )
        );
    }


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
        tars_tone_enabled &&
        tars_audio_started
    ) {
        return mp_obj_new_str(
            "TARS TONE STREAMING",
            strlen(
                "TARS TONE STREAMING"
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
   MEMORY STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_memory(
    void
)
{
    size_t free_8bit =
        heap_caps_get_free_size(
            MALLOC_CAP_8BIT
        );


    size_t largest_8bit =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_8BIT
        );


    size_t minimum_8bit =
        heap_caps_get_minimum_free_size(
            MALLOC_CAP_8BIT
        );


    char result[180];


    snprintf(
        result,

        sizeof(
            result
        ),

        "HEAP FREE: %u | LARGEST: %u | MIN: %u | TTS: %u",

        (
            unsigned int
        )
        free_8bit,

        (
            unsigned int
        )
        largest_8bit,

        (
            unsigned int
        )
        minimum_8bit,

        (
            unsigned int
        )
        tars_tts_pcm_size
    );


    return mp_obj_new_str(
        result,
        strlen(
            result
        )
    );
}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_memory_obj,
    tars_a2dp_memory
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
            MP_QSTR_tts
        ),

        MP_ROM_PTR(
            &tars_a2dp_tts_obj
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
            MP_QSTR_streaming
        ),

        MP_ROM_PTR(
            &tars_a2dp_streaming_obj
        )
    },


    {
        MP_ROM_QSTR(
            MP_QSTR_tts_playing
        ),

        MP_ROM_PTR(
            &tars_a2dp_tts_playing_obj
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
        (
            mp_obj_dict_t *
        )
        &tars_a2dp_globals
};


/* PENTING: HARUS SATU BARIS */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
