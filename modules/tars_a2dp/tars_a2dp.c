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
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"


/* =========================================================
   TARS A2DP SOURCE

   ESP32
   Bluetooth Classic
   A2DP Source

   Features:
   - Scan Bluetooth device
   - Find I7-TWS
   - Connect
   - Internal 440 Hz tone
   - PCM streaming from MicroPython
   - Ring buffer
   - Start / Stop audio

   Audio format:

   44100 Hz
   Stereo
   Signed 16-bit
   Little Endian

   1 frame = 4 bytes
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS"
#define TARS_TARGET_NAME "I7-TWS"


/*
   PCM buffer.

   Dibuat lebih kecil dari versi sebelumnya.

   Static buffer tidak memakai heap biasa,
   tetapi buffer lebih kecil tetap membantu
   penggunaan RAM keseluruhan firmware.
*/

#define PCM_BUFFER_SIZE 8192


#define TARS_SAMPLE_RATE     44100
#define TARS_CHANNELS        2
#define TARS_BITS_PER_SAMPLE 16

#define TARS_FRAME_BYTES     4


/* =========================================================
   BLUETOOTH STATE
   ========================================================= */

static bool tars_bt_started = false;

static bool tars_scanning = false;

static bool tars_device_found = false;

static bool tars_a2dp_connected = false;

static bool tars_a2dp_connecting = false;

static bool tars_audio_started = false;


/* =========================================================
   TARGET BLUETOOTH ADDRESS
   ========================================================= */

static esp_bd_addr_t tars_target_bda = {0};


/* =========================================================
   PCM RING BUFFER
   ========================================================= */


/*
   Buffer static.

   Tidak menggunakan malloc().
*/

static uint8_t pcm_buffer[PCM_BUFFER_SIZE];


static volatile size_t pcm_read_pos = 0;

static volatile size_t pcm_write_pos = 0;

static volatile size_t pcm_used = 0;


/*
   Lock.

   PCM buffer dapat diakses oleh:

   1. MicroPython task
   2. Bluetooth A2DP callback

   Jadi akses posisi buffer harus dilindungi.
*/

static portMUX_TYPE tars_pcm_mux =
    portMUX_INITIALIZER_UNLOCKED;


/* =========================================================
   INTERNAL TONE
   ========================================================= */

static bool tars_internal_tone = false;


/*
   32-bit phase oscillator.
*/

static uint32_t tars_tone_phase = 0;


/*
   Default 440 Hz.
*/

static uint32_t tars_tone_frequency = 440;


/*
   Jika 0:
   tone berjalan terus.
*/

static uint32_t tars_tone_samples_remaining = 0;


/*
   Counter sample.
*/

static uint32_t tars_tone_sample_counter = 0;


/* =========================================================
   STATUS TEXT
   ========================================================= */

static const char *tars_status_text =
    "TARS A2DP READY";


/* =========================================================
   HELPER
   ========================================================= */

static mp_obj_t
tars_make_string(
    const char *text
) {

    return mp_obj_new_str(
        text,
        strlen(text)
    );

}


/* =========================================================
   CLEAR PCM BUFFER
   ========================================================= */

static void
tars_pcm_clear_internal(
    void
) {

    portENTER_CRITICAL(
        &tars_pcm_mux
    );


    pcm_read_pos = 0;

    pcm_write_pos = 0;

    pcm_used = 0;


    portEXIT_CRITICAL(
        &tars_pcm_mux
    );

}


/* =========================================================
   WRITE PCM BUFFER
   ========================================================= */

static size_t
tars_pcm_write(
    const uint8_t *data,
    size_t len
) {

    size_t written = 0;


    if (
        data == NULL ||
        len == 0
    ) {

        return 0;

    }


    /*
       PCM stereo 16-bit harus
       kelipatan 4 byte.
    */

    len -=
        (
            len %
            TARS_FRAME_BYTES
        );


    while (
        written < len
    ) {

        bool full = false;


        portENTER_CRITICAL(
            &tars_pcm_mux
        );


        if (
            pcm_used >=
            PCM_BUFFER_SIZE
        ) {

            full = true;

        }
        else {

            pcm_buffer[
                pcm_write_pos
            ] =
                data[
                    written
                ];


            pcm_write_pos++;


            if (
                pcm_write_pos >=
                PCM_BUFFER_SIZE
            ) {

                pcm_write_pos = 0;

            }


            pcm_used++;

        }


        portEXIT_CRITICAL(
            &tars_pcm_mux
        );


        if (
            full
        ) {

            break;

        }


        written++;

    }


    /*
       Pastikan hasil kelipatan frame.
    */

    written -=
        (
            written %
            TARS_FRAME_BYTES
        );


    return written;

}


/* =========================================================
   READ PCM BUFFER
   ========================================================= */

static size_t
tars_pcm_read(
    uint8_t *data,
    size_t len
) {

    size_t read = 0;


    if (
        data == NULL ||
        len == 0
    ) {

        return 0;

    }


    while (
        read < len
    ) {

        bool empty = false;


        portENTER_CRITICAL(
            &tars_pcm_mux
        );


        if (
            pcm_used == 0
        ) {

            empty = true;

        }
        else {

            data[
                read
            ] =
                pcm_buffer[
                    pcm_read_pos
                ];


            pcm_read_pos++;


            if (
                pcm_read_pos >=
                PCM_BUFFER_SIZE
            ) {

                pcm_read_pos = 0;

            }


            pcm_used--;

        }


        portEXIT_CRITICAL(
            &tars_pcm_mux
        );


        if (
            empty
        ) {

            break;

        }


        read++;

    }


    return read;

}


/* =========================================================
   PCM BUFFER USED
   ========================================================= */

static size_t
tars_pcm_get_used(
    void
) {

    size_t used;


    portENTER_CRITICAL(
        &tars_pcm_mux
    );


    used =
        pcm_used;


    portEXIT_CRITICAL(
        &tars_pcm_mux
    );


    return used;

}


/* =========================================================
   GENERATE INTERNAL TONE
   ========================================================= */

static int32_t
tars_generate_tone(
    uint8_t *data,
    int32_t len
) {

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
            TARS_FRAME_BYTES
        );


    uint32_t phase_increment =
        (
            (
                uint64_t
            )
            tars_tone_frequency
            *
            4294967296ULL
        )
        /
        TARS_SAMPLE_RATE;


    int32_t position = 0;


    while (
        position <
        usable_len
    ) {

        /*
           Jika tone memiliki batas sample
           dan sudah selesai,
           kirim silence.
        */

        if (
            tars_tone_samples_remaining > 0 &&
            tars_tone_sample_counter >=
            tars_tone_samples_remaining
        ) {

            data[
                position + 0
            ] = 0;

            data[
                position + 1
            ] = 0;

            data[
                position + 2
            ] = 0;

            data[
                position + 3
            ] = 0;


            position +=
                TARS_FRAME_BYTES;


            continue;

        }


        int16_t sample;


        if (
            tars_tone_phase &
            0x80000000UL
        ) {

            sample = 10000;

        }
        else {

            sample = -10000;

        }


        /*
           Left channel.
        */

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
                    sample >> 8
                )
                &
                0xFF
            );


        /*
           Right channel.
        */

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
                    sample >> 8
                )
                &
                0xFF
            );


        position +=
            TARS_FRAME_BYTES;


        tars_tone_phase +=
            phase_increment;


        tars_tone_sample_counter++;

    }


    return usable_len;

}


/* =========================================================
   A2DP AUDIO DATA CALLBACK
   ========================================================= */

static int32_t
tars_a2dp_data_callback(
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
       Internal tone mode.
    */

    if (
        tars_internal_tone
    ) {

        return
            tars_generate_tone(
                data,
                len
            );

    }


    /*
       PCM mode.
    */

    size_t received =
        tars_pcm_read(
            data,
            (
                size_t
            )
            len
        );


    /*
       Jika PCM habis,
       isi sisa dengan silence.

       Jangan mengembalikan buffer
       setengah kosong.
    */

    if (
        received <
        (
            size_t
        )
        len
    ) {

        memset(
            data + received,
            0,
            (
                size_t
            )
            len -
            received
        );

    }


    /*
       A2DP membutuhkan panjang buffer
       penuh.
    */

    return len;

}


/* =========================================================
   A2DP EVENT CALLBACK
   ========================================================= */

static void
tars_a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
) {

    if (
        param == NULL
    ) {

        return;

    }


    switch (
        event
    ) {


        /* =============================================
           CONNECTION STATE
           ============================================= */

        case
        ESP_A2D_CONNECTION_STATE_EVT:


            switch (
                param
                ->
                conn_stat
                .
                state
            ) {


                case
                ESP_A2D_CONNECTION_STATE_DISCONNECTED:


                    tars_a2dp_connected =
                        false;


                    tars_a2dp_connecting =
                        false;


                    tars_audio_started =
                        false;


                    tars_internal_tone =
                        false;


                    tars_tone_phase =
                        0;


                    tars_tone_sample_counter =
                        0;


                    tars_pcm_clear_internal();


                    tars_status_text =
                        "A2DP DISCONNECTED";


                    break;


                case
                ESP_A2D_CONNECTION_STATE_CONNECTING:


                    tars_a2dp_connected =
                        false;


                    tars_a2dp_connecting =
                        true;


                    tars_audio_started =
                        false;


                    tars_status_text =
                        "A2DP CONNECTING";


                    break;


                case
                ESP_A2D_CONNECTION_STATE_CONNECTED:


                    tars_a2dp_connected =
                        true;


                    tars_a2dp_connecting =
                        false;


                    tars_audio_started =
                        false;


                    tars_status_text =
                        "A2DP CONNECTED";


                    break;


                case
                ESP_A2D_CONNECTION_STATE_DISCONNECTING:


                    tars_a2dp_connected =
                        false;


                    tars_a2dp_connecting =
                        false;


                    tars_audio_started =
                        false;


                    tars_status_text =
                        "A2DP DISCONNECTING";


                    break;


                default:

                    break;

            }


            break;


        /* =============================================
           AUDIO STATE
           ============================================= */

        case
        ESP_A2D_AUDIO_STATE_EVT:


            switch (
                param
                ->
                audio_stat
                .
                state
            ) {


                case
                ESP_A2D_AUDIO_STATE_STARTED:


                    tars_audio_started =
                        true;


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


                    break;


                case
                ESP_A2D_AUDIO_STATE_STOPPED:


                    tars_audio_started =
                        false;


                    if (
                        tars_a2dp_connected
                    ) {

                        tars_status_text =
                            "A2DP CONNECTED";

                    }
                    else {

                        tars_status_text =
                            "A2DP STOPPED";

                    }


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
   BLUETOOTH GAP CALLBACK
   ========================================================= */

static void
tars_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param
) {

    if (
        param == NULL
    ) {

        return;

    }


    switch (
        event
    ) {


        /* =============================================
           DISCOVERY RESULT
           ============================================= */

        case
        ESP_BT_GAP_DISC_RES_EVT:


        {

            uint8_t *eir =
                NULL;


            for (
                int i = 0;

                i <
                param
                ->
                disc_res
                .
                num_prop;

                i++
            ) {

                esp_bt_gap_dev_prop_t
                *prop =
                    &
                    param
                    ->
                    disc_res
                    .
                    prop[
                        i
                    ];


                if (
                    prop
                    ->
                    type
                    ==
                    ESP_BT_GAP_DEV_PROP_EIR
                ) {

                    eir =
                        (
                            uint8_t *
                        )
                        prop
                        ->
                        val;

                }

            }


            if (
                eir == NULL
            ) {

                break;

            }


            uint8_t name_len =
                0;


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
                name == NULL ||
                name_len == 0
            ) {

                break;

            }


            if (

                strlen(
                    TARS_TARGET_NAME
                )
                !=
                name_len

            ) {

                break;

            }


            if (

                memcmp(
                    name,
                    TARS_TARGET_NAME,
                    name_len
                )
                !=
                0

            ) {

                break;

            }


            /*
               Target ditemukan.
            */

            memcpy(
                tars_target_bda,
                param
                ->
                disc_res
                .
                bda,
                ESP_BD_ADDR_LEN
            );


            tars_device_found =
                true;


            tars_status_text =
                "I7-TWS FOUND";


            /*
               Hentikan scan agar
               tidak membuang resource.
            */

            if (
                tars_scanning
            ) {

                esp_bt_gap_cancel_discovery();

            }


            break;

        }


        /* =============================================
           DISCOVERY STATE
           ============================================= */

        case
        ESP_BT_GAP_DISC_STATE_CHANGED_EVT:


            if (

                param
                ->
                disc_st_chg
                .
                state

                ==

                ESP_BT_GAP_DISCOVERY_STOPPED

            ) {

                tars_scanning =
                    false;


                if (
                    tars_device_found
                ) {

                    tars_status_text =
                        "I7-TWS FOUND";

                }

            }


            break;


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
) {

    esp_err_t ret;


    if (
        tars_bt_started
    ) {

        return
            tars_make_string(
                "TARS BLUETOOTH ALREADY STARTED"
            );

    }


    /*
       Controller config.
    */

    esp_bt_controller_config_t
    bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret =
        esp_bt_controller_init(
            &bt_cfg
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: BT CONTROLLER INIT FAILED"
            );

    }


    ret =
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: BT CONTROLLER ENABLE FAILED"
            );

    }


    ret =
        esp_bluedroid_init();


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: BLUEDROID INIT FAILED"
            );

    }


    ret =
        esp_bluedroid_enable();


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: BLUEDROID ENABLE FAILED"
            );

    }


    ret =
        esp_bt_gap_register_callback(
            tars_gap_callback
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: GAP CALLBACK FAILED"
            );

    }


    ret =
        esp_bt_gap_set_device_name(
            TARS_DEVICE_NAME
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: SET DEVICE NAME FAILED"
            );

    }


    /*
       TARS dibuat discoverable
       dan connectable.
    */

    ret =
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: SET SCAN MODE FAILED"
            );

    }


    ret =
        esp_a2d_register_callback(
            tars_a2dp_event_callback
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: A2DP CALLBACK FAILED"
            );

    }


    ret =
        esp_a2d_source_init();


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: A2DP SOURCE INIT FAILED"
            );

    }


    ret =
        esp_a2d_source_register_data_callback(
            tars_a2dp_data_callback
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: AUDIO CALLBACK FAILED"
            );

    }


    /*
       Reset semua state.
    */

    tars_pcm_clear_internal();


    tars_internal_tone =
        false;


    tars_tone_phase =
        0;


    tars_tone_sample_counter =
        0;


    tars_tone_samples_remaining =
        0;


    tars_bt_started =
        true;


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


    tars_status_text =
        "TARS BLUETOOTH CLASSIC A2DP READY";


    return
        tars_make_string(
            "TARS BLUETOOTH CLASSIC A2DP READY"
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
) {

    if (
        !tars_bt_started
    ) {

        return
            tars_make_string(
                "ERROR: START BLUETOOTH FIRST"
            );

    }


    if (
        tars_scanning
    ) {

        return
            tars_make_string(
                "TARS ALREADY SCANNING..."
            );

    }


    /*
       Reset hasil scan sebelumnya.
    */

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

        return
            tars_make_string(
                "ERROR: BLUETOOTH SCAN FAILED"
            );

    }


    tars_scanning =
        true;


    tars_status_text =
        "SCANNING FOR I7-TWS";


    return
        tars_make_string(
            "TARS SCANNING FOR I7-TWS..."
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
) {

    if (
        !tars_device_found
    ) {

        if (
            tars_scanning
        ) {

            return
                tars_make_string(
                    "STILL SCANNING..."
                );

        }


        return
            tars_make_string(
                "I7-TWS NOT FOUND"
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


    return
        mp_obj_new_str(
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
tars_a2dp_connect(
    void
) {

    if (
        !tars_bt_started
    ) {

        return
            tars_make_string(
                "ERROR: START BLUETOOTH FIRST"
            );

    }


    if (
        !tars_device_found
    ) {

        return
            tars_make_string(
                "ERROR: I7-TWS NOT FOUND"
            );

    }


    if (
        tars_a2dp_connected
    ) {

        return
            tars_make_string(
                "TARS ALREADY CONNECTED"
            );

    }


    if (
        tars_a2dp_connecting
    ) {

        return
            tars_make_string(
                "TARS ALREADY CONNECTING"
            );

    }


    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: A2DP CONNECT FAILED"
            );

    }


    tars_a2dp_connecting =
        true;


    tars_status_text =
        "A2DP CONNECTING";


    return
        tars_make_string(
            "TARS CONNECTING TO I7-TWS..."
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
tars_a2dp_play(
    void
) {

    if (
        !tars_bt_started
    ) {

        return
            tars_make_string(
                "ERROR: START BLUETOOTH FIRST"
            );

    }


    if (
        !tars_a2dp_connected
    ) {

        return
            tars_make_string(
                "ERROR: A2DP NOT CONNECTED"
            );

    }


    if (
        tars_audio_started
    ) {

        return
            tars_make_string(
                "A2DP AUDIO ALREADY STREAMING"
            );

    }


    /*
       PCM harus tersedia.
    */

    if (
        !tars_internal_tone &&
        tars_pcm_get_used() == 0
    ) {

        return
            tars_make_string(
                "ERROR: PCM BUFFER EMPTY - WRITE AUDIO FIRST"
            );

    }


    esp_err_t ret;


    ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: A2DP SOURCE NOT READY"
            );

    }


    ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_START
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: A2DP AUDIO START FAILED"
            );

    }


    if (
        tars_internal_tone
    ) {

        tars_status_text =
            "A2DP INTERNAL TONE STARTING";

    }
    else {

        tars_status_text =
            "A2DP AUDIO STARTING";

    }


    return
        tars_make_string(
            "TARS A2DP AUDIO STARTING..."
        );

}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_play_obj,
    tars_a2dp_play
);


/* =========================================================
   STOP AUDIO
   ========================================================= */

static mp_obj_t
tars_a2dp_stop(
    void
) {

    /*
       Tone dimatikan.
    */

    tars_internal_tone =
        false;


    tars_tone_phase =
        0;


    tars_tone_sample_counter =
        0;


    if (
        !tars_audio_started
    ) {

        if (
            tars_a2dp_connected
        ) {

            tars_status_text =
                "A2DP CONNECTED";

        }


        return
            tars_make_string(
                "A2DP AUDIO ALREADY STOPPED"
            );

    }


    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_STOP
        );


    if (
        ret != ESP_OK
    ) {

        return
            tars_make_string(
                "ERROR: A2DP AUDIO STOP FAILED"
            );

    }


    tars_status_text =
        "A2DP AUDIO STOPPING";


    return
        tars_make_string(
            "TARS A2DP AUDIO STOPPING..."
        );

}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_stop_obj,
    tars_a2dp_stop
);


/* =========================================================
   INTERNAL TONE START
   ========================================================= */

static mp_obj_t
tars_a2dp_tone(
    void
) {

    if (
        !tars_bt_started
    ) {

        return
            tars_make_string(
                "ERROR: START BLUETOOTH FIRST"
            );

    }


    if (
        !tars_a2dp_connected
    ) {

        return
            tars_make_string(
                "ERROR: A2DP NOT CONNECTED"
            );

    }


    /*
       Aktifkan tone.
    */

    tars_internal_tone =
        true;


    tars_tone_phase =
        0;


    tars_tone_sample_counter =
        0;


    tars_tone_samples_remaining =
        0;


    tars_tone_frequency =
        440;


    /*
       Jika audio sudah berjalan,
       callback langsung menggunakan tone.
    */

    if (
        tars_audio_started
    ) {

        tars_status_text =
            "A2DP INTERNAL TONE STREAMING";


        return
            tars_make_string(
                "TARS INTERNAL 440HZ TONE ACTIVE"
            );

    }


    /*
       Cek source ready.
    */

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
        );


    if (
        ret != ESP_OK
    ) {

        tars_internal_tone =
            false;


        return
            tars_make_string(
                "ERROR: TONE SOURCE NOT READY"
            );

    }


    ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_START
        );


    if (
        ret != ESP_OK
    ) {

        tars_internal_tone =
            false;


        return
            tars_make_string(
                "ERROR: TONE START FAILED"
            );

    }


    tars_status_text =
        "A2DP INTERNAL TONE STARTING";


    return
        tars_make_string(
            "TARS INTERNAL 440HZ TONE STARTING..."
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
tars_a2dp_tone_stop(
    void
) {

    /*
       Penting:

       Jangan hanya mengubah flag.

       Kita juga meminta A2DP
       menghentikan media streaming.
    */

    tars_internal_tone =
        false;


    tars_tone_phase =
        0;


    tars_tone_sample_counter =
        0;


    if (
        tars_audio_started
    ) {

        esp_err_t ret =
            esp_a2d_media_ctrl(
                ESP_A2D_MEDIA_CTRL_STOP
            );


        if (
            ret != ESP_OK
        ) {

            return
                tars_make_string(
                    "ERROR: TONE STOP FAILED"
                );

        }


        tars_status_text =
            "A2DP TONE STOPPING";


        return
            tars_make_string(
                "TARS INTERNAL TONE STOPPING..."
            );

    }


    if (
        tars_a2dp_connected
    ) {

        tars_status_text =
            "A2DP CONNECTED";

    }


    return
        tars_make_string(
            "TARS INTERNAL TONE STOPPED"
        );

}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tone_stop_obj,
    tars_a2dp_tone_stop
);


/* =========================================================
   WRITE PCM
   ========================================================= */

static mp_obj_t
tars_a2dp_write(
    mp_obj_t data_in
) {

    mp_buffer_info_t
    buffer_info;


    mp_get_buffer_raise(
        data_in,
        &buffer_info,
        MP_BUFFER_READ
    );


    if (
        !tars_a2dp_connected
    ) {

        return
            mp_obj_new_int(
                0
            );

    }


    if (
        buffer_info.buf == NULL ||
        buffer_info.len == 0
    ) {

        return
            mp_obj_new_int(
                0
            );

    }


    /*
       Jika Python mengirim PCM,
       internal tone dimatikan.

       Tetapi stream tidak dihentikan.
       Callback akan langsung membaca
       PCM buffer.
    */

    tars_internal_tone =
        false;


    size_t len =
        buffer_info.len;


    /*
       PCM stereo frame.
    */

    len -=
        (
            len %
            TARS_FRAME_BYTES
        );


    size_t written =
        tars_pcm_write(
            (
                const uint8_t *
            )
            buffer_info.buf,
            len
        );


    return
        mp_obj_new_int_from_uint(
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
tars_a2dp_buffer(
    void
) {

    size_t used =
        tars_pcm_get_used();


    char result[80];


    snprintf(
        result,
        sizeof(result),
        "PCM BUFFER: %u / %u BYTES",
        (
            unsigned int
        )
        used,
        (
            unsigned int
        )
        PCM_BUFFER_SIZE
    );


    return
        mp_obj_new_str(
            result,
            strlen(result)
        );

}


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_buffer_obj,
    tars_a2dp_buffer
);


/* =========================================================
   CLEAR PCM BUFFER
   ========================================================= */

static mp_obj_t
tars_a2dp_clear(
    void
) {

    tars_pcm_clear_internal();


    return
        tars_make_string(
            "PCM BUFFER CLEARED"
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
tars_a2dp_status(
    void
) {

    return
        mp_obj_new_str(
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
) {

    if (
        tars_internal_tone
    ) {

        return
            tars_make_string(
                "TARS INTERNAL TONE ACTIVE"
            );

    }


    if (
        tars_audio_started
    ) {

        return
            tars_make_string(
                "TARS A2DP AUDIO STREAMING"
            );

    }


    if (
        tars_a2dp_connected
    ) {

        return
            tars_make_string(
                "TARS A2DP CONNECTED"
            );

    }


    if (
        tars_a2dp_connecting
    ) {

        return
            tars_make_string(
                "TARS A2DP CONNECTING"
            );

    }


    if (
        tars_scanning
    ) {

        return
            tars_make_string(
                "TARS A2DP SCANNING"
            );

    }


    if (
        tars_bt_started
    ) {

        return
            tars_make_string(
                "TARS A2DP STARTED"
            );

    }


    return
        tars_make_string(
            "TARS A2DP READY"
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
            MP_QSTR_stop
        ),

        MP_ROM_PTR(
            &tars_a2dp_stop_obj
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


/* HARUS SATU BARIS */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
