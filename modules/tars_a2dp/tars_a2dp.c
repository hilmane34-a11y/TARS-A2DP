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

   REVISED AUDIO VERSION
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS"
#define TARS_TARGET_NAME "I7-TWS"


/*
   PCM ring buffer.

   Tetap 2048 byte agar tidak terlalu membebani RAM.
*/

#define PCM_BUFFER_SIZE 2048


/*
   Audio format.

   Stereo
   16-bit
   44100 Hz
*/

#define TARS_SAMPLE_RATE 44100
#define TARS_CHANNELS 2
#define TARS_BITS_PER_SAMPLE 16

#define TARS_BYTES_PER_SAMPLE 2
#define TARS_BYTES_PER_FRAME 4


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
   HELPER
   CREATE STRING OBJECT
   ========================================================= */

static mp_obj_t tars_return_string(
    const char *text
)
{
    return mp_obj_new_str(
        text,
        strlen(text)
    );
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
   PCM AVAILABLE
   ========================================================= */

static size_t tars_pcm_available(void)
{
    return pcm_used;
}


/* =========================================================
   PCM FREE SPACE
   ========================================================= */

static size_t tars_pcm_free(void)
{
    size_t used = pcm_used;

    if (
        used >= PCM_BUFFER_SIZE
    ) {
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

    if (
        data == NULL ||
        len == 0
    ) {
        return 0;
    }


    /*
       Jangan mengisi buffer melebihi kapasitas.

       Pembacaan pcm_used dilakukan terus karena
       callback A2DP dapat membaca buffer.
    */

    while (
        written < len
    ) {
        if (
            pcm_used >= PCM_BUFFER_SIZE
        ) {
            break;
        }


        pcm_buffer[
            pcm_write_pos
        ] =
            data[
                written
            ];


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
        read < len
    ) {
        if (
            pcm_used == 0
        ) {
            break;
        }


        data[
            read
        ] =
            pcm_buffer[
                pcm_read_pos
            ];


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
   FILL SILENCE

   PCM signed 16-bit silence = 0x0000

   memset aman karena seluruh byte bernilai nol.
   ========================================================= */

static void tars_fill_silence(
    uint8_t *data,
    int32_t len
)
{
    if (
        data == NULL ||
        len <= 0
    ) {
        return;
    }


    memset(
        data,
        0,
        (size_t)len
    );
}


/* =========================================================
   GENERATE INTERNAL TONE

   Square wave
   16-bit signed
   Stereo
   Little Endian

   Callback ini dibuat sesederhana mungkin karena
   dipanggil oleh sistem A2DP secara berulang.
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


    /*
       Satu stereo frame:

       Left  = 16 bit = 2 byte
       Right = 16 bit = 2 byte

       Total = 4 byte
    */

    int32_t usable_len =
        len -
        (
            len %
            TARS_BYTES_PER_FRAME
        );


    if (
        usable_len <= 0
    ) {
        return 0;
    }


    /*
       Hitung phase increment sekali per callback.

       32-bit phase accumulator.
    */

    uint32_t phase_increment =
        (uint32_t)(
            (
                (
                    (uint64_t)
                    tars_tone_frequency
                )
                *
                4294967296ULL
            )
            /
            TARS_SAMPLE_RATE
        );


    uint32_t phase =
        tars_tone_phase;


    int32_t position = 0;


    while (
        position < usable_len
    ) {
        int16_t sample;


        /*
           Square wave.

           Volume dibuat moderat.
        */

        if (
            phase &
            0x80000000UL
        ) {
            sample = 4000;
        }
        else
        {
            sample = -4000;
        }


        uint8_t low =
            (uint8_t)(
                sample &
                0xFF
            );


        uint8_t high =
            (uint8_t)(
                (
                    sample >>
                    8
                )
                &
                0xFF
            );


        /*
           LEFT
        */

        data[
            position
        ] =
            low;


        data[
            position + 1
        ] =
            high;


        /*
           RIGHT
        */

        data[
            position + 2
        ] =
            low;


        data[
            position + 3
        ] =
            high;


        position +=
            TARS_BYTES_PER_FRAME;


        phase +=
            phase_increment;
    }


    tars_tone_phase =
        phase;


    /*
       Jika ada sisa byte yang bukan kelipatan
       frame, isi silence.
    */

    if (
        usable_len < len
    ) {
        memset(
            data + usable_len,
            0,
            (size_t)(
                len -
                usable_len
            )
        );
    }


    return len;
}


/* =========================================================
   A2DP AUDIO DATA CALLBACK

   PENTING:

   Callback harus:

   - Cepat
   - Tidak melakukan print
   - Tidak melakukan alloc memory
   - Tidak melakukan API MicroPython
   - Tidak melakukan operasi Bluetooth control

   Callback hanya mengisi data audio.
   ========================================================= */

static int32_t tars_a2dp_data_callback(
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


    /*
       INTERNAL TONE
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
       PCM BUFFER
    */

    size_t received =
        tars_pcm_read(
            data,
            (size_t)len
        );


    /*
       Jika data PCM kurang,
       isi sisanya dengan silence.

       Sangat penting agar callback selalu
       memberikan buffer audio penuh.
    */

    if (
        received <
        (size_t)len
    ) {
        memset(
            data + received,
            0,
            (size_t)len -
            received
        );
    }


    /*
       Tetap return len karena seluruh buffer
       sudah diisi, termasuk silence.
    */

    return len;
}


/* =========================================================
   REQUEST AUDIO START
   ========================================================= */

static esp_err_t
tars_request_audio_start(void)
{
    if (
        !tars_bt_started
    ) {
        return ESP_FAIL;
    }


    if (
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
        tars_media_check_pending
    ) {
        return ESP_OK;
    }


    if (
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
    }


    return ret;
}


/* =========================================================
   REQUEST AUDIO STOP
   ========================================================= */

static esp_err_t
tars_request_audio_stop(void)
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


        tars_status_text =
            "A2DP STOP FAILED";
    }


    return ret;
}


/* =========================================================
   RESET MEDIA STATE
   ========================================================= */

static void
tars_reset_media_state(void)
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
        param == NULL
    ) {
        return;
    }


    switch (
        event
    )
    {


        /* =============================================
           CONNECTION STATE
           ============================================= */

        case ESP_A2D_CONNECTION_STATE_EVT:
        {
            switch (
                param->conn_stat.state
            )
            {


                case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
                {
                    tars_a2dp_connected =
                        false;


                    tars_a2dp_connecting =
                        false;


                    tars_reset_media_state();


                    /*
                       Matikan tone.

                       PCM juga dibersihkan karena
                       koneksi audio sudah hilang.
                    */

                    tars_internal_tone =
                        false;


                    tars_tone_phase =
                        0;


                    tars_pcm_clear();


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


                    tars_reset_media_state();


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


                    tars_status_text =
                        "A2DP DISCONNECTING";


                    break;
                }


                default:
                {
                    break;
                }
            }


            break;
        }


        /* =============================================
           AUDIO STATE
           ============================================= */

        case ESP_A2D_AUDIO_STATE_EVT:
        {
            switch (
                param->audio_stat.state
            )
            {


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
                        tars_internal_tone
                    ) {
                        tars_status_text =
                            "A2DP INTERNAL TONE STREAMING";
                    }
                    else
                    {
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
                    else
                    {
                        tars_status_text =
                            "A2DP AUDIO STOPPED";
                    }


                    break;
                }


                default:
                {
                    break;
                }
            }


            break;
        }


        /* =============================================
           MEDIA CONTROL ACK
           ============================================= */

        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        {


            /* -----------------------------------------
               CHECK SOURCE READY
               ----------------------------------------- */

            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
            )
            {
                tars_media_check_pending =
                    false;


                if (
                    param->media_ctrl_stat.status ==
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                )
                {
                    if (
                        tars_media_start_requested &&
                        !tars_audio_started &&
                        tars_a2dp_connected
                    )
                    {
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
                        )
                        {
                            tars_media_start_pending =
                                false;


                            tars_media_start_requested =
                                false;


                            tars_status_text =
                                "A2DP START FAILED";
                        }
                    }
                }
                else
                {
                    tars_media_start_requested =
                        false;


                    tars_media_start_pending =
                        false;


                    tars_status_text =
                        "A2DP SOURCE NOT READY";
                }


                break;
            }


            /* -----------------------------------------
               START ACK
               ----------------------------------------- */

            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_START
            )
            {
                if (
                    param->media_ctrl_stat.status !=
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                )
                {
                    tars_media_start_pending =
                        false;


                    tars_media_start_requested =
                        false;


                    tars_status_text =
                        "A2DP START ACK FAILED";
                }


                break;
            }


            /* -----------------------------------------
               STOP ACK
               ----------------------------------------- */

            if (
                param->media_ctrl_stat.cmd ==
                ESP_A2D_MEDIA_CTRL_STOP
            )
            {
                tars_media_stop_pending =
                    false;


                if (
                    param->media_ctrl_stat.status ==
                    ESP_A2D_MEDIA_CTRL_ACK_SUCCESS
                )
                {
                    tars_status_text =
                        "A2DP STOP REQUEST ACCEPTED";
                }
                else
                {
                    tars_status_text =
                        "A2DP STOP ACK FAILED";
                }


                break;
            }


            break;
        }


        default:
        {
            break;
        }
    }
}


/* =========================================================
   GAP CALLBACK

   BAGIAN SCAN DIPERTAHANKAN KARENA SUDAH TERBUKTI
   MENEMUKAN I7-TWS.
   ========================================================= */

static void
tars_gap_callback(
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
    )
    {


        /* =============================================
           DISCOVERY RESULT
           ============================================= */

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
            )
            {
                esp_bt_gap_dev_prop_t *prop =
                    &param->disc_res.prop[
                        i
                    ];


                if (
                    prop->type ==
                    ESP_BT_GAP_DEV_PROP_EIR
                )
                {
                    eir =
                        (uint8_t *)
                        prop->val;
                }
            }


            if (
                eir != NULL
            )
            {
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
                )
                {
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
                )
                {
                    if (
                        strlen(
                            TARS_TARGET_NAME
                        ) ==
                        name_len
                        &&
                        memcmp(
                            name,
                            TARS_TARGET_NAME,
                            name_len
                        ) == 0
                    )
                    {
                        memcpy(
                            tars_target_bda,
                            param->disc_res.bda,
                            ESP_BD_ADDR_LEN
                        );


                        tars_device_found =
                            true;


                        /*
                           Hentikan scan setelah
                           perangkat target ditemukan.
                        */

                        esp_bt_gap_cancel_discovery();
                    }
                }
            }


            break;
        }


        /* =============================================
           DISCOVERY STATE
           ============================================= */

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        {
            if (
                param->disc_st_chg.state ==
                ESP_BT_GAP_DISCOVERY_STOPPED
            )
            {
                tars_scanning =
                    false;
            }


            break;
        }


        default:
        {
            break;
        }
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
    )
    {
        return tars_return_string(
            "TARS BLUETOOTH ALREADY STARTED"
        );
    }


    /*
       Bluetooth controller configuration.
    */

    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret =
        esp_bt_controller_init(
            &bt_cfg
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: BT CONTROLLER INIT FAILED"
        );
    }


    ret =
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: BT CONTROLLER ENABLE FAILED"
        );
    }


    ret =
        esp_bluedroid_init();


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: BLUEDROID INIT FAILED"
        );
    }


    ret =
        esp_bluedroid_enable();


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: BLUEDROID ENABLE FAILED"
        );
    }


    /*
       Register GAP callback.
    */

    ret =
        esp_bt_gap_register_callback(
            tars_gap_callback
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: GAP CALLBACK FAILED"
        );
    }


    /*
       Set device name.
    */

    ret =
        esp_bt_gap_set_device_name(
            TARS_DEVICE_NAME
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: SET DEVICE NAME FAILED"
        );
    }


    /*
       ESP32 dibuat connectable.

       Tidak dibuat discoverable karena fokus
       sistem ini adalah A2DP Source.
    */

    ret =
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_NON_CONNECTABLE
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: SET SCAN MODE FAILED"
        );
    }


    /*
       Register A2DP callback.
    */

    ret =
        esp_a2d_register_callback(
            tars_a2dp_event_callback
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: A2DP CALLBACK FAILED"
        );
    }


    /*
       Initialize A2DP Source.
    */

    ret =
        esp_a2d_source_init();


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: A2DP SOURCE INIT FAILED"
        );
    }


    /*
       Register audio data callback.
    */

    ret =
        esp_a2d_source_register_data_callback(
            tars_a2dp_data_callback
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: AUDIO CALLBACK FAILED"
        );
    }


    /*
       Reset runtime state.
    */

    tars_pcm_clear();


    tars_internal_tone =
        false;


    tars_tone_phase =
        0;


    tars_scanning =
        false;


    tars_device_found =
        false;


    memset(
        tars_target_bda,
        0,
        ESP_BD_ADDR_LEN
    );


    tars_a2dp_connected =
        false;


    tars_a2dp_connecting =
        false;


    tars_reset_media_state();


    tars_bt_started =
        true;


    tars_status_text =
        "TARS BLUETOOTH CLASSIC A2DP READY";


    return tars_return_string(
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
tars_a2dp_scan(void)
{
    if (
        !tars_bt_started
    )
    {
        return tars_return_string(
            "ERROR: START BLUETOOTH FIRST"
        );
    }


    if (
        tars_a2dp_connected
    )
    {
        return tars_return_string(
            "ERROR: ALREADY CONNECTED"
        );
    }


    if (
        tars_scanning
    )
    {
        return tars_return_string(
            "TARS ALREADY SCANNING..."
        );
    }


    /*
       Reset hasil scan lama.
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
    )
    {
        return tars_return_string(
            "ERROR: BLUETOOTH SCAN FAILED"
        );
    }


    tars_scanning =
        true;


    tars_status_text =
        "TARS SCANNING";


    return tars_return_string(
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
tars_a2dp_found(void)
{
    if (
        !tars_device_found
    )
    {
        if (
            tars_scanning
        )
        {
            return tars_return_string(
                "STILL SCANNING..."
            );
        }


        return tars_return_string(
            "I7-TWS NOT FOUND"
        );
    }


    char result[
        64
    ];


    snprintf(
        result,
        sizeof(
            result
        ),
        "I7-TWS FOUND: %02X:%02X:%02X:%02X:%02X:%02X",
        tars_target_bda[
            0
        ],
        tars_target_bda[
            1
        ],
        tars_target_bda[
            2
        ],
        tars_target_bda[
            3
        ],
        tars_target_bda[
            4
        ],
        tars_target_bda[
            5
        ]
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
tars_a2dp_connect(void)
{
    if (
        !tars_bt_started
    )
    {
        return tars_return_string(
            "ERROR: START BLUETOOTH FIRST"
        );
    }


    if (
        !tars_device_found
    )
    {
        return tars_return_string(
            "ERROR: I7-TWS NOT FOUND"
        );
    }


    if (
        tars_a2dp_connected
    )
    {
        return tars_return_string(
            "TARS ALREADY CONNECTED"
        );
    }


    if (
        tars_a2dp_connecting
    )
    {
        return tars_return_string(
            "TARS ALREADY CONNECTING"
        );
    }


    /*
       Pastikan discovery dihentikan.

       Bagian ini dipertahankan dari kode
       yang sebelumnya berhasil connect.
    */

    if (
        tars_scanning
    )
    {
        esp_bt_gap_cancel_discovery();

        tars_scanning =
            false;
    }


    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: A2DP CONNECT FAILED"
        );
    }


    tars_a2dp_connecting =
        true;


    tars_status_text =
        "A2DP CONNECTING";


    return tars_return_string(
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
tars_a2dp_play(void)
{
    if (
        !tars_bt_started
    )
    {
        return tars_return_string(
            "ERROR: START BLUETOOTH FIRST"
        );
    }


    if (
        !tars_a2dp_connected
    )
    {
        return tars_return_string(
            "ERROR: A2DP NOT CONNECTED"
        );
    }


    if (
        tars_audio_started
    )
    {
        return tars_return_string(
            "A2DP AUDIO ALREADY STREAMING"
        );
    }


    if (
        tars_pcm_available() == 0
    )
    {
        return tars_return_string(
            "ERROR: PCM BUFFER EMPTY - WRITE AUDIO FIRST"
        );
    }


    /*
       PCM mode.
    */

    tars_internal_tone =
        false;


    esp_err_t ret =
        tars_request_audio_start();


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: A2DP SOURCE NOT READY"
        );
    }


    return tars_return_string(
        "TARS A2DP START REQUESTED - WAITING FOR ACK"
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
    )
    {
        return tars_return_string(
            "ERROR: START BLUETOOTH FIRST"
        );
    }


    if (
        !tars_a2dp_connected
    )
    {
        return tars_return_string(
            "ERROR: A2DP NOT CONNECTED"
        );
    }


    /*
       Aktifkan generator tone.
    */

    tars_internal_tone =
        true;


    tars_tone_phase =
        0;


    tars_tone_frequency =
        440;


    /*
       Jika audio stream sudah berjalan,
       callback langsung mulai menghasilkan tone.
    */

    if (
        tars_audio_started
    )
    {
        tars_status_text =
            "A2DP INTERNAL TONE STREAMING";


        return tars_return_string(
            "TARS INTERNAL 440HZ TONE ACTIVE"
        );
    }


    /*
       Jangan request START berkali-kali.
    */

    if (
        tars_media_check_pending
    )
    {
        return tars_return_string(
            "TARS TONE START ALREADY PENDING"
        );
    }


    if (
        tars_media_start_pending
    )
    {
        return tars_return_string(
            "TARS TONE START ALREADY PENDING"
        );
    }


    esp_err_t ret =
        tars_request_audio_start();


    if (
        ret != ESP_OK
    )
    {
        tars_internal_tone =
            false;


        return tars_return_string(
            "ERROR: TONE SOURCE NOT READY"
        );
    }


    return tars_return_string(
        "TARS INTERNAL TONE REQUESTED - WAITING FOR ACK"
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
    /*
       Hanya mematikan generator tone.

       Stream A2DP tidak otomatis dihentikan.
       Callback akan menghasilkan silence.
    */

    tars_internal_tone =
        false;


    tars_tone_phase =
        0;


    if (
        tars_audio_started
    )
    {
        tars_status_text =
            "A2DP AUDIO STREAMING";
    }


    return tars_return_string(
        "TARS INTERNAL TONE STOPPED"
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
    )
    {
        return tars_return_string(
            "ERROR: A2DP NOT CONNECTED"
        );
    }


    /*
       Matikan tone terlebih dahulu.
    */

    tars_internal_tone =
        false;


    tars_tone_phase =
        0;


    /*
       Jika audio belum berjalan,
       tidak perlu request STOP.
    */

    if (
        !tars_audio_started
    )
    {
        return tars_return_string(
            "A2DP AUDIO ALREADY STOPPED"
        );
    }


    esp_err_t ret =
        tars_request_audio_stop();


    if (
        ret != ESP_OK
    )
    {
        return tars_return_string(
            "ERROR: A2DP STOP FAILED"
        );
    }


    return tars_return_string(
        "TARS A2DP STOP REQUESTED"
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
    )
    {
        return mp_obj_new_int(
            0
        );
    }


    /*
       Data PCM yang ditulis berarti
       kita berpindah ke PCM mode.
    */

    tars_internal_tone =
        false;


    size_t written =
        tars_pcm_write(
            (
                const uint8_t *
            )
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
    char result[
        96
    ];


    size_t used =
        tars_pcm_available();


    size_t free_space =
        tars_pcm_free();


    snprintf(
        result,
        sizeof(
            result
        ),
        "PCM BUFFER: %u / %u BYTES, FREE: %u",
        (unsigned int)
        used,

        (unsigned int)
        PCM_BUFFER_SIZE,

        (unsigned int)
        free_space
    );


    return mp_obj_new_str(
        result,
        strlen(
            result
        )
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


    return tars_return_string(
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
   TEST
   ========================================================= */

static mp_obj_t
tars_a2dp_test(void)
{
    if (
        tars_media_stop_pending
    )
    {
        return tars_return_string(
            "TARS A2DP STOPPING"
        );
    }


    if (
        tars_internal_tone &&
        tars_audio_started
    )
    {
        return tars_return_string(
            "TARS INTERNAL TONE STREAMING"
        );
    }


    if (
        tars_media_start_pending
    )
    {
        return tars_return_string(
            "TARS A2DP START PENDING"
        );
    }


    if (
        tars_media_check_pending
    )
    {
        return tars_return_string(
            "TARS A2DP CHECKING SOURCE"
        );
    }


    if (
        tars_audio_started
    )
    {
        return tars_return_string(
            "TARS A2DP AUDIO STREAMING"
        );
    }


    if (
        tars_a2dp_connected
    )
    {
        return tars_return_string(
            "TARS A2DP CONNECTED"
        );
    }


    if (
        tars_a2dp_connecting
    )
    {
        return tars_return_string(
            "TARS A2DP CONNECTING"
        );
    }


    if (
        tars_scanning
    )
    {
        return tars_return_string(
            "TARS A2DP SCANNING"
        );
    }


    if (
        tars_bt_started
    )
    {
        return tars_return_string(
            "TARS A2DP STARTED"
        );
    }


    return tars_return_string(
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


/* =========================================================
   REGISTER MODULE

   HARUS SATU BARIS
   ========================================================= */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
