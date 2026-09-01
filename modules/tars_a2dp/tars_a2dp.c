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
   PCM TTS STREAMING REVISION

   ESP32
   Bluetooth Classic
   A2DP Source

   FORMAT A2DP OUTPUT:
   - 44100 Hz
   - Stereo
   - 16-bit signed
   - PCM little endian

   FORMAT TARS CLOUD TTS INPUT:
   - 24000 Hz
   - Mono
   - 16-bit signed
   - PCM_S16LE

   FITUR:
   - Scan I7-TWS
   - Connect
   - PCM ring buffer besar
   - Streaming saat audio aktif
   - write() untuk PCM 44100 stereo
   - write_tts() untuk PCM 24000 mono
   - Konversi 24k mono -> 44.1k stereo
   ========================================================= */


/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS V1 MANZ"
#define TARS_TARGET_NAME "I7-TWS"

/*
   32768 byte.

   Output 44.1kHz stereo 16-bit:
   44100 * 2 channel * 2 byte
   = 176400 byte/detik

   Buffer ini sekitar 185 ms audio.

   Tujuannya bukan menyimpan seluruh TTS,
   tetapi menjadi buffer streaming.
*/

#define PCM_BUFFER_SIZE 16384

#define TARS_SAMPLE_RATE 44100
#define TARS_CHANNELS 2
#define TARS_BITS_PER_SAMPLE 16

#define TTS_SAMPLE_RATE 24000


/* =========================================================
   BLUETOOTH STATE
   ========================================================= */

static bool tars_bt_started = false;
static bool tars_scanning = false;
static bool tars_device_found = false;
static bool tars_a2dp_connected = false;
static bool tars_a2dp_connecting = false;
static bool tars_audio_started = false;

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

static size_t pcm_read_pos = 0;
static size_t pcm_write_pos = 0;
static size_t pcm_used = 0;


/* =========================================================
   BUFFER LOCK

   Callback Bluetooth dan fungsi MicroPython
   dapat mengakses buffer secara terpisah.

   Lock ini melindungi:
   - read position
   - write position
   - used
   ========================================================= */

static portMUX_TYPE pcm_mux =
    portMUX_INITIALIZER_UNLOCKED;


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
   CLEAR PCM BUFFER
   ========================================================= */

static void tars_pcm_clear(void)
{
    portENTER_CRITICAL(&pcm_mux);

    pcm_read_pos = 0;
    pcm_write_pos = 0;
    pcm_used = 0;

    portEXIT_CRITICAL(&pcm_mux);
}


/* =========================================================
   GET PCM USED
   ========================================================= */

static size_t tars_pcm_used(void)
{
    size_t used;

    portENTER_CRITICAL(&pcm_mux);

    used = pcm_used;

    portEXIT_CRITICAL(&pcm_mux);

    return used;
}


/* =========================================================
   GET PCM FREE SPACE
   ========================================================= */

static size_t tars_pcm_free(void)
{
    size_t free_space;

    portENTER_CRITICAL(&pcm_mux);

    free_space =
        PCM_BUFFER_SIZE - pcm_used;

    portEXIT_CRITICAL(&pcm_mux);

    return free_space;
}


/* =========================================================
   WRITE PCM BUFFER

   Menulis sebanyak mungkin.

   Jika buffer penuh,
   fungsi mengembalikan jumlah byte
   yang benar-benar berhasil ditulis.
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

    portENTER_CRITICAL(&pcm_mux);

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

    portEXIT_CRITICAL(&pcm_mux);

    return written;
}


/* =========================================================
   READ PCM BUFFER

   Callback Bluetooth mengambil data.

   Jika data kurang,
   callback akan menambahkan silence.
   ========================================================= */

static size_t tars_pcm_read(
    uint8_t *data,
    size_t len
)
{
    size_t read_count = 0;

    if (
        data == NULL ||
        len == 0
    ) {
        return 0;
    }

    portENTER_CRITICAL(&pcm_mux);

    while (
        read_count < len &&
        pcm_used > 0
    ) {
        data[read_count] =
            pcm_buffer[pcm_read_pos];

        pcm_read_pos++;

        if (
            pcm_read_pos >= PCM_BUFFER_SIZE
        ) {
            pcm_read_pos = 0;
        }

        pcm_used--;
        read_count++;
    }

    portEXIT_CRITICAL(&pcm_mux);

    return read_count;
}


/* =========================================================
   WRITE ONE 16-BIT SAMPLE

   Helper internal.

   Menghasilkan:
   LEFT  16-bit
   RIGHT 16-bit

   Total 4 byte.
   ========================================================= */

static bool tars_pcm_write_stereo_sample(
    int16_t sample
)
{
    bool success = false;

    portENTER_CRITICAL(&pcm_mux);

    if (
        pcm_used + 4 <= PCM_BUFFER_SIZE
    ) {
        uint8_t lo =
            (uint8_t)(sample & 0xFF);

        uint8_t hi =
            (uint8_t)(
                (sample >> 8) & 0xFF
            );

        pcm_buffer[pcm_write_pos] = lo;

        pcm_write_pos =
            (pcm_write_pos + 1) %
            PCM_BUFFER_SIZE;

        pcm_buffer[pcm_write_pos] = hi;

        pcm_write_pos =
            (pcm_write_pos + 1) %
            PCM_BUFFER_SIZE;

        pcm_buffer[pcm_write_pos] = lo;

        pcm_write_pos =
            (pcm_write_pos + 1) %
            PCM_BUFFER_SIZE;

        pcm_buffer[pcm_write_pos] = hi;

        pcm_write_pos =
            (pcm_write_pos + 1) %
            PCM_BUFFER_SIZE;

        pcm_used += 4;

        success = true;
    }

    portEXIT_CRITICAL(&pcm_mux);

    return success;
}


/* =========================================================
   WRITE TTS PCM
   24000 HZ MONO -> 44100 HZ STEREO

   Input:
   PCM_S16LE
   24000 Hz
   Mono

   Output:
   44100 Hz
   Stereo
   PCM_S16LE

   Menggunakan nearest-neighbor resampling.

   Ini cukup ringan untuk ESP32 dan
   jauh lebih penting daripada
   langsung memutar PCM 24k mono
   sebagai 44.1k stereo.
   ========================================================= */

static size_t tars_pcm_write_tts(
    const uint8_t *data,
    size_t len
)
{
    if (
        data == NULL ||
        len < 2
    ) {
        return 0;
    }

    size_t input_samples = len / 2;

    /*
       Posisi input dalam satuan:

       1 / 44100 sample output

       Kita menggunakan integer agar ringan
       dan aman untuk ESP32.
    */

    uint32_t phase = 0;

    size_t output_count = 0;

    /*
       Maksimum output yang dapat dibuat
       berdasarkan ruang buffer.

       1 sample stereo = 4 byte.
    */

    size_t max_output_samples =
        tars_pcm_free() / 4;

    /*
       Rasio:

       Input  = 24000 Hz
       Output = 44100 Hz

       Untuk setiap sample output,
       phase maju 24000.

       Index input:
       phase / 44100
    */

    while (
        output_count < max_output_samples
    ) {
        size_t input_index =
            phase / TARS_SAMPLE_RATE;

        if (
            input_index >= input_samples
        ) {
            break;
        }

        size_t byte_index =
            input_index * 2;

        int16_t sample =
            (int16_t)(
                (uint16_t)data[
                    byte_index
                ]
                |
                (
                    (uint16_t)data[
                        byte_index + 1
                    ]
                    << 8
                )
            );

        if (
            !tars_pcm_write_stereo_sample(
                sample
            )
        ) {
            break;
        }

        /*
           Bergerak maju dalam domain
           input sample rate.
        */

        phase += TTS_SAMPLE_RATE;

        output_count++;
    }

    /*
       Hitung berapa byte input
       yang sudah benar-benar digunakan.

       Jika seluruh input selesai,
       kembalikan seluruh len genap.
    */

    size_t consumed_samples =
        phase / TARS_SAMPLE_RATE;

    if (
        consumed_samples >
        input_samples
    ) {
        consumed_samples =
            input_samples;
    }

    size_t consumed_bytes =
        consumed_samples * 2;

    /*
       Jangan pernah melebihi len.
    */

    if (
        consumed_bytes > len
    ) {
        consumed_bytes = len;
    }

    return consumed_bytes;
}


/* =========================================================
   INTERNAL TONE

   Square wave
   16-bit signed
   Stereo
   Little Endian
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
                (uint64_t)
                tars_tone_frequency *
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
            sample = 3000;
        }
        else {
            sample = -3000;
        }

        data[position + 0] =
            (uint8_t)(
                sample & 0xFF
            );

        data[position + 1] =
            (uint8_t)(
                (sample >> 8) &
                0xFF
            );

        data[position + 2] =
            (uint8_t)(
                sample & 0xFF
            );

        data[position + 3] =
            (uint8_t)(
                (sample >> 8) &
                0xFF
            );

        position += 4;

        tars_tone_phase +=
            phase_increment;
    }

    return usable_len;
}


/* =========================================================
   A2DP AUDIO DATA CALLBACK

   Callback harus cepat.

   Tidak boleh melakukan:
   - scan
   - connect
   - HTTP
   - MicroPython API
   - operasi berat

   Hanya:
   - tone
   - ambil PCM
   - silence jika kosong
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
       Jika audio belum aktif,
       jangan menghasilkan data audio.
    */

    if (
        !tars_audio_started
    ) {
        return 0;
    }

    /*
       INTERNAL TONE

       Tone tetap menghasilkan audio penuh.
       Bagian ini dipertahankan karena
       sebelumnya sudah terbukti stabil.
    */

    if (
        tars_internal_tone
    ) {
        return tars_generate_tone(
            data,
            len
        );
    }

    /*
       PCM STREAM

       Ambil hanya data PCM yang benar-benar
       tersedia di ring buffer.

       Jangan terus mengisi silence setelah
       seluruh PCM habis.
    */

    size_t received =
        tars_pcm_read(
            data,
            (size_t)len
        );

    /*
       Jika buffer kosong,
       beri silence untuk callback ini.

       Tetap return panjang yang diminta
       agar format callback A2DP tetap valid.
    */

    if (
        received == 0
    ) {
        memset(
            data,
            0,
            (size_t)len
        );

        return len;
    }

    /*
       Jika data kurang dari ukuran request,
       isi sisa dengan silence.
    */

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
        param == NULL
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
                            "A2DP PCM STREAMING";
                    }

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
                        ) == name_len
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

                        tars_device_found =
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
                tars_scanning =
                    false;
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
            ESP_BT_NON_CONNECTABLE
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

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_connect_obj,
    tars_a2dp_connect
);


/* =========================================================
   PLAY PCM STREAM
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

    /*
       Tetap minta minimal data
       sebelum memulai.

       Ini mencegah stream langsung
       dimulai dalam keadaan kosong.
    */

    if (
        tars_pcm_used() < 1024
    ) {
        return mp_obj_new_str(
            "ERROR: WRITE PCM FIRST",
            strlen(
                "ERROR: WRITE PCM FIRST"
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
        "TARS A2DP STREAM START REQUESTED",
        strlen(
            "TARS A2DP STREAM START REQUESTED"
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
        "TARS INTERNAL TONE REQUESTED",
        strlen(
            "TARS INTERNAL TONE REQUESTED"
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
        tars_audio_started &&
        tars_a2dp_connected
    ) {
        tars_request_audio_stop();
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
   WRITE NORMAL PCM

   FORMAT HARUS:
   44100 Hz
   Stereo
   PCM S16LE
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
   WRITE TTS PCM

   KHUSUS TARS CLOUD:

   INPUT:
   24000 Hz
   Mono
   PCM_S16LE

   Fungsi ini mengubahnya menjadi:

   OUTPUT:
   44100 Hz
   Stereo
   PCM_S16LE
   ========================================================= */

static mp_obj_t
tars_a2dp_write_tts(
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

    size_t consumed =
        tars_pcm_write_tts(
            (const uint8_t *)
            buffer_info.buf,
            buffer_info.len
        );

    return mp_obj_new_int_from_uint(
        consumed
    );
}

static MP_DEFINE_CONST_FUN_OBJ_1(
    tars_a2dp_write_tts_obj,
    tars_a2dp_write_tts
);


/* =========================================================
   BUFFER STATUS
   ========================================================= */

static mp_obj_t
tars_a2dp_buffer(void)
{
    char result[100];

    size_t used =
        tars_pcm_used();

    snprintf(
        result,
        sizeof(result),
        "PCM BUFFER: %u / %u BYTES",
        (unsigned int)used,
        (unsigned int)
        PCM_BUFFER_SIZE
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
   BUFFER FREE SPACE
   ========================================================= */

static mp_obj_t
tars_a2dp_space(void)
{
    return mp_obj_new_int_from_uint(
        tars_pcm_free()
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_space_obj,
    tars_a2dp_space
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
   STREAMING STATUS

   Return:
   True  = audio benar-benar berjalan
   False = belum berjalan
   ========================================================= */

static mp_obj_t
tars_a2dp_streaming(void)
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
            "TARS A2DP PCM STREAMING",
            strlen(
                "TARS A2DP PCM STREAMING"
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
            MP_QSTR_write_tts
        ),

        MP_ROM_PTR(
            &tars_a2dp_write_tts_obj
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
            MP_QSTR_space
        ),

        MP_ROM_PTR(
            &tars_a2dp_space_obj
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
            MP_QSTR_streaming
        ),

        MP_ROM_PTR(
            &tars_a2dp_streaming_obj
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
        (mp_obj_dict_t *)
        &tars_a2dp_globals
};


/* =========================================================
   REGISTER MODULE

   PENTING:
   HARUS TETAP SATU BARIS
   ========================================================= */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
