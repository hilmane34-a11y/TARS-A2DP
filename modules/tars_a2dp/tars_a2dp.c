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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_audio_dec_default.h"
#include "esp_audio_simple_dec_default.h"
#include "esp_audio_simple_dec.h"
/* =========================================================
   TARS V1 MANZ
   ESP32 + MICROPYTHON
   BLUETOOTH CLASSIC A2DP SOURCE
   CLOUDFLARE MELOTTS MP3
   ========================================================= */

/* =========================================================
   SETTINGS
   ========================================================= */

#define TARS_DEVICE_NAME "TARS V1 MANZ"
#define TARS_TARGET_NAME "I7-TWS"

#define TARS_SAMPLE_RATE 44100

#define TARS_CLOUD_TTS_HOST "tars-cloud-v1.hilmane34.workers.dev"
#define TARS_CLOUD_TTS_PATH "/tts"
#define TARS_CLOUD_TTS_PORT 443

#define TARS_TTS_MAX_TEXT_LENGTH 300

/* RAM kecil */
#define TARS_HTTP_BUFFER_SIZE 512
#define TARS_MP3_INPUT_BUFFER_SIZE 512
#define TARS_MP3_OUTPUT_BUFFER_SIZE 4096

/* PCM ring buffer 8 KB */
#define TARS_PCM_RING_SIZE 8192

#define TARS_FLASH_READ_BUFFER_SIZE 512

#define TARS_TTS_PARTITION_NAME "ttsdata"

#define TARS_TTS_HEADER_DEBUG_SIZE 16

/* 2.8x volume */
#define TARS_TTS_VOLUME_NUM 28
#define TARS_TTS_VOLUME_DEN 10

/* Decoder task */
#define TARS_MP3_TASK_STACK_SIZE 6144
#define TARS_MP3_TASK_PRIORITY 5

/* =========================================================
   BLUETOOTH MEMORY PROTECTION
   ========================================================= */

#define TARS_BT_MIN_HEAP    (48 * 1024)
#define TARS_BT_MIN_LARGEST (16 * 1024)

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
   MEDIA CONTROL
   ========================================================= */

static bool tars_media_check_pending = false;
static bool tars_media_start_requested = false;
static bool tars_media_start_pending = false;
static bool tars_media_stop_pending = false;

/* =========================================================
   TARGET ADDRESS
   ========================================================= */

static esp_bd_addr_t tars_target_bda = {0};

/* =========================================================
   INTERNAL TONE
   ========================================================= */

static bool tars_tone_enabled = false;
static uint32_t tars_tone_phase = 0;
static uint32_t tars_tone_frequency = 440;

/* =========================================================
   TTS FLASH
   ========================================================= */

static const esp_partition_t *tars_tts_partition = NULL;

static size_t tars_tts_flash_size = 0;
static volatile size_t tars_tts_read_pos = 0;

static volatile bool tars_tts_playing = false;
static bool tars_tts_ready = false;

/* =========================================================
   TTS DEBUG
   ========================================================= */

static int16_t tars_tts_debug_first_sample = 0;
static int16_t tars_tts_debug_min_sample = 0;
static int16_t tars_tts_debug_max_sample = 0;

static size_t tars_tts_debug_sample_count = 0;
static bool tars_tts_debug_valid = false;

/* =========================================================
   HEADER DEBUG
   ========================================================= */

static uint8_t tars_tts_header_debug[TARS_TTS_HEADER_DEBUG_SIZE];
static size_t tars_tts_header_debug_size = 0;

/* =========================================================
   MP3 DECODER
   ========================================================= */

static esp_audio_simple_dec_handle_t tars_mp3_decoder = NULL;

static TaskHandle_t tars_mp3_task_handle = NULL;

static volatile bool tars_mp3_decoder_running = false;
static volatile bool tars_mp3_decoder_eof = false;
static volatile bool tars_mp3_decoder_error = false;

static uint32_t tars_mp3_sample_rate = 44100;
static uint8_t tars_mp3_channels = 1;
static uint8_t tars_mp3_bits = 16;

/* =========================================================
   SMALL MP3 BUFFERS
   ========================================================= */

static uint8_t tars_mp3_input[TARS_MP3_INPUT_BUFFER_SIZE];

static uint8_t tars_mp3_output[TARS_MP3_OUTPUT_BUFFER_SIZE];

/* =========================================================
   PCM RING BUFFER
   FORMAT:
   16 BIT
   44100 Hz
   STEREO
   ========================================================= */

static uint8_t tars_pcm_ring[TARS_PCM_RING_SIZE];

static volatile size_t tars_pcm_ring_read = 0;
static volatile size_t tars_pcm_ring_write = 0;

/* =========================================================
   MP3 SOURCE BUFFER
   ========================================================= */

static uint8_t tars_flash_read_buffer[TARS_FLASH_READ_BUFFER_SIZE];

static size_t tars_flash_buffer_start = 0;
static size_t tars_flash_buffer_length = 0;

/* =========================================================
   RESAMPLER STATE
   ========================================================= */

static int32_t tars_resample_previous = 0;
static int32_t tars_resample_current = 0;
static uint64_t tars_resample_phase = 0;

/* =========================================================
   ERROR
   ========================================================= */

static const char *tars_tts_error = "";

/* =========================================================
   STATUS
   ========================================================= */

static const char *tars_status_text = "TARS READY";

/* =========================================================
   FIND TTS PARTITION
   ========================================================= */

static bool tars_find_tts_partition(void)
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
   RESET DEBUG
   ========================================================= */

static void tars_reset_tts_debug(void)
{
    tars_tts_debug_first_sample = 0;
    tars_tts_debug_min_sample = 0;
    tars_tts_debug_max_sample = 0;
    tars_tts_debug_sample_count = 0;
    tars_tts_debug_valid = false;

    memset(
        tars_tts_header_debug,
        0,
        sizeof(tars_tts_header_debug)
    );

    tars_tts_header_debug_size = 0;
}

/* =========================================================
   RESET PCM RING
   ========================================================= */

static void tars_pcm_ring_reset(void)
{
    tars_pcm_ring_read = 0;
    tars_pcm_ring_write = 0;
}

/* =========================================================
   RING USED
   ========================================================= */

static size_t tars_pcm_ring_used(void)
{
    size_t r = tars_pcm_ring_read;
    size_t w = tars_pcm_ring_write;

    if (w >= r) {
        return w - r;
    }

    return TARS_PCM_RING_SIZE - r + w;
}

/* =========================================================
   RING FREE
   ========================================================= */

static size_t tars_pcm_ring_free(void)
{
    return (TARS_PCM_RING_SIZE - 1) -
           tars_pcm_ring_used();
}

/* =========================================================
   RING WRITE
   ========================================================= */

static size_t tars_pcm_ring_write_data(
    const uint8_t *data,
    size_t len
)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    size_t free_space = tars_pcm_ring_free();

    if (len > free_space) {
        len = free_space;
    }

    if (len == 0) {
        return 0;
    }

    size_t w = tars_pcm_ring_write;

    size_t first =
        TARS_PCM_RING_SIZE - w;

    if (first > len) {
        first = len;
    }

    memcpy(
        tars_pcm_ring + w,
        data,
        first
    );

    if (len > first) {
        memcpy(
            tars_pcm_ring,
            data + first,
            len - first
        );
    }

    w += len;

    if (w >= TARS_PCM_RING_SIZE) {
        w -= TARS_PCM_RING_SIZE;
    }

    tars_pcm_ring_write = w;

    return len;
}

/* =========================================================
   RING READ
   ========================================================= */

static size_t tars_pcm_ring_read_data(
    uint8_t *data,
    size_t len
)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    size_t used = tars_pcm_ring_used();

    if (len > used) {
        len = used;
    }

    if (len == 0) {
        return 0;
    }

    size_t r = tars_pcm_ring_read;

    size_t first =
        TARS_PCM_RING_SIZE - r;

    if (first > len) {
        first = len;
    }

    memcpy(
        data,
        tars_pcm_ring + r,
        first
    );

    if (len > first) {
        memcpy(
            data + first,
            tars_pcm_ring,
            len - first
        );
    }

    r += len;

    if (r >= TARS_PCM_RING_SIZE) {
        r -= TARS_PCM_RING_SIZE;
    }

    tars_pcm_ring_read = r;

    return len;
}

/* =========================================================
   CLEAR PLAY STATE
   ========================================================= */

static void tars_clear_tts_play_state(void)
{
    tars_tts_read_pos = 0;

    tars_pcm_ring_reset();

    tars_resample_previous = 0;
    tars_resample_current = 0;
    tars_resample_phase = 0;

    tars_mp3_decoder_eof = false;
    tars_mp3_decoder_error = false;

    tars_tts_playing = false;
}

/* =========================================================
   CLEAR FULL TTS STATE
   ========================================================= */

static void tars_clear_tts_state(void)
{
    tars_tts_flash_size = 0;

    tars_tts_read_pos = 0;

    tars_tts_ready = false;
    tars_tts_playing = false;

    tars_pcm_ring_reset();

    tars_flash_buffer_start = 0;
    tars_flash_buffer_length = 0;

    tars_resample_previous = 0;
    tars_resample_current = 0;
    tars_resample_phase = 0;

    tars_mp3_decoder_eof = false;
    tars_mp3_decoder_error = false;
}

/* =========================================================
   ERASE TTS FLASH
   ========================================================= */

static bool tars_erase_tts_flash(void)
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

static bool tars_load_flash_buffer(
    size_t position
)
{
    if (tars_tts_partition == NULL) {
        return false;
    }

    if (position >= tars_tts_flash_size) {
        return false;
    }

    size_t aligned =
        position -
        (position % TARS_FLASH_READ_BUFFER_SIZE);

    size_t remaining =
        tars_tts_flash_size - aligned;

    size_t read_len =
        remaining;

    if (read_len >
        TARS_FLASH_READ_BUFFER_SIZE) {
        read_len =
            TARS_FLASH_READ_BUFFER_SIZE;
    }

    esp_err_t ret =
        esp_partition_read(
            tars_tts_partition,
            aligned,
            tars_flash_read_buffer,
            read_len
        );

    if (ret != ESP_OK) {
        return false;
    }

    tars_flash_buffer_start = aligned;
    tars_flash_buffer_length = read_len;

    return true;
}

/* =========================================================
   READ FLASH BYTES
   ========================================================= */

static size_t tars_read_flash_bytes(
    size_t position,
    uint8_t *dst,
    size_t len
)
{
    if (dst == NULL || len == 0) {
        return 0;
    }

    size_t total = 0;

    while (total < len &&
           position + total <
           tars_tts_flash_size) {

        size_t p = position + total;

        bool inside =
            p >= tars_flash_buffer_start &&
            p <
            tars_flash_buffer_start +
            tars_flash_buffer_length;

        if (!inside) {
            if (!tars_load_flash_buffer(p)) {
                break;
            }
        }

        size_t local =
            p - tars_flash_buffer_start;

        size_t available =
            tars_flash_buffer_length -
            local;

        size_t wanted =
            len - total;

        if (wanted > available) {
            wanted = available;
        }

        size_t remain_flash =
            tars_tts_flash_size - p;

        if (wanted > remain_flash) {
            wanted = remain_flash;
        }

        memcpy(
            dst + total,
            tars_flash_read_buffer + local,
            wanted
        );

        total += wanted;
    }

    return total;
}

/* =========================================================
   VOLUME
   ========================================================= */

static inline int16_t tars_apply_volume(
    int16_t sample
)
{
    int32_t v =
        ((int32_t)sample *
         TARS_TTS_VOLUME_NUM) /
        TARS_TTS_VOLUME_DEN;

    if (v > 32767) {
        v = 32767;
    }

    if (v < -32768) {
        v = -32768;
    }

    return (int16_t)v;
}

/* =========================================================
   DEBUG SAMPLE
   ========================================================= */

static void tars_debug_sample(
    int16_t sample
)
{
    if (!tars_tts_debug_valid) {

        tars_tts_debug_first_sample =
            sample;

        tars_tts_debug_min_sample =
            sample;

        tars_tts_debug_max_sample =
            sample;

        tars_tts_debug_valid = true;
    }
    else {

        if (sample <
            tars_tts_debug_min_sample) {

            tars_tts_debug_min_sample =
                sample;
        }

        if (sample >
            tars_tts_debug_max_sample) {

            tars_tts_debug_max_sample =
                sample;
        }
    }

    tars_tts_debug_sample_count++;
}

/* =========================================================
   INTERNAL TONE
   ========================================================= */

static int32_t tars_generate_tone(
    uint8_t *data,
    int32_t len
)
{
    if (data == NULL || len <= 0) {
        return 0;
    }

    int32_t usable =
        len - (len % 4);

    int32_t pos = 0;

    uint32_t increment =
        (uint32_t)(
            (
                (uint64_t)tars_tone_frequency *
                4294967296ULL
            ) /
            TARS_SAMPLE_RATE
        );

    while (pos < usable) {

        int16_t sample;

        if (tars_tone_phase &
            0x80000000UL) {
            sample = 3500;
        }
        else {
            sample = -3500;
        }

        data[pos + 0] =
            (uint8_t)(sample & 0xFF);

        data[pos + 1] =
            (uint8_t)((sample >> 8) & 0xFF);

        data[pos + 2] =
            (uint8_t)(sample & 0xFF);

        data[pos + 3] =
            (uint8_t)((sample >> 8) & 0xFF);

        pos += 4;

        tars_tone_phase += increment;
    }

    return usable;
}

/* =========================================================
   HTTP RECEIVE
   MP3 DIRECTLY TO FLASH
   ========================================================= */

static esp_err_t tars_tts_http_event(
    esp_http_client_event_t *evt
)
{
    if (evt == NULL) {
        return ESP_FAIL;
    }

    switch (evt->event_id) {

        case HTTP_EVENT_ON_CONNECTED:

            tars_status_text =
                "TTS HTTP CONNECTED";

            break;

        case HTTP_EVENT_HEADERS_SENT:

            tars_status_text =
                "TTS REQUEST SENT";

            break;

        case HTTP_EVENT_ON_DATA: {

            if (evt->data == NULL ||
                evt->data_len <= 0) {
                return ESP_OK;
            }

            if (tars_tts_header_debug_size <
                TARS_TTS_HEADER_DEBUG_SIZE) {

                size_t available =
                    TARS_TTS_HEADER_DEBUG_SIZE -
                    tars_tts_header_debug_size;

                size_t copy_len =
                    (size_t)evt->data_len;

                if (copy_len > available) {
                    copy_len = available;
                }

                memcpy(
                    tars_tts_header_debug +
                    tars_tts_header_debug_size,
                    evt->data,
                    copy_len
                );

                tars_tts_header_debug_size +=
                    copy_len;
            }

            if (tars_bt_started) {

                tars_tts_error =
                    "BLUETOOTH ACTIVE DURING DOWNLOAD";

                return ESP_FAIL;
            }

            if (!tars_find_tts_partition()) {

                tars_tts_error =
                    "TTS FLASH PARTITION NOT FOUND";

                return ESP_FAIL;
            }

            size_t incoming =
                (size_t)evt->data_len;

            if (tars_tts_flash_size +
                incoming >
                tars_tts_partition->size) {

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

            break;
        }

        case HTTP_EVENT_ON_FINISH:

            tars_status_text =
                "TTS HTTP FINISHED";

            break;

        default:
            break;
    }

    return ESP_OK;
}

/* =========================================================
   JSON ESCAPE
   ========================================================= */

static char *tars_json_escape(
    const char *text,
    size_t length,
    size_t *out_length
)
{
    size_t capacity =
        (length * 2) + 32;

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

    for (size_t i = 0; i < length; i++) {

        char c = text[i];

        if (c == '"' || c == '\\') {

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
        else if ((unsigned char)c < 32) {

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
   MP3 DECODER OPEN
   ========================================================= */

static bool tars_mp3_decoder_open(void)
{
    if (tars_mp3_decoder != NULL) {
        return true;
    }

    esp_audio_dec_register_default();

    esp_audio_simple_dec_register_default();

    esp_audio_simple_dec_cfg_t cfg = {
        .dec_type =
            ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
        .dec_cfg = NULL,
        .cfg_size = 0,
        .use_frame_dec = false
    };

    esp_audio_err_t ret =
        esp_audio_simple_dec_open(
            &cfg,
            &tars_mp3_decoder
        );

    if (ret != ESP_AUDIO_ERR_OK) {

        tars_mp3_decoder = NULL;

        esp_audio_simple_dec_unregister_default();
        esp_audio_dec_unregister_default();

        return false;
    }

    return true;
}

/* =========================================================
   MP3 DECODER CLOSE
   ========================================================= */

static void tars_mp3_decoder_close(void)
{
    if (tars_mp3_decoder != NULL) {

        esp_audio_simple_dec_close(
            tars_mp3_decoder
        );

        tars_mp3_decoder = NULL;
    }

    esp_audio_simple_dec_unregister_default();
    esp_audio_dec_unregister_default();
}

/* =========================================================
   WRITE PCM TO RING
   ========================================================= */

static bool tars_decoder_write_pcm(
    const uint8_t *pcm,
    size_t bytes
)
{
    if (pcm == NULL || bytes < 2) {
        return true;
    }

    size_t channels =
        tars_mp3_channels;

    size_t sample_bytes =
        channels * 2;

    if (sample_bytes == 0) {
        return false;
    }

    size_t samples =
        bytes / sample_bytes;

    if (samples == 0) {
        return true;
    }

    /*
     * Decoder output can be 1 or 2 channel.
     * Convert to 44.1 kHz stereo.
     */

    const int16_t *src =
        (const int16_t *)pcm;

    uint32_t src_rate =
        tars_mp3_sample_rate;

    if (src_rate == 0) {
        src_rate = TARS_SAMPLE_RATE;
    }

    /*
     * If source already 44100,
     * direct conversion is cheapest.
     */

    if (src_rate == TARS_SAMPLE_RATE) {

        for (size_t i = 0; i < samples; i++) {

            int16_t left;
            int16_t right;

            if (channels == 1) {

                left = src[i];
                right = src[i];
            }
            else {

                left = src[i * 2];
                right = src[i * 2 + 1];
            }

            left =
                tars_apply_volume(left);

            right =
                tars_apply_volume(right);

            uint8_t out[4];

            out[0] =
                (uint8_t)(left & 0xFF);

            out[1] =
                (uint8_t)((left >> 8) & 0xFF);

            out[2] =
                (uint8_t)(right & 0xFF);

            out[3] =
                (uint8_t)((right >> 8) & 0xFF);

            while (
                tars_pcm_ring_free() < 4 &&
                tars_tts_playing &&
                !tars_mp3_decoder_error
            ) {
                vTaskDelay(1);
            }

            if (!tars_tts_playing) {
                return false;
            }

            tars_pcm_ring_write_data(
                out,
                4
            );

            tars_debug_sample(left);
        }

        return true;
    }

    /*
     * Linear resampling.
     *
     * Phase is source samples in Q32.
     */

    uint64_t step =
        ((uint64_t)src_rate << 32) /
        TARS_SAMPLE_RATE;

    for (size_t out_index = 0;
         ;
         out_index++) {

        uint64_t phase =
            tars_resample_phase;

        size_t index =
            (size_t)(phase >> 32);

        if (index >= samples) {
            break;
        }

        size_t next =
            index + 1;

        if (next >= samples) {
            next = index;
        }

        int32_t frac =
            (int32_t)(phase & 0xFFFFFFFFULL);

        int32_t l1;
        int32_t l2;
        int32_t r1;
        int32_t r2;

        if (channels == 1) {

            l1 = src[index];
            l2 = src[next];

            r1 = l1;
            r2 = l2;
        }
        else {

            l1 =
                src[index * 2];

            l2 =
                src[next * 2];

            r1 =
                src[index * 2 + 1];

            r2 =
                src[next * 2 + 1];
        }

        int32_t left =
            l1 +
            (int32_t)(
                (
                    (int64_t)(l2 - l1) *
                    frac
                ) >> 32
            );

        int32_t right =
            r1 +
            (int32_t)(
                (
                    (int64_t)(r2 - r1) *
                    frac
                ) >> 32
            );

        int16_t ls =
            tars_apply_volume(
                (int16_t)left
            );

        int16_t rs =
            tars_apply_volume(
                (int16_t)right
            );

        uint8_t out[4];

        out[0] =
            (uint8_t)(ls & 0xFF);

        out[1] =
            (uint8_t)((ls >> 8) & 0xFF);

        out[2] =
            (uint8_t)(rs & 0xFF);

        out[3] =
            (uint8_t)((rs >> 8) & 0xFF);

        while (
            tars_pcm_ring_free() < 4 &&
            tars_tts_playing &&
            !tars_mp3_decoder_error
        ) {
            vTaskDelay(1);
        }

        if (!tars_tts_playing) {
            return false;
        }

        tars_pcm_ring_write_data(
            out,
            4
        );

        tars_resample_phase += step;

        tars_debug_sample(ls);

        /*
         * Prevent phase overflow while
         * preserving fractional part.
         */

        if (tars_resample_phase >
            ((uint64_t)samples << 32)) {

            tars_resample_phase =
                ((uint64_t)samples << 32);

            break;
        }
    }

    /*
     * Keep phase relative to this decoder
     * output buffer.
     */

    uint64_t consumed =
        ((uint64_t)samples << 32);

    if (tars_resample_phase >= consumed) {

        tars_resample_phase -= consumed;
    }

    return true;
}

/* =========================================================
   MP3 DECODER TASK
   ========================================================= */

static void tars_mp3_decode_task(
    void *arg
)
{
    (void)arg;

    tars_mp3_decoder_running = true;
    tars_mp3_decoder_eof = false;
    tars_mp3_decoder_error = false;

    tars_status_text =
        "MP3 DECODER STARTED";

    if (!tars_mp3_decoder_open()) {

        tars_mp3_decoder_error = true;
        tars_mp3_decoder_running = false;

        tars_status_text =
            "MP3 DECODER OPEN FAILED";

        tars_mp3_task_handle = NULL;

        vTaskDelete(NULL);
        return;
    }

    size_t flash_pos = 0;

    bool source_finished = false;

    while (
        tars_tts_playing &&
        !tars_mp3_decoder_error
    ) {

        /*
         * Do not allow ring buffer to
         * overflow.
         */

        if (tars_pcm_ring_free() <
            1024) {

            vTaskDelay(2);
            continue;
        }

        if (!source_finished) {

            size_t remaining =
                tars_tts_flash_size -
                flash_pos;

            if (remaining == 0) {

                source_finished = true;
            }
            else {

                size_t read_len =
                    remaining;

                if (read_len >
                    TARS_MP3_INPUT_BUFFER_SIZE) {

                    read_len =
                        TARS_MP3_INPUT_BUFFER_SIZE;
                }

                size_t got =
                    tars_read_flash_bytes(
                        flash_pos,
                        tars_mp3_input,
                        read_len
                    );

                if (got == 0) {

                    tars_mp3_decoder_error =
                        true;

                    break;
                }

                flash_pos += got;

                esp_audio_simple_dec_raw_t raw = {
                    .buffer =
                        tars_mp3_input,
                    .len =
                        (uint32_t)got,
                    .eos =
                        (flash_pos >=
                         tars_tts_flash_size),
                    .consumed = 0
                };

                while (
                    raw.len > 0 &&
                    tars_tts_playing
                ) {

                    esp_audio_simple_dec_out_t out = {
                        .buffer =
                            tars_mp3_output,
                        .len =
                            TARS_MP3_OUTPUT_BUFFER_SIZE,
                        .needed_size = 0,
                        .decoded_size = 0
                    };

                    esp_audio_err_t ret =
                        esp_audio_simple_dec_process(
                            tars_mp3_decoder,
                            &raw,
                            &out
                        );

                    if (ret ==
                        ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {

                        /*
                         * We intentionally do not
                         * dynamically enlarge the
                         * output buffer.
                         *
                         * MP3 frames normally fit
                         * inside 4096 bytes.
                         */

                        tars_mp3_decoder_error =
                            true;

                        break;
                    }

                    if (ret !=
                        ESP_AUDIO_ERR_OK) {

                        tars_mp3_decoder_error =
                            true;

                        break;
                    }

                    if (out.decoded_size > 0) {

                        /*
                         * Get audio information
                         * after first decoded frame.
                         */

                        esp_audio_simple_dec_info_t info;

                        memset(
                            &info,
                            0,
                            sizeof(info)
                        );

                        if (
                            esp_audio_simple_dec_get_info(
                                tars_mp3_decoder,
                                &info
                            ) ==
                            ESP_AUDIO_ERR_OK
                        ) {

                            if (info.sample_rate >
                                0) {

                                tars_mp3_sample_rate =
                                    info.sample_rate;
                            }

                            if (
                                info.channel >= 1 &&
                                info.channel <= 2
                            ) {

                                tars_mp3_channels =
                                    info.channel;
                            }

                            if (info.bits_per_sample >
                                0) {

                                tars_mp3_bits =
                                    info.bits_per_sample;
                            }
                        }

                        if (tars_mp3_bits != 16) {

                            tars_mp3_decoder_error =
                                true;

                            break;
                        }

                        if (
                            !tars_decoder_write_pcm(
                                out.buffer,
                                out.decoded_size
                            )
                        ) {

                            tars_mp3_decoder_error =
                                true;

                            break;
                        }
                    }

                    if (raw.consumed == 0) {

                        /*
                         * Decoder may keep data
                         * internally.
                         */

                        break;
                    }

                    if (raw.consumed >
                        raw.len) {

                        tars_mp3_decoder_error =
                            true;

                        break;
                    }

                    raw.len -= raw.consumed;
                    raw.buffer += raw.consumed;
                    raw.consumed = 0;
                }
            }
        }
        else {

            /*
             * All MP3 data has been supplied.
             * Wait until PCM ring drains.
             */

            if (tars_pcm_ring_used() == 0) {
                break;
            }

            vTaskDelay(2);
        }
    }

    /*
     * Drain decoded PCM before ending.
     */

    if (!tars_mp3_decoder_error) {

        while (
            tars_pcm_ring_used() > 0 &&
            tars_tts_playing
        ) {
            vTaskDelay(2);
        }
    }

    if (tars_mp3_decoder_error) {

        tars_status_text =
            "MP3 DECODER ERROR";
    }
    else {

        tars_mp3_decoder_eof = true;

        if (tars_tts_playing) {
            tars_status_text =
                "TTS FINISHED";
        }
    }

    tars_mp3_decoder_close();

    tars_mp3_decoder_running = false;

    if (tars_tts_playing &&
        !tars_mp3_decoder_error) {

        tars_tts_playing = false;
        tars_tts_read_pos =
            tars_tts_flash_size;
    }

    tars_mp3_task_handle = NULL;

    vTaskDelete(NULL);
}

/* =========================================================
   START MP3 DECODER TASK
   ========================================================= */

static bool tars_start_mp3_decoder(void)
{
    if (tars_mp3_task_handle != NULL) {
        return true;
    }

    tars_mp3_sample_rate = 44100;
    tars_mp3_channels = 1;
    tars_mp3_bits = 16;

    tars_mp3_decoder_eof = false;
    tars_mp3_decoder_error = false;

    tars_resample_phase = 0;

    BaseType_t ret =
        xTaskCreatePinnedToCore(
            tars_mp3_decode_task,
            "tars_mp3",
            TARS_MP3_TASK_STACK_SIZE,
            NULL,
            TARS_MP3_TASK_PRIORITY,
            &tars_mp3_task_handle,
            0
        );

    return ret == pdPASS;
}

/* =========================================================
   A2DP AUDIO CALLBACK
   ONLY PCM RING BUFFER
   NO FLASH
   NO MP3 DECODER
   ========================================================= */

static int32_t tars_a2dp_data_callback(
    uint8_t *data,
    int32_t len
)
{
    if (data == NULL || len <= 0) {
        return 0;
    }

    int32_t usable =
        len - (len % 4);

    if (!tars_audio_started) {

        memset(
            data,
            0,
            usable
        );

        return usable;
    }

    /*
     * TTS PCM already converted to
     * 44.1 kHz stereo.
     */

    if (tars_tts_playing ||
        tars_pcm_ring_used() > 0) {

        size_t got =
            tars_pcm_ring_read_data(
                data,
                usable
            );

        if (got < (size_t)usable) {

            memset(
                data + got,
                0,
                usable - got
            );

            /*
             * When decoder has finished and
             * buffer is empty, end playback.
             */

            if (
                tars_mp3_decoder_eof &&
                tars_pcm_ring_used() == 0
            ) {

                tars_tts_playing = false;

                tars_status_text =
                    "TTS FINISHED";
            }
        }

        tars_tts_read_pos =
            tars_tts_flash_size -
            tars_pcm_ring_used();

        return usable;
    }

    /*
     * INTERNAL TONE
     */

    if (tars_tone_enabled) {

        return tars_generate_tone(
            data,
            usable
        );
    }

    /*
     * SILENCE
     */

    memset(
        data,
        0,
        usable
    );

    return usable;
}

/* =========================================================
   REQUEST AUDIO START
   ========================================================= */

static esp_err_t tars_request_audio_start(void)
{
    if (!tars_bt_started ||
        !tars_a2dp_connected) {

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

    esp_err_t ret =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_STOP
        );

    if (ret != ESP_OK) {
        tars_media_stop_pending = false;
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

            switch (
                param->conn_stat.state
            ) {

                case ESP_A2D_CONNECTION_STATE_DISCONNECTED:

                    tars_a2dp_connected = false;
                    tars_a2dp_connecting = false;
                    tars_audio_started = false;

                    tars_tone_enabled = false;
                    tars_tts_playing = false;

                    tars_media_check_pending = false;
                    tars_media_start_requested = false;
                    tars_media_start_pending = false;
                    tars_media_stop_pending = false;

                    tars_pcm_ring_reset();

                    if (!tars_bt_stopping) {
                        tars_status_text =
                            "A2DP DISCONNECTED";
                    }

                    break;

                case ESP_A2D_CONNECTION_STATE_CONNECTING:
                    break;

                default:
                    break;
            }

            /*
             * Handle CONNECTING separately because
             * enum spelling can differ between IDF
             * revisions.
             */

            if (
                param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_CONNECTING
            ) {

                tars_a2dp_connected = false;
                tars_a2dp_connecting = true;
                tars_audio_started = false;

                tars_status_text =
                    "A2DP CONNECTING";
            }

            if (
                param->conn_stat.state ==
                ESP_A2D_CONNECTION_STATE_CONNECTED
            ) {

                tars_a2dp_connected = true;
                tars_a2dp_connecting = false;
                tars_audio_started = false;

                tars_status_text =
                    "A2DP CONNECTED";
            }

            break;

        case ESP_A2D_AUDIO_STATE_EVT:

            switch (
                param->audio_stat.state
            ) {

                case ESP_A2D_AUDIO_STATE_STARTED:

                    tars_audio_started = true;

                    tars_media_check_pending = false;
                    tars_media_start_requested = false;
                    tars_media_start_pending = false;
                    tars_media_stop_pending = false;

                    if (tars_tts_playing) {

                        tars_status_text =
                            "TTS MP3 PLAYING";
                    }
                    else {

                        tars_status_text =
                            "A2DP AUDIO STREAMING";
                    }

                    break;

                case ESP_A2D_AUDIO_STATE_STOPPED:

                    tars_audio_started = false;

                    tars_media_check_pending = false;
                    tars_media_start_requested = false;
                    tars_media_start_pending = false;
                    tars_media_stop_pending = false;

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
   GAP CALLBACK
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

        case ESP_BT_GAP_DISC_RES_EVT: {

            if (tars_device_found) {
                break;
            }

            uint8_t *eir = NULL;

            int i = 0;

            while (
                i < param->disc_res.num_prop
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

                i++;
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
                        name_len &&
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

                tars_scanning = false;
            }

            break;

        default:
            break;
    }
}

/* =========================================================
   RESET BLUETOOTH
   ========================================================= */

static void tars_reset_bluetooth_state(void)
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
   BT MEMORY CHECK
   ========================================================= */

static bool tars_try_start_bt(void)
{
    esp_err_t release_ret =
        esp_bt_controller_mem_release(
            ESP_BT_MODE_BLE
        );

    if (
        release_ret != ESP_OK &&
        release_ret != ESP_ERR_INVALID_STATE
    ) {

        printf(
            "TARS BT: BLE MEMORY RELEASE ERROR: %d\n",
            (int)release_ret
        );
    }

    size_t free_heap =
        heap_caps_get_free_size(
            MALLOC_CAP_8BIT
        );

    size_t largest =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_8BIT
        );

    printf(
        "TARS BT BEFORE START | "
        "HEAP=%u | "
        "LARGEST=%u | "
        "MIN_HEAP=%u | "
        "MIN_LARGEST=%u\n",

        (unsigned int)free_heap,
        (unsigned int)largest,
        (unsigned int)TARS_BT_MIN_HEAP,
        (unsigned int)TARS_BT_MIN_LARGEST
    );

    if (free_heap <
        TARS_BT_MIN_HEAP) {

        printf(
            "TARS: BT NOT STARTED - "
            "FREE HEAP TOO LOW\n"
        );

        return false;
    }

    if (largest <
        TARS_BT_MIN_LARGEST) {

        printf(
            "TARS: BT NOT STARTED - "
            "INSUFFICIENT CONTIGUOUS HEAP\n"
        );

        return false;
    }

    return true;
}

/* =========================================================
   START BLUETOOTH
   ========================================================= */

static mp_obj_t tars_a2dp_start(void)
{
    esp_err_t ret;

    if (tars_tts_loading) {

        return mp_obj_new_str(
            "ERROR: TTS DOWNLOAD STILL RUNNING",
            33
        );
    }

    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            26
        );
    }

    if (tars_bt_started) {

        return mp_obj_new_str(
            "TARS BLUETOOTH ALREADY STARTED",
            31
        );
    }

    if (!tars_try_start_bt()) {

        size_t free_heap =
            heap_caps_get_free_size(
                MALLOC_CAP_8BIT
            );

        size_t largest =
            heap_caps_get_largest_free_block(
                MALLOC_CAP_8BIT
            );

        char result[160];

        snprintf(
            result,
            sizeof(result),
            "ERROR: BT MEMORY LOW | "
            "HEAP:%u | "
            "LARGEST:%u",

            (unsigned int)free_heap,
            (unsigned int)largest
        );

        tars_status_text =
            "BT START BLOCKED LOW MEMORY";

        return mp_obj_new_str(
            result,
            strlen(result)
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

        tars_status_text =
            "BT CONTROLLER INIT FAILED";

        return mp_obj_new_str(
            "ERROR: BT CONTROLLER INIT FAILED",
            35
        );
    }

    ret =
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT
        );

    if (ret != ESP_OK) {

        esp_bt_controller_deinit();

        tars_status_text =
            "BT CONTROLLER ENABLE FAILED";

        return mp_obj_new_str(
            "ERROR: BT CONTROLLER ENABLE FAILED",
            37
        );
    }

    ret =
        esp_bluedroid_init();

    if (ret != ESP_OK) {

        esp_bt_controller_disable();
        esp_bt_controller_deinit();

        tars_status_text =
            "BLUEDROID INIT FAILED";

        return mp_obj_new_str(
            "ERROR: BLUEDROID INIT FAILED",
            30
        );
    }

    ret =
        esp_bluedroid_enable();

    if (ret != ESP_OK) {

        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();

        tars_status_text =
            "BLUEDROID ENABLE FAILED";

        return mp_obj_new_str(
            "ERROR: BLUEDROID ENABLE FAILED",
            32
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

        tars_status_text =
            "GAP CALLBACK FAILED";

        return mp_obj_new_str(
            "ERROR: GAP CALLBACK FAILED",
            28
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

        tars_status_text =
            "A2DP CALLBACK FAILED";

        return mp_obj_new_str(
            "ERROR: A2DP CALLBACK FAILED",
            29
        );
    }

    ret =
        esp_a2d_source_init();

    if (ret != ESP_OK) {

        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();

        tars_status_text =
            "A2DP SOURCE INIT FAILED";

        return mp_obj_new_str(
            "ERROR: A2DP SOURCE INIT FAILED",
            32
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

        tars_status_text =
            "AUDIO CALLBACK FAILED";

        return mp_obj_new_str(
            "ERROR: AUDIO CALLBACK FAILED",
            31
        );
    }

    tars_bt_started = true;
    tars_bt_stopping = false;

    size_t free_heap_after =
        heap_caps_get_free_size(
            MALLOC_CAP_8BIT
        );

    size_t largest_after =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_8BIT
        );

    printf(
        "TARS BT STARTED | "
        "HEAP=%u | "
        "LARGEST=%u\n",

        (unsigned int)free_heap_after,
        (unsigned int)largest_after
    );

    tars_status_text =
        "TARS BLUETOOTH READY";

    char result[180];

    snprintf(
        result,
        sizeof(result),
        "TARS BLUETOOTH CLASSIC A2DP READY | "
        "HEAP:%u | "
        "LARGEST:%u",

        (unsigned int)free_heap_after,
        (unsigned int)largest_after
    );

    return mp_obj_new_str(
        result,
        strlen(result)
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);

/* =========================================================
   SCAN
   ========================================================= */

static mp_obj_t tars_a2dp_scan(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            29
        );
    }

    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            26
        );
    }

    if (tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: ALREADY CONNECTED",
            25
        );
    }

    if (tars_scanning) {

        return mp_obj_new_str(
            "TARS ALREADY SCANNING",
            22
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
            29
        );
    }

    tars_scanning = true;

    tars_status_text =
        "SCANNING FOR I7-TWS";

    return mp_obj_new_str(
        "TARS SCANNING FOR I7-TWS...",
        28
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_scan_obj,
    tars_a2dp_scan
);

/* =========================================================
   FOUND
   ========================================================= */

static mp_obj_t tars_a2dp_found(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH NOT STARTED",
            29
        );
    }

    if (tars_device_found) {

        return mp_obj_new_str(
            "I7-TWS FOUND",
            12
        );
    }

    if (tars_scanning) {

        return mp_obj_new_str(
            "STILL SCANNING",
            14
        );
    }

    return mp_obj_new_str(
        "I7-TWS NOT FOUND",
        16
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_found_obj,
    tars_a2dp_found
);

/* =========================================================
   CONNECT
   ========================================================= */

static mp_obj_t tars_a2dp_connect(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            29
        );
    }

    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            26
        );
    }

    if (!tars_device_found) {

        return mp_obj_new_str(
            "ERROR: I7-TWS NOT FOUND",
            24
        );
    }

    if (tars_a2dp_connected) {

        return mp_obj_new_str(
            "TARS ALREADY CONNECTED",
            22
        );
    }

    if (tars_a2dp_connecting) {

        return mp_obj_new_str(
            "TARS ALREADY CONNECTING",
            24
        );
    }

    esp_err_t ret =
        esp_a2d_source_connect(
            tars_target_bda
        );

    if (ret != ESP_OK) {

        return mp_obj_new_str(
            "ERROR: A2DP CONNECT FAILED",
            28
        );
    }

    tars_a2dp_connecting = true;

    tars_status_text =
        "A2DP CONNECTING";

    return mp_obj_new_str(
        "TARS CONNECTING TO I7-TWS...",
        29
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_connect_obj,
    tars_a2dp_connect
);

/* =========================================================
   TTS DOWNLOAD
   MP3 FROM CLOUDFLARE -> FLASH
   ========================================================= */

static mp_obj_t tars_a2dp_tts_download(
    mp_obj_t text_obj
)
{
    if (tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: STOP BLUETOOTH BEFORE DOWNLOAD",
            39
        );
    }

    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STILL STOPPING",
            33
        );
    }

    if (tars_tts_loading) {

        return mp_obj_new_str(
            "ERROR: TTS ALREADY DOWNLOADING",
            32
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
            22
        );
    }

    if (
        text_length >
        TARS_TTS_MAX_TEXT_LENGTH
    ) {

        return mp_obj_new_str(
            "ERROR: TTS TEXT TOO LONG MAX 300",
            34
        );
    }

    if (!tars_find_tts_partition()) {

        return mp_obj_new_str(
            "ERROR: TTS FLASH PARTITION NOT FOUND",
            39
        );
    }

    tars_clear_tts_state();
    tars_reset_tts_debug();

    tars_tts_error = "";

    tars_tts_loading = true;

    tars_status_text =
        "TTS DOWNLOADING MP3";

    if (!tars_erase_tts_flash()) {

        tars_tts_loading = false;

        return mp_obj_new_str(
            "ERROR: TTS FLASH ERASE FAILED",
            31
        );
    }

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
            31
        );
    }

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
            120000,

        .buffer_size =
            TARS_HTTP_BUFFER_SIZE,

        .buffer_size_tx =
            TARS_HTTP_BUFFER_SIZE,

        .disable_auto_redirect =
            false,

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

        tars_status_text =
            "HTTP CLIENT INIT FAILED";

        return mp_obj_new_str(
            "ERROR: HTTP CLIENT INIT FAILED",
            32
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
        "audio/mpeg"
    );

    esp_http_client_set_header(
        client,
        "User-Agent",
        "TARS-V1-MANZ"
    );

    esp_http_client_set_post_field(
        client,
        json,
        (int)json_length
    );

    tars_status_text =
        "TTS MP3 HTTP REQUEST";

    esp_err_t ret =
        esp_http_client_perform(
            client
        );

    int status_code =
        esp_http_client_get_status_code(
            client
        );

    int64_t content_length =
        esp_http_client_get_content_length(
            client
        );

    esp_http_client_cleanup(
        client
    );

    heap_caps_free(json);

    tars_tts_loading = false;

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

    if (ret != ESP_OK) {

        tars_clear_tts_state();

        tars_status_text =
            "TTS HTTP FAILED";

        return mp_obj_new_str(
            "ERROR: TTS HTTP REQUEST FAILED",
            33
        );
    }

    if (
        status_code < 200 ||
        status_code >= 300
    ) {

        tars_clear_tts_state();

        char result[80];

        snprintf(
            result,
            sizeof(result),
            "ERROR: TTS HTTP %d",
            status_code
        );

        tars_status_text =
            "TTS HTTP ERROR";

        return mp_obj_new_str(
            result,
            strlen(result)
        );
    }

    if (tars_tts_flash_size < 16) {

        tars_clear_tts_state();

        tars_status_text =
            "TTS MP3 EMPTY";

        return mp_obj_new_str(
            "ERROR: TTS EMPTY MP3",
            21
        );
    }

    /*
     * Check MP3 signature.
     *
     * ID3 or FF FB/F3/F2.
     */

    uint8_t header[4] = {0};

    size_t got =
        tars_read_flash_bytes(
            0,
            header,
            sizeof(header)
        );

    bool mp3_ok = false;

    if (
        got >= 3 &&
        header[0] == 0x49 &&
        header[1] == 0x44 &&
        header[2] == 0x33
    ) {

        mp3_ok = true;
    }

    if (
        got >= 2 &&
        header[0] == 0xFF &&
        (
            header[1] == 0xFB ||
            header[1] == 0xF3 ||
            header[1] == 0xF2
        )
    ) {

        mp3_ok = true;
    }

    if (!mp3_ok) {

        tars_clear_tts_state();

        tars_status_text =
            "INVALID MP3";

        return mp_obj_new_str(
            "ERROR: INVALID MP3 AUDIO",
            26
        );
    }

    tars_tts_ready = true;

    tars_status_text =
        "TTS MP3 SAVED TO FLASH";

    char result[140];

    snprintf(
        result,
        sizeof(result),
        "MP3 SAVED: %u BYTES HTTP:%d",

        (unsigned int)
        tars_tts_flash_size,

        status_code
    );

    /*
     * content_length is only diagnostic.
     * Chunked HTTP may return -1.
     */

    (void)content_length;

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

static mp_obj_t tars_a2dp_tts_play(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "ERROR: START BLUETOOTH FIRST",
            29
        );
    }

    if (tars_bt_stopping) {

        return mp_obj_new_str(
            "ERROR: BLUETOOTH STOPPING",
            26
        );
    }

    if (!tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            27
        );
    }

    if (!tars_tts_ready) {

        return mp_obj_new_str(
            "ERROR: NO MP3 IN FLASH",
            23
        );
    }

    if (tars_tts_flash_size < 16) {

        return mp_obj_new_str(
            "ERROR: MP3 FLASH EMPTY",
            23
        );
    }

    if (tars_mp3_task_handle != NULL) {

        return mp_obj_new_str(
            "ERROR: MP3 ALREADY PLAYING",
            27
        );
    }

    tars_tone_enabled = false;

    tars_clear_tts_play_state();

    tars_reset_tts_debug();

    tars_tts_playing = true;

    tars_status_text =
        "TTS MP3 PREPARING";

    if (!tars_start_mp3_decoder()) {

        tars_tts_playing = false;

        tars_status_text =
            "MP3 TASK START FAILED";

        return mp_obj_new_str(
            "ERROR: MP3 DECODER TASK FAILED",
            33
        );
    }

    if (!tars_audio_started) {

        esp_err_t ret =
            tars_request_audio_start();

        if (ret != ESP_OK) {

            tars_tts_playing = false;

            return mp_obj_new_str(
                "ERROR: A2DP AUDIO START FAILED",
                33
            );
        }
    }

    return mp_obj_new_str(
        "TTS MP3 PLAY REQUESTED",
        23
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tts_play_obj,
    tars_a2dp_tts_play
);

/* =========================================================
   STOP AUDIO
   ========================================================= */

static mp_obj_t tars_a2dp_stop(void)
{
    tars_tone_enabled = false;

    tars_tts_playing = false;

    tars_tone_phase = 0;

    tars_resample_phase = 0;

    if (!tars_a2dp_connected) {

        return mp_obj_new_str(
            "ERROR: A2DP NOT CONNECTED",
            27
        );
    }

    if (!tars_audio_started) {

        return mp_obj_new_str(
            "A2DP AUDIO ALREADY STOPPED",
            27
        );
    }

    esp_err_t ret =
        tars_request_audio_stop();

    if (ret != ESP_OK) {

        return mp_obj_new_str(
            "ERROR: A2DP STOP FAILED",
            25
        );
    }

    tars_status_text =
        "A2DP STOP REQUESTED";

    return mp_obj_new_str(
        "TARS A2DP STOP REQUESTED",
        25
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_stop_obj,
    tars_a2dp_stop
);

/* =========================================================
   BLUETOOTH FULL STOP
   ========================================================= */

static mp_obj_t tars_a2dp_bluetooth_stop(void)
{
    if (!tars_bt_started) {

        return mp_obj_new_str(
            "BLUETOOTH ALREADY STOPPED",
            26
        );
    }

    if (tars_tts_loading) {

        return mp_obj_new_str(
            "ERROR: TTS DOWNLOAD ACTIVE",
            28
        );
    }

    tars_bt_stopping = true;

    tars_status_text =
        "BLUETOOTH STOPPING";

    tars_tone_enabled = false;

    tars_tts_playing = false;

    tars_tone_phase = 0;

    tars_resample_phase = 0;

    tars_pcm_ring_reset();

    if (tars_scanning) {

        esp_bt_gap_cancel_discovery();

        tars_scanning = false;
    }

    /*
     * Give MP3 task a short chance
     * to exit before decoder resources
     * are destroyed.
     */

    for (int i = 0; i < 20; i++) {

        if (tars_mp3_task_handle == NULL) {
            break;
        }

        vTaskDelay(1);
    }

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

    tars_bt_started = false;

    tars_bt_stopping = false;

    tars_reset_bluetooth_state();

    tars_status_text =
        "BLUETOOTH STOPPED";

    return mp_obj_new_str(
        "BLUETOOTH FULLY STOPPED - WIFI MODE READY",
        44
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_bluetooth_stop_obj,
    tars_a2dp_bluetooth_stop
);

/* =========================================================
   STATUS
   ========================================================= */

static mp_obj_t tars_a2dp_status(void)
{
    return mp_obj_new_str(
        tars_status_text,
        strlen(tars_status_text)
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_status_obj,
    tars_a2dp_status
);

/* =========================================================
   MEMORY
   ========================================================= */

static mp_obj_t tars_a2dp_memory(void)
{
    size_t free_8bit =
        heap_caps_get_free_size(
            MALLOC_CAP_8BIT
        );

    size_t largest_8bit =
        heap_caps_get_largest_free_block(
            MALLOC_CAP_8BIT
        );

    size_t partition_size = 0;

    if (tars_find_tts_partition()) {

        partition_size =
            tars_tts_partition->size;
    }

    char result[260];

    snprintf(
        result,
        sizeof(result),

        "HEAP:%u | "
        "LARGEST:%u | "
        "FLASH MP3:%u/%u | "
        "PCM:%u/%u | "
        "DECODER:%s | "
        "BT:%s",

        (unsigned int)free_8bit,

        (unsigned int)largest_8bit,

        (unsigned int)tars_tts_flash_size,

        (unsigned int)partition_size,

        (unsigned int)tars_pcm_ring_used(),

        (unsigned int)TARS_PCM_RING_SIZE,

        tars_mp3_decoder_running
            ? "ON"
            : "OFF",

        tars_bt_started
            ? "ON"
            : "OFF"
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

static mp_obj_t tars_a2dp_tts_info(void)
{
    char result[240];

    snprintf(
        result,
        sizeof(result),

        "TTS READY:%s | "
        "MP3:%u BYTES | "
        "PLAYING:%s | "
        "RING:%u | "
        "RATE:%u | "
        "CH:%u | "
        "DEC:%s",

        tars_tts_ready
            ? "YES"
            : "NO",

        (unsigned int)
        tars_tts_flash_size,

        tars_tts_playing
            ? "YES"
            : "NO",

        (unsigned int)
        tars_pcm_ring_used(),

        (unsigned int)
        tars_mp3_sample_rate,

        (unsigned int)
        tars_mp3_channels,

        tars_mp3_decoder_running
            ? "ON"
            : "OFF"
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
   TTS DEBUG
   ========================================================= */

static mp_obj_t tars_a2dp_tts_debug(void)
{
    char result[260];

    snprintf(
        result,
        sizeof(result),

        "TTS DEBUG | "
        "VALID:%s | "
        "FIRST:%d | "
        "MIN:%d | "
        "MAX:%d | "
        "SAMPLES:%u | "
        "MP3 RATE:%u | "
        "CH:%u | "
        "BITS:%u | "
        "A2DP:%u",

        tars_tts_debug_valid
            ? "YES"
            : "NO",

        (int)
        tars_tts_debug_first_sample,

        (int)
        tars_tts_debug_min_sample,

        (int)
        tars_tts_debug_max_sample,

        (unsigned int)
        tars_tts_debug_sample_count,

        (unsigned int)
        tars_mp3_sample_rate,

        (unsigned int)
        tars_mp3_channels,

        (unsigned int)
        tars_mp3_bits,

        (unsigned int)
        TARS_SAMPLE_RATE
    );

    return mp_obj_new_str(
        result,
        strlen(result)
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tts_debug_obj,
    tars_a2dp_tts_debug
);

/* =========================================================
   TTS HEADER
   ========================================================= */

static mp_obj_t tars_a2dp_tts_header(void)
{
    char result[360];

    char header_hex[100];

    size_t pos = 0;

    size_t i = 0;

    while (
        i <
        tars_tts_header_debug_size
    ) {

        int written =
            snprintf(
                header_hex + pos,
                sizeof(header_hex) - pos,
                "%02X ",
                (unsigned int)
                tars_tts_header_debug[i]
            );

        if (written <= 0) {
            break;
        }

        pos +=
            (size_t)written;

        if (
            pos >=
            sizeof(header_hex) - 4
        ) {
            break;
        }

        i++;
    }

    if (pos == 0) {

        snprintf(
            header_hex,
            sizeof(header_hex),
            "EMPTY"
        );
    }

    const char *format =
        "UNKNOWN";

    if (
        tars_tts_header_debug_size >= 3 &&
        tars_tts_header_debug[0] == 0x49 &&
        tars_tts_header_debug[1] == 0x44 &&
        tars_tts_header_debug[2] == 0x33
    ) {

        format = "MP3 ID3";
    }
    else if (
        tars_tts_header_debug_size >= 2 &&
        tars_tts_header_debug[0] == 0xFF &&
        (
            tars_tts_header_debug[1] == 0xFB ||
            tars_tts_header_debug[1] == 0xF3 ||
            tars_tts_header_debug[1] == 0xF2
        )
    ) {

        format = "MP3 FRAME";
    }
    else if (
        tars_tts_header_debug_size >= 4 &&
        tars_tts_header_debug[0] == 0x52 &&
        tars_tts_header_debug[1] == 0x49 &&
        tars_tts_header_debug[2] == 0x46 &&
        tars_tts_header_debug[3] == 0x46
    ) {

        format = "WAV RIFF";
    }
    else {

        format =
            "UNKNOWN AUDIO";
    }

    snprintf(
        result,
        sizeof(result),

        "TTS HEADER | "
        "FORMAT:%s | "
        "SIZE:%u | "
        "HEADER[%u]:%s",

        format,

        (unsigned int)
        tars_tts_flash_size,

        (unsigned int)
        tars_tts_header_debug_size,

        header_hex
    );

    return mp_obj_new_str(
        result,
        strlen(result)
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_tts_header_obj,
    tars_a2dp_tts_header
);

/* =========================================================
   MODULE FUNCTIONS
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
    },

    {
        MP_ROM_QSTR(
            MP_QSTR_tts_debug
        ),
        MP_ROM_PTR(
            &tars_a2dp_tts_debug_obj
        )
    },

    {
        MP_ROM_QSTR(
            MP_QSTR_tts_header
        ),
        MP_ROM_PTR(
            &tars_a2dp_tts_header_obj
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
tars_a2dp_user_cmodule = {

    .base = {
        &mp_type_module
    },

    .globals =
        (mp_obj_dict_t *)
        &tars_a2dp_globals
};

/* =========================================================
   REGISTER
   ========================================================= */

MP_REGISTER_MODULE(
    MP_QSTR_tars_a2dp,
    tars_a2dp_user_cmodule
);
