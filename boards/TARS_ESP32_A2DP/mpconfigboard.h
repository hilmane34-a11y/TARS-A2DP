#define MICROPY_HW_BOARD_NAME "TARS ESP32 A2DP"
#define MICROPY_HW_MCU_NAME "ESP32"

/*
 * Jangan membesarkan GC heap sebelum kebutuhan Bluetooth
 * benar-benar diukur.
 */
#define MICROPY_GC_INITIAL_HEAP_SIZE (36 * 1024)

/*
 * GC threshold sebaiknya jangan terlalu kecil karena bisa
 * membuat GC terlalu sering.
 */
#define MICROPY_GC_ALLOC_THRESHOLD (4096)

/*
 * Stack Python.
 * Jangan dinaikkan ke 8 KB tanpa bukti membutuhkan stack sebesar itu,
 * karena stack juga memakai RAM sistem.
 */
#define MICROPY_STACK_SIZE (6 * 1024)

/*
 * Bluetooth bawaan MicroPython tidak digunakan jika
 * Bluetooth Classic/A2DP ditangani modul C sendiri.
 */
#define MICROPY_PY_BLUETOOTH (0)

/* Fitur yang memang tidak digunakan bisa dimatikan
 * hanya jika kompatibel dengan source MicroPython versi kamu.
 */
#define MICROPY_PY_CMATH (0)

/*
 * Jangan menambahkan macro A2DP palsu seperti:
 *
 * MICROPY_A2DP_BUFFER_POOL_SIZE
 * MICROPY_A2DP_SBC_WORKSPACE
 * MICROPY_FAST_MEMCPY
 *
 * kecuali tars_a2dp.c memang secara eksplisit menggunakan macro tersebut.
 */
