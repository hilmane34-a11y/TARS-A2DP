#define MICROPY_HW_BOARD_NAME "TARS ESP32 A2DP"
#define MICROPY_HW_MCU_NAME "ESP32"

// Sisakan lebih banyak RAM internal untuk WiFi dan HTTPS/TLS.
#define MICROPY_GC_INITIAL_HEAP_SIZE (28 * 1024)

// Matikan modul Bluetooth NimBLE bawaan MicroPython.
// Bluetooth Classic/A2DP akan ditangani oleh modul C TARS.
#define MICROPY_PY_BLUETOOTH (0)
