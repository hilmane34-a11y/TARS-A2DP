#define MICROPY_HW_BOARD_NAME "TARS ESP32 A2DP"
#define MICROPY_HW_MCU_NAME "ESP32"

// ============================================================
// MICROPYTHON MEMORY CONFIGURATION
// Disesuaikan untuk keseimbangan antara Python script 
// dan Bluetooth A2DP + TTS Cloud
// ============================================================

// Tingkatkan dari 36 KB ke 52 KB untuk heap Python
// Cukup untuk script Python + temporary objects
#define MICROPY_GC_INITIAL_HEAP_SIZE (52 * 1024)

// Aggressive garbage collection
// Trigger GC lebih sering untuk prevent fragmentation
#define MICROPY_GC_ALLOC_THRESHOLD (1536)

// Stack size untuk Python tasks (cukup untuk TARS operations)
#define MICROPY_STACK_SIZE (4 * 1024)

// ============================================================
// DISABLE UNUSED MODULES (SAVE ~20 KB)
// ============================================================

// Matikan modul Bluetooth NimBLE bawaan MicroPython
// Bluetooth Classic/A2DP akan ditangani oleh modul C TARS
#define MICROPY_PY_BLUETOOTH (0)

// Disable complex math (tidak perlu untuk TARS)
#define MICROPY_PY_CMATH (0)

// Disable urandom module (hemat ~2 KB)
#define MICROPY_PY_URANDOM (0)

// Disable regex groups (hemat ~3 KB)
#define MICROPY_PY_URE_MATCH_GROUPS (0)

// ============================================================
// OPTIMIZATION FLAGS
// ============================================================

// Inline small functions
#define MICROPY_INLINE_ASM (0)

// Enable memory stats debugging
#define MICROPY_DEBUG_PRINTERS (0)
