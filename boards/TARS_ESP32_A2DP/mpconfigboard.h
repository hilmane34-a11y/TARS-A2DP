#define MICROPY_HW_BOARD_NAME "TARS ESP32 A2DP"
#define MICROPY_HW_MCU_NAME "ESP32"

// ============================================================
// MICROPYTHON MEMORY CONFIGURATION
// Disesuaikan untuk Bluetooth A2DP + TTS Cloud dengan 
// heap yang lebih besar saat A2DP aktif
// ============================================================

// Tingkatkan heap Python untuk A2DP
// 52 KB → 80 KB untuk buffer A2DP + Python objects
// Memory allocation: 40KB untuk A2DP, 40KB untuk Python
#define MICROPY_GC_INITIAL_HEAP_SIZE (35 * 1024)

// Aggressive garbage collection untuk prevent fragmentation
// Trigger lebih sering saat A2DP streaming
#define MICROPY_GC_ALLOC_THRESHOLD (2048)

// Stack size untuk Python tasks dengan A2DP support
// 4 KB → 8 KB untuk handle A2DP callbacks dan TTS
#define MICROPY_STACK_SIZE (8 * 1024)

// ============================================================
// ENABLE MEMORY POOL FOR A2DP BUFFERS
// ============================================================

// Memory pool untuk audio frames
// Alokasikan dedicated memory untuk A2DP streaming
#define MICROPY_A2DP_BUFFER_POOL_SIZE (16 * 1024)

// SBC encoder workspace
#define MICROPY_A2DP_SBC_WORKSPACE (8 * 1024)

// ============================================================
// DISABLE UNUSED MODULES (SAVE ~20 KB)
// ============================================================

// Matikan modul Bluetooth NimBLE bawaan MicroPython
// Bluetooth Classic/A2DP akan ditangani oleh modul C TARS
#define MICROPY_PY_BLUETOOTH (0)

// Disable complex math (hemat ~2 KB)
#define MICROPY_PY_CMATH (0)

// Disable urandom module (hemat ~2 KB)
#define MICROPY_PY_URANDOM (0)

// Disable regex groups (hemat ~3 KB)
#define MICROPY_PY_URE_MATCH_GROUPS (0)

// ============================================================
// ENABLE FEATURES UNTUK A2DP PERFORMANCE
// ============================================================

// Enable inline assembly untuk faster audio processing
#define MICROPY_INLINE_ASM (1)

// Disable debug printers untuk save memory
#define MICROPY_DEBUG_PRINTERS (0)

// Enable memory optimization untuk A2DP streaming
#define MICROPY_ALLOC_CHUNK_INIT_LEN (256)

// ============================================================
// A2DP SPECIFIC OPTIMIZATIONS
// ============================================================

// Enable fast memcpy untuk audio data
#define MICROPY_FAST_MEMCPY (1)

// Reduce Python VM overhead
#define MICROPY_SUPPRESS_BACKTRACE (1)

// ============================================================
// RECOMMENDED: Runtime Configuration untuk A2DP
// Jalankan kode ini di startup script Python:
// ============================================================
/*
import micropython
import gc

// Allocate extra heap untuk A2DP saat startup
micropython.alloc(16 * 1024)  // Extra 16KB untuk A2DP buffers

// Set GC threshold lebih aggressive saat A2DP aktif
gc.threshold(1024)

// Enable memcpy optimization
import a2dp
a2dp.enable_fast_memcpy()
*/
