#include "py/runtime.h"
#include "tars_bt.h"


// ========================================
// TEST
// ========================================

static mp_obj_t tars_a2dp_test(void) {

    return mp_obj_new_str(
        "TARS A2DP MODULE READY",
        22
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


// ========================================
// BLUETOOTH INITIALIZATION
// ========================================

static mp_obj_t tars_a2dp_init(void) {

    if (tars_bt_is_ready()) {

        return mp_obj_new_str(
            "Bluetooth already initialized",
            29
        );
    }

    int result = tars_bt_init();

    if (result != 0) {

        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT(
                "Bluetooth initialization failed"
            )
        );
    }

    return mp_obj_new_str(
        "Bluetooth Classic READY",
        23
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_init_obj,
    tars_a2dp_init
);


// ========================================
// MODULE
// ========================================

static const mp_rom_map_elem_t
tars_a2dp_globals_table[] = {

    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_a2dp)
    },

    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(
            &tars_a2dp_test_obj
        )
    },

    {
        MP_ROM_QSTR(MP_QSTR_init),
        MP_ROM_PTR(
            &tars_a2dp_init_obj
        )
    },

};

static MP_DEFINE_CONST_DICT(
    tars_a2dp_globals,
    tars_a2dp_globals_table
);


const mp_obj_module_t
tars_a2dp_user_cmodule = {

    .base = {
        &mp_type_module
    },

    .globals = (
        mp_obj_dict_t *
    )&tars_a2dp_globals,

};


MP_REGISTER_MODULE(
    MP_QSTR_tars_a2dp,
    tars_a2dp_user_cmodule
);
