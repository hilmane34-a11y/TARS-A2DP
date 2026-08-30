#include "py/runtime.h"


/* =========================================================
   TARS BLUETOOTH MODULE
   ========================================================= */

static mp_obj_t tars_bluetooth_test(void) {
    return mp_obj_new_str(
        "TARS BLUETOOTH READY",
        20
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_bluetooth_test_obj,
    tars_bluetooth_test
);


/* =========================================================
   MODULE TABLE
   ========================================================= */

static const mp_rom_map_elem_t
tars_bluetooth_globals_table[] = {

    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_bluetooth)
    },

    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(&tars_bluetooth_test_obj)
    },

};

static MP_DEFINE_CONST_DICT(
    tars_bluetooth_globals,
    tars_bluetooth_globals_table
);


/* =========================================================
   MODULE DEFINITION
   ========================================================= */

const mp_obj_module_t
tars_bluetooth_user_cmodule = {

    .base = {
        &mp_type_module
    },

    .globals =
        (mp_obj_dict_t *)&tars_bluetooth_globals,

};


/* =========================================================
   REGISTER MODULE
   ========================================================= */

MP_REGISTER_MODULE(
    MP_QSTR_tars_bluetooth,
    tars_bluetooth_user_cmodule
);
