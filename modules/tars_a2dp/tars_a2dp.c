#include "py/runtime.h"

static mp_obj_t tars_a2dp_test(void) {
    return mp_obj_new_str(
        "TARS A2DP MODULE READY",
        strlen("TARS A2DP MODULE READY")
    );
}

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);

static const mp_rom_map_elem_t tars_a2dp_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_tars_a2dp) },
    { MP_ROM_QSTR(MP_QSTR_test), MP_ROM_PTR(&tars_a2dp_test_obj) },
};

static MP_DEFINE_CONST_DICT(
    tars_a2dp_module_globals,
    tars_a2dp_module_globals_table
);

const mp_obj_module_t tars_a2dp_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&tars_a2dp_module_globals,
};

MP_REGISTER_MODULE(
    MP_QSTR_tars_a2dp,
    tars_a2dp_user_cmodule
);
