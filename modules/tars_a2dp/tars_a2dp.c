#include "py/runtime.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_log.h"

#define TARS_TAG "TARS_A2DP"
#define TARGET_NAME "I7-TWS"

static bool tars_bt_started = false;
static bool tars_device_found = false;

static uint8_t tars_target_bda[ESP_BD_ADDR_LEN] = {0};


/* ==================================================
   A2DP EVENT CALLBACK
   ================================================== */

static void tars_a2dp_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
) {
    switch (event) {

        case ESP_A2D_CONNECTION_STATE_EVT:

            ESP_LOGI(
                TARS_TAG,
                "A2DP CONNECTION STATE: %d",
                param->conn_stat.state
            );

            break;


        case ESP_A2D_AUDIO_STATE_EVT:

            ESP_LOGI(
                TARS_TAG,
                "A2DP AUDIO STATE: %d",
                param->audio_stat.state
            );

            break;


        default:

            break;

    }
}


/* ==================================================
   BLUETOOTH GAP CALLBACK
   ================================================== */

static void tars_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param
) {

    if (event == ESP_BT_GAP_DISC_RES_EVT) {

        uint8_t *eir = NULL;

        char device_name[
            ESP_BT_GAP_MAX_BDNAME_LEN + 1
        ];

        memset(
            device_name,
            0,
            sizeof(device_name)
        );


        for (
            int i = 0;
            i < param->disc_res.num_prop;
            i++
        ) {

            if (
                param->disc_res.prop[i].type ==
                ESP_BT_GAP_DEV_PROP_EIR
            ) {

                eir =
                    (uint8_t *)
                    param->disc_res.prop[i].val;

            }


            if (
                param->disc_res.prop[i].type ==
                ESP_BT_GAP_DEV_PROP_BDNAME
            ) {

                int len =
                    param->disc_res.prop[i].len;


                if (
                    len >
                    ESP_BT_GAP_MAX_BDNAME_LEN
                ) {

                    len =
                        ESP_BT_GAP_MAX_BDNAME_LEN;

                }


                memcpy(
                    device_name,
                    param->disc_res.prop[i].val,
                    len
                );


                device_name[len] = '\0';

            }

        }


        /* GET DEVICE NAME FROM EIR */

        if (
            strlen(device_name) == 0 &&
            eir != NULL
        ) {

            uint8_t eir_len = 0;


            uint8_t *eir_name =
                esp_bt_gap_resolve_eir_data(
                    eir,
                    ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,
                    &eir_len
                );


            if (
                eir_name != NULL &&
                eir_len > 0
            ) {

                if (
                    eir_len >
                    ESP_BT_GAP_MAX_BDNAME_LEN
                ) {

                    eir_len =
                        ESP_BT_GAP_MAX_BDNAME_LEN;

                }


                memcpy(
                    device_name,
                    eir_name,
                    eir_len
                );


                device_name[eir_len] = '\0';

            }

        }


        /* PRINT FOUND DEVICE */

        if (
            strlen(device_name) > 0
        ) {

            ESP_LOGI(
                TARS_TAG,
                "FOUND: %s [%02X:%02X:%02X:%02X:%02X:%02X]",

                device_name,

                param->disc_res.bda[0],
                param->disc_res.bda[1],
                param->disc_res.bda[2],
                param->disc_res.bda[3],
                param->disc_res.bda[4],
                param->disc_res.bda[5]
            );

        }


        /* FIND TARGET HEADSET */

        if (
            strcmp(
                device_name,
                TARGET_NAME
            ) == 0
        ) {

            if (
                !tars_device_found
            ) {

                memcpy(
                    tars_target_bda,
                    param->disc_res.bda,
                    ESP_BD_ADDR_LEN
                );


                tars_device_found = true;


                ESP_LOGI(
                    TARS_TAG,
                    "TARGET FOUND: %s",
                    TARGET_NAME
                );

            }

        }

    }


    /* DISCOVERY FINISHED */

    if (
        event ==
        ESP_BT_GAP_DISC_STATE_CHANGED_EVT
    ) {

        if (
            param->disc_st_chg.state ==
            ESP_BT_GAP_DISCOVERY_STOPPED
        ) {

            ESP_LOGI(
                TARS_TAG,
                "BLUETOOTH SCAN FINISHED"
            );

        }

    }

}


/* ==================================================
   START BLUETOOTH CLASSIC + A2DP
   ================================================== */

static mp_obj_t tars_a2dp_start(void) {

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
        esp_a2d_register_callback(
            tars_a2dp_callback
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
        esp_bt_gap_set_device_name(
            "TARS"
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
            ESP_BT_GENERAL_DISCOVERABLE
        );


    if (
        ret != ESP_OK
    ) {

        return mp_obj_new_str(
            "ERROR: SET DISCOVERABLE FAILED",

            strlen(
                "ERROR: SET DISCOVERABLE FAILED"
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


    tars_bt_started = true;


    return mp_obj_new_str(
        "TARS BLUETOOTH CLASSIC A2DP READY",

        strlen(
            "TARS BLUETOOTH CLASSIC A2DP READY"
        )
    );

}


/* ==================================================
   SCAN FOR I7-TWS
   ================================================== */

static mp_obj_t tars_a2dp_scan(void) {

    esp_err_t ret;


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


    tars_device_found = false;


    memset(
        tars_target_bda,
        0,
        ESP_BD_ADDR_LEN
    );


    ret =
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


    return mp_obj_new_str(
        "TARS SCANNING FOR I7-TWS...",

        strlen(
            "TARS SCANNING FOR I7-TWS..."
        )
    );

}


/* ==================================================
   CHECK FOUND HEADSET
   ================================================== */

static mp_obj_t tars_a2dp_found(void) {

    if (
        !tars_device_found
    ) {

        return mp_obj_new_str(
            "I7-TWS NOT FOUND",

            strlen(
                "I7-TWS NOT FOUND"
            )
        );

    }


    char result[100];


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


/* ==================================================
   CONNECT TO FOUND HEADSET
   ================================================== */

static mp_obj_t tars_a2dp_connect(void) {

    esp_err_t ret;


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
            "ERROR: I7-TWS NOT FOUND - RUN SCAN FIRST",

            strlen(
                "ERROR: I7-TWS NOT FOUND - RUN SCAN FIRST"
            )
        );

    }


    ret =
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


    return mp_obj_new_str(
        "TARS CONNECTING TO I7-TWS...",

        strlen(
            "TARS CONNECTING TO I7-TWS..."
        )
    );

}


/* ==================================================
   STATUS
   ================================================== */

static mp_obj_t tars_a2dp_test(void) {

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


/* ==================================================
   MICROPYTHON FUNCTION OBJECTS
   ================================================== */

static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_start_obj,
    tars_a2dp_start
);


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_scan_obj,
    tars_a2dp_scan
);


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_found_obj,
    tars_a2dp_found
);


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_connect_obj,
    tars_a2dp_connect
);


static MP_DEFINE_CONST_FUN_OBJ_0(
    tars_a2dp_test_obj,
    tars_a2dp_test
);


/* ==================================================
   MICROPYTHON MODULE
   ================================================== */

static const mp_rom_map_elem_t
tars_a2dp_globals_table[] = {

    {
        MP_ROM_QSTR(MP_QSTR___name__),
        MP_ROM_QSTR(MP_QSTR_tars_a2dp)
    },

    {
        MP_ROM_QSTR(MP_QSTR_test),
        MP_ROM_PTR(&tars_a2dp_test_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_start),
        MP_ROM_PTR(&tars_a2dp_start_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_scan),
        MP_ROM_PTR(&tars_a2dp_scan_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_found),
        MP_ROM_PTR(&tars_a2dp_found_obj)
    },

    {
        MP_ROM_QSTR(MP_QSTR_connect),
        MP_ROM_PTR(&tars_a2dp_connect_obj)
    },

};


static MP_DEFINE_CONST_DICT(
    tars_a2dp_globals,
    tars_a2dp_globals_table
);


const mp_obj_module_t tars_a2dp_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&tars_a2dp_globals,
};


/* HARUS SATU BARIS */

MP_REGISTER_MODULE(MP_QSTR_tars_a2dp, tars_a2dp_user_cmodule);
