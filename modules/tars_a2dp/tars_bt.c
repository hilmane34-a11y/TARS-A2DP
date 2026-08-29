#include "tars_bt.h"

#include "esp_err.h"
#include "esp_bt.h"


static bool tars_bt_ready = false;


bool tars_bt_is_ready(void) {

    return tars_bt_ready;
}


int tars_bt_init(void) {

    if (tars_bt_ready) {

        return 0;
    }


    esp_err_t ret;


    // Lepaskan memori BLE.
    // TARS akan menggunakan Bluetooth Classic.
    ret = esp_bt_controller_mem_release(
        ESP_BT_MODE_BLE
    );

    // Jika gagal karena sudah pernah dilepas,
    // lanjutkan proses.
    (void)ret;


    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ret = esp_bt_controller_init(
        &bt_cfg
    );

    if (ret != ESP_OK) {

        return -1;
    }


    ret = esp_bt_controller_enable(
        ESP_BT_MODE_CLASSIC_BT
    );

    if (ret != ESP_OK) {

        return -2;
    }


    tars_bt_ready = true;


    return 0;
}
