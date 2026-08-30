#ifndef TARS_BLUETOOTH_H
#define TARS_BLUETOOTH_H

#include <stdbool.h>

#include "esp_err.h"


/* Menyalakan Bluetooth Classic + Bluedroid */

esp_err_t tars_bluetooth_start(void);


/* Status Bluetooth */

bool tars_bluetooth_ready(void);


/* Mengatur nama perangkat */

esp_err_t tars_bluetooth_set_name(
    const char *name
);

#endif
