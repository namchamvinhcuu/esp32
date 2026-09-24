/* mb_tcp: minimal hand-rolled Modbus TCP master for the demo.
 * TODO(RS485): swap for the managed component espressif/esp-modbus when
 * RTU over RS485 is added — keep this file's start() surface. */
#ifndef FMS_MB_TCP_H
#define FMS_MB_TCP_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the poller task (core 1). Reads every BUS_MBTCP channel from cfg
 * on its own period_ms; failures push Q_COMM_ERR measurements. */
esp_err_t mb_tcp_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_MB_TCP_H */
