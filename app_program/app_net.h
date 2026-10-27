/**
 * @file    app_net.h
 * @brief   IS-02 Wi-Fi + MQTT link (DR 6.4), plain C interface.
 *
 * The MQTT client (lwIP apps/mqtt) runs inside the template's radio-owner
 * task (cyw43_utk.c calls app_net_poll() every poll), because lwIP is
 * NO_SYS and owned by that task. Other tasks only queue messages here:
 *
 *   is02/telemetry  JSON frame, 2 Hz, kept in a 64-frame (~32 s) ring while
 *                   offline; when the ring is full the oldest frame is
 *                   overwritten and counted as dropped (COMMS_DEGRADED)
 *   is02/status     heartbeat, 1 Hz
 *   is02/event      events (barcode, hump, obstacle, state changes, faults)
 *   is02/ack        replies to commands
 *   is02/cmd        commands to the robot (subscribed)
 *   is02/config     retained calibration / parameters, "NAME=VALUE" lines
 *                   (subscribed; the broker re-sends it at every connect)
 *
 * Broker discovery: MQTT_BROKER_IP is tried first. After 2 failed connects
 * the robot follows the UDP beacon ("IS02_BROKER") that the laptop monitor
 * broadcasts on port 47002, so a changed laptop IP needs no rebuild.
 *
 * Recovery (NFR7, FR20): Wi-Fi rejoin and MQTT reconnect with back-off
 * 1 -> 2 -> 4 -> 8 -> 16 s. Navigation never waits for the network.
 */
#ifndef APP_NET_H
#define APP_NET_H

#include <stdbool.h>
#include <stdint.h>

#define NET_TOPIC_TELEMETRY  "is02/telemetry"
#define NET_TOPIC_STATUS     "is02/status"
#define NET_TOPIC_EVENT      "is02/event"
#define NET_TOPIC_ACK        "is02/ack"
#define NET_TOPIC_CMD        "is02/cmd"
#define NET_TOPIC_CONFIG     "is02/config"      /* retained "NAME=VALUE" lines */
#define NET_TOPIC_MAP        "is02/map"         /* retained barcode map, e.g. "F=FWD,K=LEFT,R=RIGHT,U=UTURN" */
#define NET_BEACON_PORT      (47002u)           /* UDP broker beacon from the laptop tool */

#define NET_FRAME_MAX        (512u)     /* one telemetry frame (~300-450 bytes) */
#define NET_MSG_MAX          (800u)     /* event / ack / status (GET:PARAMS is the longest) */
#define NET_FRAME_SLOTS      (64u)
#define NET_MSG_SLOTS        (8u)

typedef enum
{
    NET_T_STATUS = 0,
    NET_T_EVENT,
    NET_T_ACK
} net_topic_t;

/* Producers (any task) */
bool     net_post_frame(const char *p_json, uint32_t len);      /* buffered while offline */
bool     net_post(net_topic_t topic, const char *p_json);       /* status is dropped while offline */
void     net_set_led(bool b_on);                                /* Pico W on-board LED */

/* Status */
bool     net_wifi_up(void);
bool     net_mqtt_up(void);
int32_t  net_rssi(void);
uint32_t net_frames_waiting(void);
uint32_t net_frames_dropped(void);
uint32_t net_reconnects(void);
uint32_t net_published(void);
uint32_t net_publish_errors(void);
int32_t  net_last_connect_status(void);     /* lwIP mqtt_connection_status_t, -1 = none yet */
const char *net_broker_ip(void);            /* broker in use (config or discovered) */

/* Called by the template's radio-owner task (TM_APP_NET_HOOK) */
void     app_net_poll(void);

/* Implemented by the application (tsk_telemetry.c): a command arrived on
   is02/cmd. Runs in the radio-owner task: must not block. */
void     app_net_command(const char *p_cmd, uint64_t rx_us);

/* Implemented by the application: the retained is02/config message arrived
   (on every connect). Runs in the radio-owner task: must not block. */
void     app_net_config(const char *p_text);

#endif /* APP_NET_H */
