/**
 * @file    app_net.c
 * @brief   IS-02 Wi-Fi + MQTT link (see app_net.h). Pico SDK / lwIP side.
 */
#include <string.h>

#include "pico.h"
#include "hardware/sync.h"
#include "cyw43.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"
#include "lwip/udp.h"

#include "app_net.h"
#include "hal.h"
#include "mqtt_config.h"
#include "wifi_credentials.h"

#define BOOT_GRACE_MS        (15000u)   /* the template scans + joins once at boot */
#define JOIN_WAIT_MS         (15000u)   /* give a join this long before retrying */
#define BACKOFF_MAX_MS       (16000u)
#define CONNECT_TIMEOUT_MS   (5000u)
#define RSSI_EVERY_MS        (2000u)
#define PUBLISH_PER_POLL     (4u)
#define IN_MAX               (1024u)    /* incoming command / config payload */
#define DISCOVERY_FAILS      (2u)       /* failed connects before a beacon may move the broker */

typedef struct
{
    uint16_t len;
    char     data[NET_FRAME_MAX];
} frame_slot_t;

typedef struct
{
    uint8_t  topic;
    uint16_t len;
    char     data[NET_MSG_MAX];
} msg_slot_t;

static frame_slot_t   g_frames[NET_FRAME_SLOTS];
static uint32_t       g_f_head;          /* producers */
static uint32_t       g_f_tail;          /* radio task */
static msg_slot_t     g_msgs[NET_MSG_SLOTS];
static uint32_t       g_m_head;
static uint32_t       g_m_tail;

static volatile bool     g_wifi_up;
static volatile bool     g_mqtt_up;
static volatile bool     g_led;
static bool              g_led_applied;
static volatile int32_t  g_rssi = -100;
static volatile uint32_t g_dropped;
static volatile uint32_t g_reconnects;
static volatile uint32_t g_published;
static volatile uint32_t g_pub_errors;
static volatile int32_t  g_last_status = -1;

static mqtt_client_t    *gp_client;
static struct mqtt_connect_client_info_t g_ci;
static bool              g_connecting;
static bool              g_ever_connected;
static uint32_t          g_t_connect;
static uint32_t          g_next_try;
static uint32_t          g_backoff = 1000u;
static uint32_t          g_t_join;
static uint32_t          g_t_rssi;
static bool              g_join_pending;
static uint32_t          g_join_backoff = 1000u;
static uint32_t          g_t_next_join;
static char              g_in_topic[32];
static char              g_in[IN_MAX];
static uint32_t          g_in_len;
static ip_addr_t         g_broker;
static bool              g_broker_set;
static char              g_broker_str[16];
static uint32_t          g_connect_fails;
static struct udp_pcb   *gp_beacon;
static char              g_tx_frame[NET_FRAME_MAX];

/* ------------------------------------------------------------------------ */
/* Producers                                                                 */
/* ------------------------------------------------------------------------ */

bool net_post_frame(const char *p_json, uint32_t len)
{
    uint32_t irq;
    uint32_t slot;

    if (len >= NET_FRAME_MAX)
    {
        return false;
    }
    irq = save_and_disable_interrupts();
    if ((g_f_head - g_f_tail) >= NET_FRAME_SLOTS)
    {
        g_f_tail++;                       /* full: overwrite the oldest frame */
        g_dropped++;
    }
    slot = g_f_head % NET_FRAME_SLOTS;
    memcpy(g_frames[slot].data, p_json, len);
    g_frames[slot].len = (uint16_t)len;
    g_f_head++;
    restore_interrupts(irq);
    return true;
}

bool net_post(net_topic_t topic, const char *p_json)
{
    uint32_t len = (uint32_t)strlen(p_json);
    uint32_t irq;
    uint32_t slot;

    if ((len >= NET_MSG_MAX) || ((topic == NET_T_STATUS) && !g_mqtt_up))
    {
        return false;
    }
    irq = save_and_disable_interrupts();
    if ((g_m_head - g_m_tail) >= NET_MSG_SLOTS)
    {
        g_m_tail++;                       /* keep the newest */
    }
    slot = g_m_head % NET_MSG_SLOTS;
    memcpy(g_msgs[slot].data, p_json, len);
    g_msgs[slot].len   = (uint16_t)len;
    g_msgs[slot].topic = (uint8_t)topic;
    g_m_head++;
    restore_interrupts(irq);
    return true;
}

void net_set_led(bool b_on)
{
    g_led = b_on;
}

bool     net_wifi_up(void)             { return g_wifi_up; }
bool     net_mqtt_up(void)             { return g_mqtt_up; }
int32_t  net_rssi(void)                { return g_rssi; }
uint32_t net_frames_waiting(void)      { return g_f_head - g_f_tail; }
uint32_t net_frames_dropped(void)      { return g_dropped; }
uint32_t net_reconnects(void)          { return g_reconnects; }
uint32_t net_published(void)           { return g_published; }
uint32_t net_publish_errors(void)      { return g_pub_errors; }
int32_t  net_last_connect_status(void) { return g_last_status; }
const char *net_broker_ip(void)        { return (g_broker_str[0] != '\0') ? g_broker_str : MQTT_BROKER_IP; }

/* ------------------------------------------------------------------------ */
/* MQTT callbacks (radio-owner task context)                                 */
/* ------------------------------------------------------------------------ */

static void schedule_retry(uint32_t now);

static void on_pub_done(void *p_arg, err_t err)
{
    (void)p_arg;
    if (err == ERR_OK)
    {
        g_published++;
    }
    else
    {
        g_pub_errors++;
    }
}

static void on_sub_done(void *p_arg, err_t err)
{
    (void)p_arg;
    (void)err;
}

static void on_in_publish(void *p_arg, const char *p_topic, u32_t tot_len)
{
    (void)p_arg;
    (void)tot_len;
    strncpy(g_in_topic, p_topic, sizeof(g_in_topic) - 1u);
    g_in_topic[sizeof(g_in_topic) - 1u] = '\0';
    g_in_len = 0u;
}

static void on_in_data(void *p_arg, const u8_t *p_data, u16_t len, u8_t flags)
{
    uint64_t rx_us = hal_time_us();
    uint32_t room = (IN_MAX - 1u) - g_in_len;
    uint32_t n = (len < room) ? len : room;

    (void)p_arg;
    /* a payload can arrive in several pieces: collect until the last one */
    memcpy(&g_in[g_in_len], p_data, n);
    g_in_len += n;
    g_in[g_in_len] = '\0';
    if ((flags & MQTT_DATA_FLAG_LAST) == 0u)
    {
        return;
    }
    while ((g_in_len > 0u) && ((g_in[g_in_len - 1u] == '\r') || (g_in[g_in_len - 1u] == '\n') ||
                               (g_in[g_in_len - 1u] == ' ')))
    {
        g_in_len--;
        g_in[g_in_len] = '\0';
    }
    if (strcmp(g_in_topic, NET_TOPIC_CMD) == 0)
    {
        app_net_command(g_in, rx_us);
    }
    else if (strcmp(g_in_topic, NET_TOPIC_CONFIG) == 0)
    {
        app_net_config(g_in);
    }
    else if (strcmp(g_in_topic, NET_TOPIC_MAP) == 0)
    {
        /* retained map: handled exactly like "MAP:<payload>" on is02/cmd */
        static char s_map_cmd[100];

        if (g_in_len > 0u)
        {
            memcpy(s_map_cmd, "MAP:", 4u);
            strncpy(&s_map_cmd[4], g_in, sizeof(s_map_cmd) - 5u);
            s_map_cmd[sizeof(s_map_cmd) - 1u] = '\0';
            app_net_command(s_map_cmd, rx_us);
        }
    }
    else
    {
        /* not ours */
    }
    g_in_len = 0u;
}

/* Broker discovery: the laptop tool broadcasts "IS02_BROKER <port>" on UDP
   NET_BEACON_PORT. If the configured broker keeps failing, the robot follows
   the beacon's address, so a new laptop IP does not need a rebuild. */
static void on_beacon(void *p_arg, struct udp_pcb *p_pcb, struct pbuf *p, const ip_addr_t *p_addr, u16_t port)
{
    char text[24];
    u16_t n;

    (void)p_arg;
    (void)p_pcb;
    (void)port;
    n = pbuf_copy_partial(p, text, sizeof(text) - 1u, 0u);
    text[n] = '\0';
    pbuf_free(p);
    if ((strncmp(text, "IS02_BROKER", 11) != 0) || g_mqtt_up || (g_connect_fails < DISCOVERY_FAILS))
    {
        return;
    }
    if (!ip_addr_cmp(p_addr, &g_broker))
    {
        ip_addr_copy(g_broker, *p_addr);
        strncpy(g_broker_str, ipaddr_ntoa(&g_broker), sizeof(g_broker_str) - 1u);
        g_connect_fails = 0u;
        g_backoff       = 1000u;
        g_next_try      = hal_time_ms();
    }
}

static void on_connect(mqtt_client_t *p_client, void *p_arg, mqtt_connection_status_t status)
{
    (void)p_arg;
    g_connecting  = false;
    g_last_status = (int32_t)status;
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        if (g_ever_connected)
        {
            g_reconnects++;
        }
        g_ever_connected = true;
        g_mqtt_up        = true;
        g_backoff        = 1000u;
        g_connect_fails  = 0u;
        (void)mqtt_subscribe(p_client, NET_TOPIC_CMD, 1u, on_sub_done, NULL);
        (void)mqtt_subscribe(p_client, NET_TOPIC_CONFIG, 1u, on_sub_done, NULL);   /* retained calibration */
        (void)mqtt_subscribe(p_client, NET_TOPIC_MAP, 1u, on_sub_done, NULL);      /* retained barcode map */
    }
    else
    {
        /* refused, timed out or disconnected: try again after the back-off */
        if (!g_mqtt_up)
        {
            g_connect_fails++;
        }
        g_mqtt_up = false;
        schedule_retry(hal_time_ms());
    }
}

/* ------------------------------------------------------------------------ */
/* Poll (radio-owner task)                                                   */
/* ------------------------------------------------------------------------ */

static void schedule_retry(uint32_t now)
{
    g_next_try = now + g_backoff;
    g_backoff  = (g_backoff >= BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : (g_backoff * 2u);
}

static void manage_wifi(uint32_t now)
{
    int32_t link = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
    struct netif *p_nif = &cyw43_state.netif[CYW43_ITF_STA];

    g_wifi_up = netif_is_up(p_nif) && netif_is_link_up(p_nif) &&
                !ip4_addr_isany_val(*netif_ip4_addr(p_nif));

    if (link == CYW43_LINK_JOIN)
    {
        g_join_pending = false;
        g_join_backoff = 1000u;
        return;
    }
    if (now < BOOT_GRACE_MS)
    {
        return;                               /* the template's first join */
    }
    if (g_join_pending && ((now - g_t_join) < JOIN_WAIT_MS))
    {
        return;
    }
    if ((int32_t)(now - g_t_next_join) < 0)
    {
        return;
    }
    /* link down, failed or never joined: try again (NFR7) */
    g_join_pending = true;
    g_t_join       = now;
    (void)cyw43_wifi_join(&cyw43_state, sizeof(UTK_WIFI_SSID) - 1u, (const uint8_t *)UTK_WIFI_SSID,
                          sizeof(UTK_WIFI_PASSWORD) - 1u, (const uint8_t *)UTK_WIFI_PASSWORD,
                          UTK_WIFI_AUTH, NULL, CYW43_CHANNEL_NONE);
    g_t_next_join  = now + JOIN_WAIT_MS + g_join_backoff;
    g_join_backoff = (g_join_backoff >= BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : (g_join_backoff * 2u);
}

static void manage_mqtt(uint32_t now)
{
    if (!g_broker_set)
    {
        g_broker_set = ipaddr_aton(MQTT_BROKER_IP, &g_broker) != 0;
        strncpy(g_broker_str, MQTT_BROKER_IP, sizeof(g_broker_str) - 1u);
    }
    if (gp_beacon == NULL)
    {
        gp_beacon = udp_new();
        if (gp_beacon != NULL)
        {
            (void)udp_bind(gp_beacon, IP_ANY_TYPE, NET_BEACON_PORT);
            udp_recv(gp_beacon, on_beacon, NULL);
        }
    }
    if (gp_client == NULL)
    {
        gp_client = mqtt_client_new();
        if (gp_client == NULL)
        {
            return;
        }
        mqtt_set_inpub_callback(gp_client, on_in_publish, on_in_data, NULL);
        memset(&g_ci, 0, sizeof(g_ci));
        g_ci.client_id  = MQTT_CLIENT_ID;
        g_ci.keep_alive = 5u;                 /* the broker notices a dead robot in ~7 s */
    }

    if (g_mqtt_up && !mqtt_client_is_connected(gp_client))
    {
        g_mqtt_up = false;                    /* connection lost without a callback */
        schedule_retry(now);
    }
    if (!g_wifi_up)
    {
        if (g_mqtt_up || g_connecting)
        {
            mqtt_disconnect(gp_client);       /* drop the dead TCP session now */
            g_mqtt_up    = false;
            g_connecting = false;
        }
        return;
    }
    if (g_connecting && ((now - g_t_connect) > CONNECT_TIMEOUT_MS))
    {
        mqtt_disconnect(gp_client);
        g_connecting = false;
        g_connect_fails++;
        schedule_retry(now);
    }
    if (!g_mqtt_up && !g_connecting && ((int32_t)(now - g_next_try) >= 0))
    {
        if (g_broker_set &&
            (mqtt_client_connect(gp_client, &g_broker, MQTT_BROKER_PORT, on_connect, NULL, &g_ci) == ERR_OK))
        {
            g_connecting = true;
            g_t_connect  = now;
        }
        else
        {
            schedule_retry(now);
        }
    }
}

static const char *topic_name(uint8_t t)
{
    return (t == (uint8_t)NET_T_STATUS) ? NET_TOPIC_STATUS
         : (t == (uint8_t)NET_T_EVENT)  ? NET_TOPIC_EVENT : NET_TOPIC_ACK;
}

static void flush(void)
{
    uint32_t sent = 0u;

    /* events and acks first (small, time-critical), then telemetry frames */
    while ((sent < PUBLISH_PER_POLL) && (g_m_tail != g_m_head))
    {
        msg_slot_t *p = &g_msgs[g_m_tail % NET_MSG_SLOTS];

        if (mqtt_publish(gp_client, topic_name(p->topic), p->data, p->len, 0u, 0u,
                         on_pub_done, NULL) != ERR_OK)
        {
            return;                           /* output buffer full: next poll */
        }
        g_m_tail++;
        sent++;
    }
    while ((sent < PUBLISH_PER_POLL) && (g_f_tail != g_f_head))
    {
        frame_slot_t *p;
        uint32_t      irq = save_and_disable_interrupts();
        uint16_t      len;

        /* copy out under the lock: a producer may overwrite the oldest slot */
        p   = &g_frames[g_f_tail % NET_FRAME_SLOTS];
        len = p->len;
        memcpy(g_tx_frame, p->data, len);
        restore_interrupts(irq);

        if (mqtt_publish(gp_client, NET_TOPIC_TELEMETRY, g_tx_frame, len, 0u, 0u,
                         on_pub_done, NULL) != ERR_OK)
        {
            return;
        }
        irq = save_and_disable_interrupts();
        if ((g_f_head - g_f_tail) > 0u)
        {
            g_f_tail++;
        }
        restore_interrupts(irq);
        sent++;
    }
}

void app_net_poll(void)
{
    uint32_t now = hal_time_ms();

    if (g_led != g_led_applied)
    {
        cyw43_gpio_set(&cyw43_state, 0, g_led);   /* WL_GPIO0 = on-board LED */
        g_led_applied = g_led;
    }

    manage_wifi(now);
    if (g_wifi_up && ((now - g_t_rssi) >= RSSI_EVERY_MS))
    {
        int32_t rssi = 0;

        g_t_rssi = now;
        if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0)
        {
            g_rssi = rssi;
        }
    }
    manage_mqtt(now);
    if (g_mqtt_up)
    {
        flush();
    }
}
