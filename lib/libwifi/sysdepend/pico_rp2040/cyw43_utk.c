/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Core-0-owned CYW43439 polling service for micro T-Kernel. */

#include "cyw43.h"
#include "cyw43_country.h"
#include "cyw43_utk.h"

#if TM_WIFI_NETIF
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#endif
#if TM_WIFI_DHCP
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#endif
#if TM_WIFI_DHCP
#include "lwip_utk_ipv4.h"
#endif

#define CYW43_POLL_MS        2
/* Bounded so a reader can never wedge if the writer is starved; an exhausted
 * budget returns the last copy and is visible through status_retries. */
#define CYW43_STATUS_READ_ATTEMPTS 16
#define CYW43_JOIN_TIMEOUT_US 30000000ULL
#define CYW43_DHCP_TIMEOUT_US 30000000ULL
#define CYW43_DNS_TIMEOUT_US  30000000ULL

#if TM_WIFI_JOIN
#include "wifi_credentials.h"

static const uint8_t wifi_ssid[] = UTK_WIFI_SSID;
static const uint8_t wifi_password[] = UTK_WIFI_PASSWORD;

_Static_assert(sizeof(wifi_ssid) > 1, "UTK_WIFI_SSID must not be empty");
_Static_assert(sizeof(wifi_ssid) - 1 <= 32, "UTK_WIFI_SSID exceeds 32 bytes");
_Static_assert(UTK_WIFI_AUTH == CYW43_AUTH_OPEN
            || (sizeof(wifi_password) - 1 >= 8
             && sizeof(wifi_password) - 1 <= 63),
               "WPA/WPA2/WPA3 password must contain 8 to 63 bytes");
#endif

extern volatile uint32_t cyw43_utk_poll_requested;
#if TM_APP_NET_HOOK
/* IS-02: application network work (MQTT, Wi-Fi rejoin) runs here, in the
 * radio-owner context, because lwIP is NO_SYS and owned by this task. */
extern void app_net_poll(void);
#endif
#include "smp_compat.h"
extern int32_t tk_dly_tsk(uint32_t delay);
extern void mp_memory_barrir(void);
extern uint64_t time_us_64(void);

/* Seqlock guarding radio_status.  The radio-owner task publishes field by
 * field on processor 1 while the qualification harness reads the whole
 * structure from the other processor, so an unguarded copy can mix fields
 * from two different updates.  Odd means a publication is in progress. */
static volatile uint32_t status_version;
static volatile uint32_t status_read_retries;

static T_CYW43_UTK_STATUS radio_status = {
    .init_result = -5,
    .strongest_rssi = -32768,
    .join_enabled = TM_WIFI_JOIN,
    .join_request_result = -5,
    .join_result = -5,
    .link_status = CYW43_LINK_DOWN,
    .link_rssi = -32768,
    .dhcp_enabled = TM_WIFI_DHCP,
    .dhcp_result = -5,
    .dns_enabled = TM_WIFI_DNS,
    .dns_request_result = -5,
    .dns_result = -3,
    .ipv4_config_result = -6,
    .ping_send_result = -6,
    .ping_result = -3,
};

static void status_publish_begin(void)
{
    status_version++;
    mp_memory_barrir();
}

static void status_publish_end(void)
{
    mp_memory_barrir();
    status_version++;
}

#if TM_WIFI_NETIF
static void update_netif_status(void)
{
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];

    radio_status.netif_initialized = (netif->state == &cyw43_state);
    radio_status.netif_up = netif_is_up(netif);
    radio_status.netif_link_up = netif_is_link_up(netif);
    radio_status.netif_default = (netif_default == netif);
    radio_status.netif_callbacks_ready =
            netif->input != NULL && netif->output != NULL
         && netif->linkoutput != NULL;
    radio_status.netif_no_sys = NO_SYS;
    radio_status.netif_mtu = netif->mtu;
    for(unsigned int i = 0; i < sizeof(radio_status.netif_mac); i++) {
        radio_status.netif_mac[i] = netif->hwaddr[i];
    }
    radio_status.link_up = radio_status.netif_link_up;
}
#endif

#if TM_WIFI_DNS
static void copy_ip4(uint8_t dst[4], const ip4_addr_t *src);
static uint64_t dns_started_us;

static void dns_found(const char *name, const ip_addr_t *address, void *arg)
{
    (void)name;
    (void)arg;
    if(radio_status.dns_complete) return;

    radio_status.dns_elapsed_ms =
            (uint32_t)((time_us_64() - dns_started_us) / 1000ULL);
    if(address == NULL) {
        radio_status.dns_result = ERR_VAL;
    } else {
        copy_ip4(radio_status.dns_address, ip_2_ip4(address));
        radio_status.dns_result = ERR_OK;
    }
    mp_memory_barrir();
    radio_status.dns_complete = 1;
}

static void update_dns_status(void)
{
    uint64_t now = time_us_64();

    if(!radio_status.dns_started && radio_status.dhcp_complete
       && radio_status.dhcp_result == ERR_OK) {
        ip_addr_t address;
        err_t result;

        radio_status.dns_started = 1;
        dns_started_us = now;
        result = dns_gethostbyname(CYW43_UTK_DNS_TEST_HOSTNAME, &address,
                                   dns_found, NULL);
        radio_status.dns_request_result = result;
        if(result == ERR_OK) {
            copy_ip4(radio_status.dns_address, ip_2_ip4(&address));
            radio_status.dns_result = ERR_OK;
            mp_memory_barrir();
            radio_status.dns_complete = 1;
        } else if(result != ERR_INPROGRESS) {
            radio_status.dns_result = result;
            mp_memory_barrir();
            radio_status.dns_complete = 1;
        }
    }

    if(radio_status.dns_started && !radio_status.dns_complete) {
        radio_status.dns_elapsed_ms =
                (uint32_t)((now - dns_started_us) / 1000ULL);
        if(now - dns_started_us >= CYW43_DNS_TIMEOUT_US) {
            radio_status.dns_result = ERR_TIMEOUT;
            mp_memory_barrir();
            radio_status.dns_complete = 1;
        }
    }
}
#endif


#if TM_WIFI_DHCP
static void update_ipv4_status(void)
{
    T_LWIP_UTK_IPV4_STATUS status;

    lwip_utk_ipv4_get_status(&status);
    radio_status.ipv4_config_result = status.config_result;
    radio_status.ipv4_configured = status.configured;
    radio_status.arp_resolved = status.arp_resolved;
    radio_status.ping_started = status.ping_started;
    radio_status.ping_complete = status.ping_complete;
    radio_status.ping_send_result = status.ping_send_result;
    radio_status.ping_result = status.ping_result;
    radio_status.ping_attempts = status.ping_attempts;
    radio_status.ping_elapsed_ms = status.ping_elapsed_ms;
    for(unsigned int i = 0; i < sizeof(status.address); i++) {
        radio_status.ipv4_address[i] = status.address[i];
        radio_status.ipv4_netmask[i] = status.netmask[i];
        radio_status.ipv4_gateway[i] = status.gateway[i];
        radio_status.ping_target[i] = status.target[i];
    }
    for(unsigned int i = 0; i < sizeof(status.arp_mac); i++) {
        radio_status.arp_mac[i] = status.arp_mac[i];
    }
}
#endif

#if TM_WIFI_DHCP
static uint64_t dhcp_started_us;
static uint32_t dhcp_ipv4_adopted;

static void copy_ip4(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

static void update_dhcp_status(void)
{
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
    struct dhcp *dhcp = netif_dhcp_data(netif);
    const ip_addr_t *dns = dns_getserver(0);
    uint64_t now = time_us_64();

    radio_status.dhcp_started = (dhcp != NULL);
    if(dhcp != NULL) radio_status.dhcp_tries = dhcp->tries;
    if(radio_status.netif_link_up && dhcp_started_us == 0) {
        dhcp_started_us = now;
    }
    if(dhcp_started_us != 0 && !radio_status.dhcp_complete) {
        radio_status.dhcp_elapsed_ms =
                (uint32_t)((now - dhcp_started_us) / 1000ULL);
    }

    if(!radio_status.dhcp_complete && dhcp_supplied_address(netif)) {
        copy_ip4(radio_status.dhcp_address, netif_ip4_addr(netif));
        copy_ip4(radio_status.dhcp_netmask, netif_ip4_netmask(netif));
        copy_ip4(radio_status.dhcp_gateway, netif_ip4_gw(netif));
        if(dns != NULL) {
            copy_ip4(radio_status.dhcp_dns, ip_2_ip4(dns));
        }
        if(!ip4_addr_isany_val(*netif_ip4_addr(netif))
           && !ip4_addr_isany_val(*netif_ip4_netmask(netif))
           && !ip4_addr_isany_val(*netif_ip4_gw(netif))
           && dns != NULL && !ip4_addr_isany_val(*ip_2_ip4(dns))) {
            if(!dhcp_ipv4_adopted) {
                radio_status.ipv4_config_result =
                        lwip_utk_ipv4_adopt_netif(netif);
                dhcp_ipv4_adopted = 1;
            }
            radio_status.dhcp_result = ERR_OK;
            mp_memory_barrir();
            radio_status.dhcp_complete = 1;
        }
    }

    if(!radio_status.dhcp_complete && dhcp_started_us != 0
       && now - dhcp_started_us >= CYW43_DHCP_TIMEOUT_US) {
        radio_status.dhcp_result = ERR_TIMEOUT;
        mp_memory_barrir();
        radio_status.dhcp_complete = 1;
    }
}
#endif

void cyw43_utk_link_state_changed(int up)
{
    radio_status.link_up = (up != 0);
    if(!up) radio_status.link_status = CYW43_LINK_DOWN;
    mp_memory_barrir();
}

static int scan_result(void *env, const cyw43_ev_scan_result_t *result)
{
    T_CYW43_UTK_STATUS *status = env;

    if(result == NULL) return 0;
    status->scan_count++;
    if(result->rssi > status->strongest_rssi) {
        status->strongest_rssi = result->rssi;
    }
    return 0;
}

void cyw43_utk_task(int32_t stacd, void *exinf)
{
    cyw43_wifi_scan_options_t options = {0};
    int32_t result;
    bool was_scanning;
#if TM_WIFI_JOIN
    uint64_t join_started_us = 0;
#endif

    (void)stacd;
    (void)exinf;
    radio_status.owner_prc = (uint32_t)CYW43_THIS_PRC();

#if TM_WIFI_NETIF
    lwip_init();
    radio_status.lwip_initialized = 1;
#endif
    cyw43_init(&cyw43_state);
    cyw43_thread_enter();
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true,
                      CYW43_COUNTRY_WORLDWIDE);
    if((cyw43_state.itf_state & (1u << CYW43_ITF_STA)) == 0) {
        /* cyw43_wifi_set_up() has a void API and otherwise hides its bus-init
         * error.  Report the underlying I/O class instead of letting scan()
         * replace it with the secondary "interface not up" (-4) result. */
        result = -CYW43_EIO;
    } else {
        (void)cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA,
                                 radio_status.mac);
        result = cyw43_wifi_scan(&cyw43_state, &options, &radio_status,
                                 scan_result);
    }
    cyw43_thread_exit();

#if TM_WIFI_NETIF
    update_netif_status();
#endif

    radio_status.init_result = result;
    radio_status.ready = (result == 0);
    radio_status.scan_active = (result == 0);
    mp_memory_barrir();
    radio_status.init_complete = 1;

    /* A failed bus has nothing useful to poll.  More importantly, repeatedly
     * waking a failed high-rate service must not interfere with USB or the
     * rest of the kernel qualification. */
    if(result != 0) {
        for(;;) (void)tk_dly_tsk(1000);
    }

    for(;;) {
        was_scanning = radio_status.scan_active != 0;
        cyw43_thread_enter();
        if(cyw43_poll != NULL) cyw43_poll();
        status_publish_begin();
#if TM_WIFI_NETIF
        sys_check_timeouts();
        update_netif_status();
#endif
#if TM_APP_NET_HOOK
        app_net_poll();
#endif
#if TM_WIFI_DHCP
        update_dhcp_status();
#endif
#if TM_WIFI_DNS
        update_dns_status();
#endif
#if TM_WIFI_DHCP
        lwip_utk_ipv4_poll(&cyw43_state.netif[CYW43_ITF_STA]);
        update_ipv4_status();
#endif
        radio_status.poll_count++;
        radio_status.scan_active = cyw43_wifi_scan_active(&cyw43_state);
        if(was_scanning && !radio_status.scan_active) {
            mp_memory_barrir();
            radio_status.scan_complete = 1;
#if TM_WIFI_JOIN
            radio_status.join_started = 1;
            radio_status.join_complete = 0;
            radio_status.link_up = 0;
            radio_status.link_status = CYW43_LINK_DOWN;
            join_started_us = time_us_64();
            result = cyw43_wifi_join(&cyw43_state,
                                     sizeof(wifi_ssid) - 1, wifi_ssid,
                                     sizeof(wifi_password) - 1, wifi_password,
                                     UTK_WIFI_AUTH, NULL,
                                     CYW43_CHANNEL_NONE);
            radio_status.join_request_result = result;
            if(result != 0) {
                radio_status.join_result = result;
                radio_status.join_complete = 1;
            }
#endif
        }
#if TM_WIFI_JOIN
        if(radio_status.join_started && !radio_status.join_complete) {
            uint64_t now = time_us_64();
            int32_t link = cyw43_wifi_link_status(&cyw43_state,
                                                  CYW43_ITF_STA);

            radio_status.join_elapsed_ms = (uint32_t)
                    ((now - join_started_us) / 1000ULL);
            radio_status.link_status = link;
            if(radio_status.link_up) {
                (void)cyw43_wifi_get_bssid(&cyw43_state,
                                           radio_status.bssid);
                (void)cyw43_wifi_get_rssi(&cyw43_state,
                                          (int32_t *)&radio_status.link_rssi);
                radio_status.join_result = 0;
                mp_memory_barrir();
                radio_status.join_complete = 1;
            } else if(link < CYW43_LINK_DOWN) {
                radio_status.join_result = link;
                mp_memory_barrir();
                radio_status.join_complete = 1;
            } else if(now - join_started_us >= CYW43_JOIN_TIMEOUT_US) {
                radio_status.join_result = -CYW43_ETIMEDOUT;
                mp_memory_barrir();
                radio_status.join_complete = 1;
            }
        }
#endif
        status_publish_end();
        cyw43_utk_poll_requested = 0;
        cyw43_thread_exit();
        (void)tk_dly_tsk(CYW43_POLL_MS);
    }
}

void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status)
{
    unsigned int attempt;

    if(status == NULL) return;
    /* Seqlock read.  Barriers alone cannot make a ~230-byte copy atomic, so
     * retry until the version is even (no publication in progress) and
     * unchanged across the copy.  Without this a caller can observe, say, a
     * completion flag from one update beside result fields from an earlier
     * one, and act on a state that never actually existed. */
    for(attempt = 0; attempt < CYW43_STATUS_READ_ATTEMPTS; attempt++) {
        uint32_t before = status_version;

        mp_memory_barrir();
        *status = radio_status;
        mp_memory_barrir();
        if((before & 1U) == 0U && before == status_version) break;
        status_read_retries++;
    }
    status->status_retries = status_read_retries;
}
