/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CYW43_UTK_H
#define CYW43_UTK_H

#include <stdint.h>

#define CYW43_UTK_DNS_TEST_HOSTNAME "example.com"

typedef struct {
    volatile int32_t init_result;
    volatile uint32_t init_complete;
    volatile uint32_t ready;
    volatile uint32_t scan_active;
    volatile uint32_t scan_complete;
    volatile uint32_t scan_count;
    volatile int32_t strongest_rssi;
    volatile uint32_t poll_count;
    volatile uint32_t owner_prc;
    uint8_t mac[6];
    volatile uint32_t join_enabled;
    volatile uint32_t join_started;
    volatile uint32_t join_complete;
    volatile int32_t join_request_result;
    volatile int32_t join_result;
    volatile int32_t link_status;
    volatile uint32_t link_up;
    volatile uint32_t join_elapsed_ms;
    volatile int32_t link_rssi;
    uint8_t bssid[6];
    volatile uint32_t lwip_initialized;
    volatile uint32_t netif_initialized;
    volatile uint32_t netif_up;
    volatile uint32_t netif_link_up;
    volatile uint32_t netif_default;
    volatile uint32_t netif_callbacks_ready;
    volatile uint32_t netif_no_sys;
    volatile uint32_t netif_mtu;
    uint8_t netif_mac[6];
    volatile uint32_t dhcp_enabled;
    volatile uint32_t dhcp_started;
    volatile uint32_t dhcp_complete;
    volatile int32_t dhcp_result;
    volatile uint32_t dhcp_elapsed_ms;
    volatile uint32_t dhcp_tries;
    uint8_t dhcp_address[4];
    uint8_t dhcp_netmask[4];
    uint8_t dhcp_gateway[4];
    uint8_t dhcp_dns[4];
    volatile uint32_t dns_enabled;
    volatile uint32_t dns_started;
    volatile uint32_t dns_complete;
    volatile int32_t dns_request_result;
    volatile int32_t dns_result;
    volatile uint32_t dns_elapsed_ms;
    uint8_t dns_address[4];
    volatile int32_t ipv4_config_result;
    volatile uint32_t ipv4_configured;
    volatile uint32_t arp_resolved;
    volatile uint32_t ping_started;
    volatile uint32_t ping_complete;
    volatile int32_t ping_send_result;
    volatile int32_t ping_result;
    volatile uint32_t ping_attempts;
    volatile uint32_t ping_elapsed_ms;
    uint8_t ipv4_address[4];
    uint8_t ipv4_netmask[4];
    uint8_t ipv4_gateway[4];
    uint8_t ping_target[4];
    uint8_t arp_mac[6];
    /* Stamped by the reader, not the radio task: how many torn snapshots the
     * seqlock in cyw43_utk_get_status() has had to discard since boot. */
    uint32_t status_retries;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
void cyw43_utk_task(int32_t stacd, void *exinf);
void cyw43_utk_link_state_changed(int up);

#endif
