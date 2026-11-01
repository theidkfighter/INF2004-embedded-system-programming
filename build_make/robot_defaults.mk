################################################################################
# IS-02 Robo-Wheeler: default build profile
#
# Included by makefile *before* pico_rp2040.mk, so these become the defaults
# of a bare `make`. Each one can still be overridden on the command line.
#
#   SMP=0          single core (the template's qualified default)
#   CONSOLE=usb_cdc  console on the Pico's own USB port. The UART0 console
#                  is not possible: GP0/GP1 are the right-wheel encoder.
#   WIFI=cyw43 ... radio + lwIP + DHCP, needed for MQTT telemetry
#
# Robot application options (see robot.mk), all off by default:
#   APP_LEGACY_TEXT_CMD=1  also accept the old text commands on is02/cmd,
#                          is02/map and is02/config (replies stay binary)
#   APP_PERF=1             timing probes, encode benchmark, [perf] lines
#   APP_SELFTEST=1         on-target concurrency self-test at boot
################################################################################

APP_ROBOT  ?= 1
SMP        ?= 0
CONSOLE    ?= usb_cdc
WIFI       ?= cyw43
WIFI_JOIN  ?= 1
WIFI_NETIF ?= 1
WIFI_DHCP  ?= 1

APP_LEGACY_TEXT_CMD ?= 0
APP_PERF            ?= 0
APP_SELFTEST        ?= 0
