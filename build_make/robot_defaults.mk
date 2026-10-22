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
################################################################################

APP_ROBOT  ?= 1
SMP        ?= 0
CONSOLE    ?= usb_cdc
WIFI       ?= cyw43
WIFI_JOIN  ?= 1
WIFI_NETIF ?= 1
WIFI_DHCP  ?= 1
