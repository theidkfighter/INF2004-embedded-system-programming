################################################################################
# IS-02 Robo-Wheeler application build (included at the end of the target
# makefile when APP_ROBOT=1, see robot_defaults.mk)
#
# Adds what the robot needs on top of the template's usb_cdc + Wi-Fi profile:
#   - Pico SDK hardware drivers: ADC, I2C (+ its timeout helper), PWM headers
#     (GPIO and PIO are already compiled by the Wi-Fi profile)
#   - lwIP's MQTT client (apps/mqtt), run in the radio-owner task
#   - the robot configuration flags described in README_ROBOT.md
################################################################################

ifeq ($(APP_ROBOT),1)

ifneq ($(SMP),0)
$(error The robot application is single-core (SMP=0): its critical sections use tk_dis_dsp() and interrupt masking, which are not SMP-safe)
endif
ifneq ($(CONSOLE),usb_cdc)
$(error The robot needs CONSOLE=usb_cdc: GP0/GP1 (UART0) are the right-wheel encoder)
endif
ifneq ($(WIFI_DHCP),1)
$(error The robot needs WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 for MQTT)
endif
ifneq ($(WIFI_UDP)$(WIFI_TCP)$(WIFI_TCPBULK),000)
ifneq ($(WIFI_UDP)$(WIFI_TCP)$(WIFI_TCPBULK),)
$(error WIFI_UDP/WIFI_TCP/WIFI_TCPBULK run the template's echo tests; leave them at 0 for the robot)
endif
endif
ifeq ($(wildcard ../config/mqtt_config.h),)
$(error The robot needs config/mqtt_config.h: copy it from config/mqtt_config.example.h and set the broker IP)
endif

EXE_FILE := is02_robot_smp$(SMP)

SDK_SRC := $(PICO_SDK_PATH)/src

ROBOT_SDK_SRCS := rp2_common/hardware_adc/adc.c \
                  rp2_common/hardware_i2c/i2c.c \
                  common/pico_time/timeout_helper.c
ROBOT_SDK_OBJS := $(addprefix ./mtkernel_3/sdk/,$(ROBOT_SDK_SRCS:.c=.o))
ROBOT_LWIP_OBJS := ./mtkernel_3/lwip/apps/mqtt/mqtt.o

CFLAGS += -DTM_APP_MQTT=1 -DTM_APP_NET_HOOK=1 \
          -DTM_CONSOLE_UART=0 -DBOARD_LED_PIN=19 -DUSE_SDEV_DRV=0 \
          -DCYW43_TASK_PRIORITY=9 \
          -I"$(SDK_SRC)/rp2_common/hardware_adc/include" \
          -I"$(SDK_SRC)/rp2_common/hardware_i2c/include" \
          -I"$(SDK_SRC)/rp2_common/hardware_pwm/include" \
          -I"$(SDK_SRC)/rp2_common/hardware_watchdog/include" \
          -I"$(SDK_SRC)/common/pico_time/include" \
          -I"$(SDK_SRC)/common/pico_sync/include" \
          -I"$(SDK_SRC)/common/pico_util/include"

# Stricter warnings for the application sources only (the kernel and the
# vendor libraries keep the template's flags).
mtkernel_3/app_program/%.o: CFLAGS += -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter

LIBS += -lm

OBJS += $(ROBOT_SDK_OBJS) $(ROBOT_LWIP_OBJS)
C_DEPS += $(ROBOT_SDK_OBJS:.o=.d) $(ROBOT_LWIP_OBJS:.o=.d)

mtkernel_3/sdk/%.o: $(SDK_SRC)/%.c
	@echo 'Building Pico SDK file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

endif
