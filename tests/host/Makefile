################################################################################
# IS-02 Host_Test_Suite: host (PC) build of the firmware's plain C modules
#
# Builds build/libis02.so for the pytest/Hypothesis tests (loaded with ctypes
# by py/conftest.py). Run from WSL:
#
#     make -C tests/host            # build/libis02.so
#     make -C tests/host base       # build/libbase.so (Baseline_Build, base_ prefix)
#     make -C tests/host clean
#
# All paths are relative to this directory on purpose: the repository path
# contains spaces, which make cannot handle in targets or prerequisites.
################################################################################

HOST_CC  ?= gcc
APP      := ../../app_program
BUILD    := build
OBJ      := $(BUILD)/obj

# Firmware build switches (build_make/robot_defaults.mk), same defaults.
APP_LEGACY_TEXT_CMD ?= 0
APP_PERF            ?= 0
APP_SELFTEST        ?= 0

# Firmware application warning flags (build_make/robot.mk) plus -Werror.
WARN     := -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter -Werror

HOST_CFLAGS := -std=gnu11 -fPIC -O2 -g $(WARN) \
               -DAPP_LEGACY_TEXT_CMD=$(APP_LEGACY_TEXT_CMD) \
               -DAPP_PERF=$(APP_PERF) -DAPP_SELFTEST=$(APP_SELFTEST) \
               -I$(APP) -Istubs
HOST_LDFLAGS := -shared
HOST_LIBS    := -lm

# ---- source lists -----------------------------------------------------------
# Firmware sources (names in app_program/) linked into libis02.so. Later tasks
# append to this list with  LIB_SRCS += ...  (bitpack users, wire.c,
# msg_codec.c, evt.c, app_status.c, app_params.c, ...).
LIB_SRCS := line_logic.c imu_logic.c obstacle_logic.c barcode_logic.c \
            bc_detect.c bc_multi.c code39.c nav_map.c speed_ctl.c

# Host replacements for the kernel and SDK side (names in stubs/).
STUB_SRCS := crit_stub.c sat_stub.c os_stub.c
# app_program/bitpack.h is header-only (static inline): bitpack_export.c gives
# its helpers out-of-line bpx_* symbols for ctypes (task 3.1).
STUB_SRCS += bitpack_export.c
# Wire_Codec (task 3.9); wire_export.c gives struct sizes for py/wire_ctypes.py.
LIB_SRCS  += wire.c
STUB_SRCS += wire_export.c
# Status_Snapshot accessors (status.h) and the Event_Frame builder (evt.h).
LIB_SRCS  += app_status.c evt.c msg_codec.c
# Other Task_Shared_Data accessors (shared.h, Property 18).
LIB_SRCS  += app_shared.c
# Parameter_Store (design D15)
LIB_SRCS  += app_params.c jfmt.c
STUB_SRCS += evt_stub.c

LIB_OBJS  := $(addprefix $(OBJ)/app/,$(LIB_SRCS:.c=.o)) \
             $(addprefix $(OBJ)/stubs/,$(STUB_SRCS:.c=.o))

# Rebuild everything when the flags change (for example APP_LEGACY_TEXT_CMD=1).
FLAGS_STAMP := $(BUILD)/cflags.stamp
$(shell mkdir -p $(BUILD) && (echo '$(HOST_CC) $(HOST_CFLAGS)' | cmp -s - $(FLAGS_STAMP) \
        || echo '$(HOST_CC) $(HOST_CFLAGS)' > $(FLAGS_STAMP)))

# ---- Baseline_Build differential library (build/libbase.so) -------------------
# baseline/fetch_baseline.sh extracts the Baseline_Build sources with git show
# and builds build/libbase.so with every symbol prefixed base_. The commit is
# the BASELINE_COMMIT=<hash> line of docs/evidence/README.md; override with
#     make -C tests/host base BASELINE_COMMIT=<hash>
EVIDENCE_README := ../../docs/evidence/README.md
BASELINE_COMMIT ?= $(shell sed -n 's/^BASELINE_COMMIT=\([0-9a-fA-F]\{7,40\}\).*$$/\1/p' \
                     $(EVIDENCE_README) 2>/dev/null | head -n 1)
BASE_STAMP := $(BUILD)/base.commit
BASE_DEPS  := baseline/fetch_baseline.sh $(wildcard baseline/*.c) \
              baseline/shim/tk/tkernel.h $(BASE_STAMP)
# Rebuild libbase.so when the requested commit changes.
$(shell mkdir -p $(BUILD) && (echo '$(BASELINE_COMMIT)' | cmp -s - $(BASE_STAMP) \
        || echo '$(BASELINE_COMMIT)' > $(BASE_STAMP)))

# ---- targets ----------------------------------------------------------------
.PHONY: all lib base libbase clean tools vectors-c
all: lib
lib: $(BUILD)/libis02.so
base libbase: $(BUILD)/libbase.so

$(BUILD)/libbase.so: $(BASE_DEPS)
	@test -n "$(BASELINE_COMMIT)" || { echo "no BASELINE_COMMIT=<hash> line in $(EVIDENCE_README)" >&2; exit 1; }
	bash baseline/fetch_baseline.sh $(BASELINE_COMMIT)

$(BUILD)/libis02.so: $(LIB_OBJS)
	$(HOST_CC) $(HOST_LDFLAGS) -o $@ $(LIB_OBJS) $(HOST_LIBS)

# ---- protocol tools (spec tasks 3.11, 14.5 C half) ----------------------------
# build/wire_dump: code tables + frame layouts as JSON (laptop table check).
# build/wire_vectors: shared vectors through the C codec (format: tools/wire_vectors.c).
TOOL_OBJS := $(OBJ)/app/wire.o $(OBJ)/app/app_params.o $(OBJ)/app/jfmt.o
tools: $(BUILD)/wire_dump $(BUILD)/wire_vectors

$(BUILD)/wire_dump: tools/wire_dump.c $(TOOL_OBJS)
	$(HOST_CC) $(HOST_CFLAGS) -o $@ $< $(TOOL_OBJS) $(HOST_LIBS)

$(BUILD)/wire_vectors: tools/wire_vectors.c $(TOOL_OBJS)
	$(HOST_CC) $(HOST_CFLAGS) -o $@ $< $(TOOL_OBJS) $(HOST_LIBS)

# Runs vectors/vectors.txt (when present) through the C codec.
vectors-c: tools
	$(BUILD)/wire_dump > $(BUILD)/wire_dump.json
	@if [ -f vectors/vectors.txt ]; then $(BUILD)/wire_vectors < vectors/vectors.txt > $(BUILD)/vectors_c.jsonl; \
	 echo "wrote $(BUILD)/vectors_c.jsonl"; else echo "no vectors/vectors.txt yet (LT-04 writes it)"; fi

$(OBJ)/app/%.o: $(APP)/%.c $(FLAGS_STAMP)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) -MMD -MP -c -o $@ $<

$(OBJ)/stubs/%.o: stubs/%.c $(FLAGS_STAMP)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) -MMD -MP -c -o $@ $<

clean:
	rm -rf $(BUILD)

-include $(wildcard $(OBJ)/*/*.d)
