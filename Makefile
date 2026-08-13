# cbctl — host build and tests. No hardware, no framework, no network.
#
# The vendored cbplc core is compiled from source rather than linked as a prebuilt library,
# so a firmware-side change to the wire protocol breaks this build immediately instead of
# silently disagreeing at runtime.

CC      ?= cc
CSTD    ?= -std=c11
WARN    := -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion \
           -Wpointer-arith -Wwrite-strings -Wmissing-prototypes -Wstrict-prototypes
OPT     ?= -O2 -g
INC     := -Iinclude -Iports -Ithird_party/cbplc/include
# FEATURE-TEST MACROS. One per platform, and NOT _POSIX_C_SOURCE.
#
# -std=c11 gives glibc a strict ISO view in which clock_gettime, nanosleep, cfmakeraw, CRTSCTS
# and the Bxxxxx baud constants do not exist, so the library built clean on the development Mac
# and would not compile at all on the Raspberry Pi it is written for.
#
# The obvious fix — adding -D_POSIX_C_SOURCE=200809L — repairs glibc and BREAKS DARWIN, where
# defining it strictly HIDES the same BSD extensions it reveals on Linux. Each platform needs
# its own macro, and each ignores the other's, so both can be passed unconditionally.
FEATURES := -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE

CFLAGS  := $(CSTD) $(WARN) $(OPT) $(INC) $(FEATURES) -MMD -MP

BUILD   := build
VENDOR  := $(wildcard third_party/cbplc/src/*.c)
SRC     := $(wildcard src/*.c) $(wildcard ports/*.c)
TESTSRC := $(wildcard tests/test_*.c)
TESTS   := $(patsubst tests/test_%.c,$(BUILD)/test_%,$(TESTSRC))

OBJ     := $(patsubst %.c,$(BUILD)/obj/%.o,$(VENDOR) $(SRC))

.PHONY: all check test clean vendor live live-dual fleet-bench budget deadman charge example selftest boardlog provision
# Objects are NOT intermediates. Without this make deletes them after linking, which throws
# away their .d files, and the next build silently skips a rebuild after a HEADER change —
# a stale binary that passes is worse than a broken one that fails.
.SECONDARY: $(OBJ)

all: check

$(BUILD)/obj/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/test_%: tests/test_%.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

test: $(TESTS)
	@fail=0; for t in $(TESTS); do ./$$t || fail=1; done; exit $$fail

vendor:
	@tools/check_vendor_drift.sh

check: test vendor

# Hardware driver and the worked example. Real targets so `make clean` cannot silently leave
# a stale binary behind — and so the hardware gate is one command like the host suite is.
live: $(BUILD)/live_session
$(BUILD)/live_session: tools/live_session.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

live-dual: $(BUILD)/live_dual
$(BUILD)/live_dual: tools/live_dual.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

fleet-bench: $(BUILD)/fleet_bench
$(BUILD)/fleet_bench: tools/fleet_bench.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

deadman: $(BUILD)/deadman_probe
$(BUILD)/deadman_probe: tools/deadman_probe.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

# charge_soc drives real power modules and a real meter, so it uses the SHIPPING drivers —
# cbmodules and cbmeter, the same code the fleet runs — rather than bench re-implementations.
# A bench that talks to hardware its own way is testing the bench. Linux only: SocketCAN.
CBMOD  := ../cbmodules
CBMET  := ../cbmeter
POWSRC := tools/power_cbm.cpp $(CBMOD)/src/module_manager.cpp $(wildcard $(CBMET)/src/*.cpp) \
          $(wildcard $(CBMET)/src/drivers/*.cpp)
POWINC := -I$(CBMOD)/include -I$(CBMET)/include

# module_one drives ONE address, so two instances at DIFFERENT voltages prove
# which meter loop belongs to which gun. Same driver set as module_selftest.
module-one: $(BUILD)/module_one
$(BUILD)/module_one: tools/module_one.c $(POWSRC)
	@mkdir -p $(BUILD)/pow
	$(CXX) -std=c++17 $(OPT) $(POWINC) -c $(POWSRC)
	@mv -f *.o $(BUILD)/pow/ 2>/dev/null || true
	$(CC) $(CFLAGS) -c $< -o $(BUILD)/module_one.o
	$(CXX) $(BUILD)/module_one.o $(BUILD)/pow/*.o -o $@

selftest: $(BUILD)/module_selftest
$(BUILD)/module_selftest: tools/module_selftest.c $(POWSRC)
	@mkdir -p $(BUILD)/pow
	$(CXX) -std=c++17 $(OPT) $(POWINC) -c $(POWSRC)
	@mv -f *.o $(BUILD)/pow/ 2>/dev/null || true
	$(CC) $(CFLAGS) -c $< -o $(BUILD)/module_selftest.o
	$(CXX) $(BUILD)/module_selftest.o $(BUILD)/pow/*.o -o $@

# weld_timing measures how fast the gun-side meter can prove a contactor opened,
# which is the number SafetyLimits::discharge_dwell_ms has to exceed. Same driver
# set as charge_soc for the same reason: a bench that drives hardware its own way
# is testing the bench.
weld-timing: $(BUILD)/weld_timing
$(BUILD)/weld_timing: tools/weld_timing.c $(POWSRC) $(OBJ)
	@mkdir -p $(BUILD)
	@test -d $(CBMOD) || { echo "need sibling checkouts $(CBMOD) and $(CBMET)"; exit 1; }
	$(CXX) -std=c++17 $(OPT) $(POWINC) -c $(POWSRC) 2>&1 | grep -v '^$$' || true
	@mkdir -p $(BUILD)/pow && mv -f *.o $(BUILD)/pow/ 2>/dev/null || true
	$(CC) $(CFLAGS) -c $< -o $(BUILD)/weld_timing.o
	$(CXX) $(BUILD)/weld_timing.o $(OBJ) $(BUILD)/pow/*.o -o $@

charge: $(BUILD)/charge_soc
$(BUILD)/charge_soc: tools/charge_soc.c $(POWSRC) $(OBJ)
	@mkdir -p $(BUILD)
	@test -d $(CBMOD) || { echo "need sibling checkouts $(CBMOD) and $(CBMET)"; exit 1; }
	$(CXX) -std=c++17 $(OPT) $(POWINC) -c $(POWSRC) 2>&1 | grep -v '^$$' || true
	@mkdir -p $(BUILD)/pow && mv -f *.o $(BUILD)/pow/ 2>/dev/null || true
	$(CC) $(CFLAGS) -c $< -o $(BUILD)/charge_soc.o
	$(CXX) $(BUILD)/charge_soc.o $(OBJ) $(BUILD)/pow/*.o -o $@

# The operator-facing way to read a board's log and arm its V2G trace. Pure controller code, so
# it links against $(OBJ) alone — it never touches CAN or the meter.
boardlog: $(BUILD)/board_log
$(BUILD)/board_log: tools/board_log.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

# Declare a charger's identity and a board's calibration. Controller-side only: it never
# authorises, closes a relay or commands a module.
provision: $(BUILD)/provision
$(BUILD)/provision: tools/provision.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

budget: $(BUILD)/tick_budget
$(BUILD)/tick_budget: tools/tick_budget.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

example: $(BUILD)/pi_two_guns
$(BUILD)/pi_two_guns: examples/pi_two_guns.c $(OBJ)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@

clean:
	rm -rf $(BUILD)

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)
