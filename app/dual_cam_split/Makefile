# Makefile for dual IMX335 + MLX90640 split-screen demo
# Target: RK3568 AtomPi-CA1 (aarch64, glibc)

PROG := dual_cam_split
SRC  := dual_cam_split.c
OBJ  := dual_cam_split.o

ROOT_DIR := $(abspath ../..)
TOOLCHAIN_DIR ?= $(ROOT_DIR)/buildroot/output/rockchip_rk3568/host/bin
SYSROOT ?= $(ROOT_DIR)/buildroot/output/rockchip_rk3568/host/aarch64-buildroot-linux-gnu/sysroot
CROSS_COMPILE ?= $(TOOLCHAIN_DIR)/aarch64-buildroot-linux-gnu-
CC := $(CROSS_COMPILE)gcc

CFLAGS := -O2 -Wall -Wextra -std=gnu11 -pthread --sysroot=$(SYSROOT) -I$(SYSROOT)/usr/include/libdrm
LDFLAGS := --sysroot=$(SYSROOT) -pthread -ldrm -lm

BOARD ?= root@169.254.100.100
REMOTE_DIR ?= /usr/bin
REMOTE_PROG := $(REMOTE_DIR)/$(PROG)
REMOTE_TMP ?= /tmp/$(PROG).new
RUN_ARGS ?= -n 60
SSH ?= ssh
SCP ?= scp
SSH_PASS ?= 0000
SSH_OPTS ?= -n -o StrictHostKeyChecking=no -o ConnectTimeout=5
SCP_OPTS ?= -o StrictHostKeyChecking=no -o ConnectTimeout=5

define WITH_SSH_PASS
	@tmp_askpass=$$(mktemp); \
	printf '#!/bin/sh\necho "$$SSH_PASS"\n' > $$tmp_askpass; \
	chmod 700 $$tmp_askpass; \
	SSH_PASS='$(SSH_PASS)' SSH_ASKPASS=$$tmp_askpass SSH_ASKPASS_REQUIRE=force DISPLAY=none setsid sh -c '$(1)'; \
	rc=$$?; \
	rm -f $$tmp_askpass; \
	exit $$rc
endef

.PHONY: all clean install deploy run check-board

all: $(PROG)

$(PROG): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "  Build: $(PROG)"
	@file $(PROG)

$(OBJ): $(SRC) ../thermal_cam/thermal_cam.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(PROG) $(OBJ)

install: $(PROG)
	install -d $(DESTDIR)/usr/bin
	install -m 755 $(PROG) $(DESTDIR)/usr/bin/

check-board:
	@if [ -z "$(BOARD)" ]; then \
		echo "BOARD is not set. Example:"; \
		echo "  make deploy BOARD=root@169.254.100.100"; \
		exit 1; \
	fi

deploy: $(PROG) check-board
	$(call WITH_SSH_PASS,$(SCP) $(SCP_OPTS) $(PROG) $(BOARD):$(REMOTE_TMP))
	$(call WITH_SSH_PASS,$(SSH) $(SSH_OPTS) $(BOARD) "pkill -9 -x $(PROG) 2>/dev/null || true; chmod +x $(REMOTE_TMP); mv -f $(REMOTE_TMP) $(REMOTE_PROG); sync")
	@echo "Installed to $(BOARD):$(REMOTE_PROG)"

run: deploy
	$(call WITH_SSH_PASS,$(SSH) $(SSH_OPTS) $(BOARD) "$(REMOTE_PROG) $(RUN_ARGS)")
