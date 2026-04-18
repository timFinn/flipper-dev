# Monorepo driver for FAP apps.
#
# Each subdirectory under apps/ is an independent ufbt project with its
# own application.fam. This Makefile just iterates them — ufbt does the
# actual build.
#
# Usage:
#   make            # build all apps
#   make APP=hello_world           # build one app
#   make launch APP=hello_world    # build + flash (run on carbon)
#   make clean

APPS_DIR := apps
APPS := $(notdir $(wildcard $(APPS_DIR)/*))

.PHONY: all build launch clean list sdk-update $(APPS)

all: build

list:
	@echo "Apps in this monorepo:"
	@for a in $(APPS); do echo "  - $$a"; done

build:
ifdef APP
	$(MAKE) -C $(APPS_DIR)/$(APP) -f /dev/null build-one
	@echo "Built: $(APPS_DIR)/$(APP)/dist/"
else
	@for a in $(APPS); do \
		echo "=== Building $$a ==="; \
		(cd $(APPS_DIR)/$$a && ufbt) || exit 1; \
	done
endif

# Flash a specific app. Must be run on the host with the Flipper attached.
launch:
ifndef APP
	$(error APP=<name> required, e.g. make launch APP=hello_world)
endif
	cd $(APPS_DIR)/$(APP) && ufbt launch

clean:
	@for a in $(APPS); do \
		echo "=== Cleaning $$a ==="; \
		(cd $(APPS_DIR)/$$a && ufbt -c) || true; \
	done

# Re-pin the ufbt SDK to the channel/version in scripts/sdk-version.
# Run this after pulling, or when bumping the SDK intentionally.
sdk-update:
	./scripts/sdk-update.sh

# Fallback rule so `make hello_world` works as a shortcut for APP=hello_world.
$(APPS):
	$(MAKE) build APP=$@
