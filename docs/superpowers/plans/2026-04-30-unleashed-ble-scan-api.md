# Unleashed BLE Observer-Mode Scan API — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a BLE observer-mode scanning HAL to the Unleashed Flipper Zero firmware fork (`timFinn/unleashed-firmware`), exposing `furi_hal_bt_start_scan` (callback variant) and `furi_hal_bt_start_scan_queued` (queue variant) plus supporting functions, designed for upstream-PR consideration.

**Architecture:** Surgical patch (~2-4 KB M4 code) wiring ST's already-linked `aci_gap_*` and `hci_le_*` observer primitives through a new `furi_hal_bt_scan.{c,h}` module. State machine (Idle/Starting/Active/Stopping) protected by two mutexes (`control_mutex` for FAP-side serialization, `dispatch_mutex` as event-handler join barrier). Atomic phase variable with release/acquire ordering. Backpressure via `BleEventAckFlowDisable`; backpressure timeout, duration auto-stop, and orphaned-FAP detection all signal via a new `BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP` bit observed on `ble_event_thread`. No coprocessor binary changes.

**Tech Stack:** C99, GCC arm-none-eabi (Unleashed's `fbt` build), STM32WB55RG hardware, ST `BLE_Stack_light` BLE coprocessor binary (already bundled), FURI RTOS APIs (FuriThread, FuriMutex, FuriTimer, FuriMessageQueue, atomic intrinsics via `__atomic_*`), `minunit.h` test framework.

---

## Source of truth

The spec at `docs/superpowers/specs/2026-04-28-unleashed-ble-scan-api-design.md` is authoritative. This plan implements that spec. Where this plan and the spec disagree, the spec wins; flag the discrepancy in a code comment and ask the user.

## Working directory context

All file paths in this plan are **relative to the cloned Unleashed repository root** (e.g., `~/projects/Unleashed-Firmware/` after Task 0 completes). The spec lives in a separate `flipper-dev` repo at `/home/tim/projects/flipper-dev/docs/superpowers/specs/`. Implementation work happens in the firmware repo.

## Pre-flight: working environment

The implementation must happen in a clone of the forked Unleashed firmware. Task 0 establishes that environment.

---

### Task 0: Set up working environment

**Files:**
- Clone: `~/projects/Unleashed-Firmware/` (new directory)

- [ ] **Step 1: Verify the fork exists on GitHub**

Run: `gh repo view timFinn/unleashed-firmware --json url,parent`
Expected: JSON output showing the fork with `parent.name == "unleashed-firmware"` and `parent.owner.login == "DarkFlippers"`. If the fork does not exist, ask the user before proceeding.

- [ ] **Step 2: Clone the fork**

Run: `git clone git@github.com:timFinn/unleashed-firmware.git ~/projects/Unleashed-Firmware`
Expected: Clone completes; `~/projects/Unleashed-Firmware/.git` exists.

- [ ] **Step 3: Add upstream remote and fetch the latest release tag**

Run:
```
cd ~/projects/Unleashed-Firmware
git remote add upstream https://github.com/DarkFlippers/unleashed-firmware.git
git fetch upstream --tags
```
Expected: Tags fetched. List them with `git tag --sort=-creatordate | head -5` and pick the most recent `unlshd-*` tag (e.g., `unlshd-086`).

- [ ] **Step 4: Create the feature branch from the chosen release tag**

Run (substituting the actual tag from Step 3):
```
cd ~/projects/Unleashed-Firmware
git checkout -b feature/ble-scan-api unlshd-086
```
Expected: Branch created and checked out at the tagged commit.

- [ ] **Step 5: Verify the build environment works (baseline)**

Run:
```
cd ~/projects/Unleashed-Firmware
./fbt firmware_all
```
Expected: Successful build. If this fails, do NOT proceed — diagnose with the user. Capture the output of `./fbt fwsize` (or `python3 scripts/fwsize.py`) and save the M4 `.free_flash` headroom number to `~/baseline_flash.txt`. We'll diff against this in later tasks.

- [ ] **Step 6: Push the feature branch**

Run:
```
cd ~/projects/Unleashed-Firmware
git push -u origin feature/ble-scan-api
```
Expected: Branch pushed; tracking origin set.

---

### Task 1: gap: add GAP_OBSERVER_ROLE to aci_gap_init role mask

This is the smallest possible commit that makes observer mode reachable at all. Without this change, every later HCI scan call would fail with `BLE_STATUS_NOT_ALLOWED`.

**Files:**
- Modify: `targets/f7/ble_glue/gap.c:361` (the `aci_gap_init` call)

- [ ] **Step 1: Read the current `aci_gap_init` invocation**

Run: `sed -n '355,375p' targets/f7/ble_glue/gap.c`
Expected: See the current `aci_gap_init(GAP_PERIPHERAL_ROLE, ...)` call. Confirm the line number matches the spec's claim of `gap.c:361`. If it has drifted, note the new line number and proceed accordingly.

- [ ] **Step 2: Patch the role mask**

Edit `targets/f7/ble_glue/gap.c`. Find:
```c
    status = aci_gap_init(
        GAP_PERIPHERAL_ROLE,
```
Replace with:
```c
    status = aci_gap_init(
        GAP_PERIPHERAL_ROLE | GAP_OBSERVER_ROLE,
```

- [ ] **Step 3: Verify the build still succeeds**

Run: `./fbt firmware_all`
Expected: Build succeeds. The role-mask change is purely additive and the ST stack already implements observer in `BLE_Stack_light`.

- [ ] **Step 4: Verify nothing in existing tests regressed**

Manually check on `carbon`: BLE peripheral mode (qFlipper pairing) still works. This is a 30-second smoke test; the role mask is additive, but verify before committing.

- [ ] **Step 5: Commit**

Run:
```
cd ~/projects/Unleashed-Firmware
./fbt format
git add targets/f7/ble_glue/gap.c
git commit -m "gap: add GAP_OBSERVER_ROLE to aci_gap_init role mask

Required for HCI observer-mode scan calls to succeed. Without this,
hci_le_set_scan_enable returns BLE_STATUS_NOT_ALLOWED. Additive change;
peripheral functionality preserved."
```

---

### Task 2: gap: add internal connection-state accessors

These two functions are needed by the scan HAL (Task 5+) to drive the connection-aware window/interval downshift. They are **private** — declared in a new `gap_internal.h` header, not exported via `api_symbols.csv`.

**Files:**
- Create: `targets/f7/ble_glue/gap_internal.h`
- Modify: `targets/f7/ble_glue/gap.c` (add two function definitions)

- [ ] **Step 1: Inspect existing GAP state to understand what to read**

Run: `grep -n 'connection_handle\|conn_interval\|gap->service' targets/f7/ble_glue/gap.c | head -30`
Expected: See how the existing code reads connection state (typically `gap->service.connection_handle != INVALID_HANDLE`). Note the locking pattern (the existing event handler holds `gap->state_mutex`).

- [ ] **Step 2: Create the gap_internal.h header**

Create `targets/f7/ble_glue/gap_internal.h` with:
```c
/**
 * @file gap_internal.h
 * Private GAP helpers used only by furi_hal_bt_scan.c.
 *
 * NOT part of the public HAL. Not declared in any installed header.
 * Not exported via api_symbols.csv. FAPs cannot reach this header.
 */

#pragma once

#include <gap.h>
#include <stdbool.h>
#include <stdint.h>

/* Connection-state read accessors */

/** Whether a GATT peer is currently connected. */
bool gap_has_active_connection(void);

/** Active connection interval in ms. Returns false if no connection. */
bool gap_get_connection_interval_ms(uint16_t* out_interval_ms);

/* Advertising pause/resume (added in Task 3) */

typedef struct {
    bool     was_active;
    bool     was_connectable;
    uint16_t saved_min_adv_interval;
    uint16_t saved_max_adv_interval;
} GapAdvertisingSavedState;

void gap_pause_advertising(GapAdvertisingSavedState* out_saved);
void gap_resume_advertising(const GapAdvertisingSavedState* saved);
```

- [ ] **Step 3: Add accessor implementations to gap.c**

Open `targets/f7/ble_glue/gap.c`. Add at the top:
```c
#include "gap_internal.h"
```

Add at the bottom of the file (before any existing `#endif` or end of file):
```c
bool gap_has_active_connection(void) {
    if(!gap) return false;
    furi_mutex_acquire(gap->state_mutex, FuriWaitForever);
    bool active = gap->service.connection_handle != GAP_INVALID_CONNECTION_HANDLE;
    furi_mutex_release(gap->state_mutex);
    return active;
}

bool gap_get_connection_interval_ms(uint16_t* out_interval_ms) {
    furi_check(out_interval_ms);
    if(!gap) return false;
    furi_mutex_acquire(gap->state_mutex, FuriWaitForever);
    bool has_conn = gap->service.connection_handle != GAP_INVALID_CONNECTION_HANDLE;
    if(has_conn) {
        /* connection_interval is reported by ST stack in 1.25 ms units. */
        *out_interval_ms = (uint16_t)((gap->service.connection_interval * 125U) / 100U);
    }
    furi_mutex_release(gap->state_mutex);
    return has_conn;
}
```

**Note:** `GAP_INVALID_CONNECTION_HANDLE` and `gap->service.connection_interval` are the names this plan assumes. If the actual fields in `targets/f7/ble_glue/gap.c` use different names (e.g., `gap->service.conn_interval`), substitute the real names. This is a mechanical rename, not a design change.

- [ ] **Step 4: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds. No new symbols leak to FAPs (gap_internal.h is not in any installed-header path).

- [ ] **Step 5: Verify api_symbols.csv was NOT modified**

Run: `git diff -- targets/f7/api_symbols.csv`
Expected: Empty output. The accessors are private; they MUST NOT appear in api_symbols.csv.

- [ ] **Step 6: Commit**

Run:
```
./fbt format
git add targets/f7/ble_glue/gap_internal.h targets/f7/ble_glue/gap.c
git commit -m "gap: add internal connection-state accessors

gap_has_active_connection and gap_get_connection_interval_ms read
GATT connection state under the existing gap state_mutex. Private to
ble_glue/ via gap_internal.h; not exported to FAPs."
```

---

### Task 3: gap: add internal advertising pause/resume helpers

Used by the scan HAL (Task 8) to pause advertising during active scans (so SCAN_REQ transmissions don't collide with the Flipper's own ADV_IND).

**Files:**
- Modify: `targets/f7/ble_glue/gap.c` (append two functions)

- [ ] **Step 1: Inspect the existing advertising start/stop code**

Run: `grep -n 'aci_gap_set_discoverable\|aci_gap_set_non_discoverable\|adv_interval' targets/f7/ble_glue/gap.c | head -20`
Expected: See `aci_gap_set_discoverable(...)` (around line 456 per spec) and `aci_gap_set_non_discoverable()` (around line 443) call sites. These are what `gap_pause_advertising` and `gap_resume_advertising` will wrap.

- [ ] **Step 2: Add pause/resume implementations to gap.c**

Append to `targets/f7/ble_glue/gap.c`:
```c
void gap_pause_advertising(GapAdvertisingSavedState* out_saved) {
    furi_check(out_saved);
    memset(out_saved, 0, sizeof(*out_saved));
    if(!gap) return;
    furi_mutex_acquire(gap->state_mutex, FuriWaitForever);
    /* Determine whether advertising is currently active. The existing
     * gap.c tracks this via gap->state — match the predicate the
     * existing aci_gap_set_non_discoverable callers use. Adjust the
     * field name here if the codebase uses a different state enum. */
    out_saved->was_active = (gap->state == GapStateAdvFast || gap->state == GapStateAdvLowPower);
    if(out_saved->was_active) {
        out_saved->was_connectable = gap->service.advertise_connectable;
        out_saved->saved_min_adv_interval = gap->service.adv_interval_min;
        out_saved->saved_max_adv_interval = gap->service.adv_interval_max;
        aci_gap_set_non_discoverable();
        gap->state = GapStateIdle;
    }
    furi_mutex_release(gap->state_mutex);
}

void gap_resume_advertising(const GapAdvertisingSavedState* saved) {
    furi_check(saved);
    if(!saved->was_active || !gap) return;
    furi_mutex_acquire(gap->state_mutex, FuriWaitForever);
    /* Restore the prior advertising state by re-invoking the same path
     * the regular advertising start uses. The cleanest way is to call
     * the existing gap_start_advertising() — it reads gap->service
     * fields which we did not modify. */
    furi_mutex_release(gap->state_mutex);
    gap_start_advertising();
}
```

**Note:** field/enum names (`GapStateAdvFast`, `gap->service.adv_interval_min`, etc.) are placeholders matching common patterns. Substitute the actual names from `gap.c`. If the existing `gap_start_advertising()` is not idempotent or has side effects we don't want, the resume path may need direct ACI calls instead — flag this if encountered.

- [ ] **Step 3: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds.

- [ ] **Step 4: Smoke test on hardware**

On `carbon`: flash this build, start qFlipper. Verify pairing still works (advertising is functional, since pause/resume isn't called yet). The functions are present but unused at this point.

- [ ] **Step 5: Commit**

Run:
```
./fbt format
git add targets/f7/ble_glue/gap.c
git commit -m "gap: add internal advertising pause/resume helpers

gap_pause_advertising captures current adv state and stops advertising;
gap_resume_advertising restores it. Used by scan HAL during active
scans to prevent SCAN_REQ-vs-ADV_IND collisions. Private."
```

---

### Task 4: ble_glue: bump CFG_TLBLE_EVT_QUEUE_LENGTH for observer headroom

Observer mode generates a high stream of HCI events; the default IPCC event queue depth (5) is too shallow under busy RF. Bumping to 8 costs ~1.5 KB SRAM2A but prevents queue starvation. Verified during T17 stress test in later tasks.

**Files:**
- Modify: `targets/f7/ble_glue/app_conf.h`

- [ ] **Step 1: Locate the existing definition**

Run: `grep -n 'CFG_TLBLE_EVT_QUEUE_LENGTH' targets/f7/ble_glue/app_conf.h`
Expected: Line containing `#define CFG_TLBLE_EVT_QUEUE_LENGTH 5`.

- [ ] **Step 2: Bump to 8**

Edit `targets/f7/ble_glue/app_conf.h`. Find:
```c
#define CFG_TLBLE_EVT_QUEUE_LENGTH 5
```
Replace with:
```c
#define CFG_TLBLE_EVT_QUEUE_LENGTH 8
```

- [ ] **Step 3: Verify build and check size impact**

Run:
```
./fbt firmware_all
python3 scripts/fwsize.py 2>&1 | tee /tmp/fwsize_after_task4.txt
```
Expected: Build succeeds. `__free_flash_start__` headroom should be essentially unchanged from the Task 0 baseline; the change is in SRAM2A, not flash.

- [ ] **Step 4: Smoke test**

On `carbon`: flash, verify qFlipper pairing still works.

- [ ] **Step 5: Commit**

Run:
```
git add targets/f7/ble_glue/app_conf.h
git commit -m "ble_glue: bump CFG_TLBLE_EVT_QUEUE_LENGTH for observer headroom

Observer mode generates more HCI events than peripheral-only operation.
Default queue depth of 5 is too shallow in busy RF environments,
causing IPCC starvation. 8 entries costs ~1.5 KB SRAM2A and prevents
the failure mode."
```

---

### Task 5: furi_hal_bt: add observer scan HAL skeleton (returns NotReady)

Establish the public header, the internal state struct, the validation logic, and stubs for all public functions. `start_scan` returns `NotReady` until Task 6 wires the actual dispatch. This commit produces a compilable, linkable, but functionally-disabled scan HAL — useful as a bisect anchor.

**Files:**
- Create: `targets/f7/ble_glue/furi_hal_bt_scan.h`
- Create: `targets/f7/ble_glue/furi_hal_bt_scan.c`
- Create: `targets/f7/ble_glue/furi_hal_bt_scan_internal.h` (helpers exposed for unit tests)
- Modify: `targets/furi_hal_include/furi_hal_bt.h` (transitive include)

- [ ] **Step 1: Create the public header**

Create `targets/f7/ble_glue/furi_hal_bt_scan.h` with the full content from Section 2.1 of the spec (`docs/superpowers/specs/2026-04-28-unleashed-ble-scan-api-design.md`). The header is ~150 lines; copy verbatim. Key things to verify after copying:
- `FURI_HAL_BT_SCAN_ADV_MAX_LEN` defined as 31
- `FURI_HAL_BT_SCAN_CONFIG_MIN_SIZE` defined as `sizeof(FuriHalBtScanConfig)`
- `FURI_HAL_BT_SCAN_CONFIG_DEFAULT` macro present with all 11 fields initialized
- All public function prototypes present: `furi_hal_bt_start_scan`, `furi_hal_bt_start_scan_queued`, `furi_hal_bt_stop_scan`, `furi_hal_bt_scan_resume_flow`, `furi_hal_bt_is_scanning`, `furi_hal_bt_scan_force_stop`, `furi_hal_bt_scan_stats`, `furi_hal_bt_scan_stats_reset`, `furi_hal_bt_scan_last_hci_status`

- [ ] **Step 2: Add transitive include from furi_hal_bt.h**

Edit `targets/furi_hal_include/furi_hal_bt.h`. After the existing includes (around line 14), add:
```c
#include <furi_hal_bt_scan.h>
```

- [ ] **Step 3: Create the internal header for testable helpers**

Create `targets/f7/ble_glue/furi_hal_bt_scan_internal.h` with:
```c
/**
 * @file furi_hal_bt_scan_internal.h
 * Internal helpers exposed for unit tests.
 * Not part of the public HAL.
 */

#pragma once

#include "furi_hal_bt_scan.h"
#include <stdbool.h>
#include <stdint.h>

/** Validate a scan config struct. Pure function; testable on host. */
FuriHalBtScanError validate_config(const FuriHalBtScanConfig* config);

/** Linear MAC match against an allowlist. Pure function. */
bool allowlist_permits(const uint8_t mac[6], const uint8_t (*list)[6], uint8_t len);

/**
 * Decide whether a session should be force-stopped due to backpressure
 * timeout. Pure function.
 *
 * @param flow_disabled            true iff backpressure currently active
 * @param now_ms                   current furi_get_tick() in ms
 * @param flow_disabled_since_ms   tick when flow_disabled was last set true
 * @param timeout_ms               configured timeout
 */
bool should_force_stop_backpressure(
    bool flow_disabled,
    uint32_t now_ms,
    uint32_t flow_disabled_since_ms,
    uint32_t timeout_ms);
```

- [ ] **Step 4: Create the implementation file (skeleton)**

Create `targets/f7/ble_glue/furi_hal_bt_scan.c` with:
```c
#include "furi_hal_bt_scan.h"
#include "furi_hal_bt_scan_internal.h"
#include "gap_internal.h"

#include <furi.h>
#include <furi_hal_bt.h>
#include <stdatomic.h>
#include <string.h>

/* Compile-time configuration */
#define FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS 5000

/* Internal state types */
typedef enum {
    ScanPhaseIdle = 0,
    ScanPhaseStarting,
    ScanPhaseActive,
    ScanPhaseStopping,
} ScanPhase;

typedef enum {
    ScanModeCallback,
    ScanModeQueued,
} ScanMode;

typedef struct {
    FuriMutex* control_mutex;
    FuriMutex* dispatch_mutex;
    uint32_t   phase; /* ScanPhase */

    FuriThreadId owner_thread;
    FuriThreadId dispatch_owner;

    ScanMode               mode;
    FuriHalBtScanCallback  callback;
    void*                  context;
    FuriMessageQueue*      queue;
    FuriHalBtScanEndedCallback ended_callback;
    void*                  ended_context;
    struct {
        uint8_t (*list)[6];
        uint8_t   len;
    } allowlist;
    GapAdvertisingSavedState saved_adv;
    bool       adv_was_paused;

    bool       flow_disabled;
    uint32_t   flow_disabled_since_ms;

    FuriTimer* duration_timer;

    FuriHalBtScanEndedReason pending_end_reason;
    bool       pending_end_set;

    uint8_t    last_hci_status;

    FuriHalBtScanStats stats;
} ScanModule;

static ScanModule g_scan;
static bool g_initialized = false;

/* Lazy init of mutexes on first use */
static void ensure_init(void) {
    if(!g_initialized) {
        memset(&g_scan, 0, sizeof(g_scan));
        g_scan.control_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
        g_scan.dispatch_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
        g_initialized = true;
    }
}

/* ── Pure helpers (also exposed via internal header for tests) ────── */

FuriHalBtScanError validate_config(const FuriHalBtScanConfig* config) {
    if(!config) return FuriHalBtScanErrorInvalidParameter;
    if(config->struct_size < FURI_HAL_BT_SCAN_CONFIG_MIN_SIZE) {
        return FuriHalBtScanErrorInvalidParameter;
    }
    if(config->struct_size > sizeof(FuriHalBtScanConfig)) {
        return FuriHalBtScanErrorInvalidParameter;
    }
    if(config->type != FuriHalBtScanTypePassive && config->type != FuriHalBtScanTypeActive) {
        return FuriHalBtScanErrorInvalidParameter;
    }
    if(config->window_ms == 0 || config->interval_ms == 0) {
        return FuriHalBtScanErrorInvalidParameter;
    }
    /* ETSI EN 300 328 duty-cycle: window must not exceed 95% of interval */
    if(((uint32_t)config->window_ms * 100U) > ((uint32_t)config->interval_ms * 95U)) {
        return FuriHalBtScanErrorInvalidParameter;
    }
    if(config->allowlist_len > 0 && config->allowlist == NULL) {
        return FuriHalBtScanErrorInvalidParameter;
    }
    return FuriHalBtScanErrorNone;
}

bool allowlist_permits(const uint8_t mac[6], const uint8_t (*list)[6], uint8_t len) {
    if(len == 0 || list == NULL) return true;
    for(uint8_t i = 0; i < len; i++) {
        if(memcmp(mac, list[i], 6) == 0) return true;
    }
    return false;
}

bool should_force_stop_backpressure(
    bool flow_disabled,
    uint32_t now_ms,
    uint32_t flow_disabled_since_ms,
    uint32_t timeout_ms) {
    if(!flow_disabled) return false;
    /* Unsigned subtraction handles tick wraparound naturally */
    return (now_ms - flow_disabled_since_ms) >= timeout_ms;
}

/* ── Public API stubs (skeleton — all return NotReady) ───────────── */

FuriHalBtScanError furi_hal_bt_start_scan(
    const FuriHalBtScanConfig* config,
    FuriHalBtScanCallback callback,
    void* context) {
    UNUSED(config);
    UNUSED(callback);
    UNUSED(context);
    ensure_init();
    return FuriHalBtScanErrorNotReady;
}

FuriHalBtScanError furi_hal_bt_start_scan_queued(
    const FuriHalBtScanConfig* config,
    FuriMessageQueue* report_queue) {
    UNUSED(config);
    UNUSED(report_queue);
    ensure_init();
    return FuriHalBtScanErrorNotReady;
}

FuriHalBtScanError furi_hal_bt_stop_scan(void) {
    ensure_init();
    return FuriHalBtScanErrorNone; /* idempotent */
}

void furi_hal_bt_scan_resume_flow(void) {
    ensure_init();
}

bool furi_hal_bt_is_scanning(void) {
    ensure_init();
    return false;
}

void furi_hal_bt_scan_force_stop(void) {
    ensure_init();
}

void furi_hal_bt_scan_stats(FuriHalBtScanStats* out) {
    furi_check(out);
    ensure_init();
    /* Atomic loads of u32 fields */
    out->events_received      = __atomic_load_n(&g_scan.stats.events_received,      __ATOMIC_RELAXED);
    out->events_filtered      = __atomic_load_n(&g_scan.stats.events_filtered,      __ATOMIC_RELAXED);
    out->events_delivered     = __atomic_load_n(&g_scan.stats.events_delivered,     __ATOMIC_RELAXED);
    out->events_dropped       = __atomic_load_n(&g_scan.stats.events_dropped,       __ATOMIC_RELAXED);
    out->events_dropped_phase = __atomic_load_n(&g_scan.stats.events_dropped_phase, __ATOMIC_RELAXED);
}

void furi_hal_bt_scan_stats_reset(void) {
    ensure_init();
    memset(&g_scan.stats, 0, sizeof(g_scan.stats));
}

uint8_t furi_hal_bt_scan_last_hci_status(void) {
    ensure_init();
    return __atomic_load_n(&g_scan.last_hci_status, __ATOMIC_RELAXED);
}
```

- [ ] **Step 5: Update SConscript / build configuration to include the new files**

Look for the SCons configuration that compiles `targets/f7/ble_glue/`:
```
grep -rn "ble_glue" targets/f7/SConscript* 2>/dev/null
```
The new `.c` file should be picked up automatically by the existing glob (most Flipper SConscripts use `Glob("ble_glue/*.c")`). If not, add `furi_hal_bt_scan.c` to the source list.

- [ ] **Step 6: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds. The new module compiles; symbols link.

- [ ] **Step 7: Commit**

Run:
```
./fbt format
git add targets/f7/ble_glue/furi_hal_bt_scan.h \
        targets/f7/ble_glue/furi_hal_bt_scan_internal.h \
        targets/f7/ble_glue/furi_hal_bt_scan.c \
        targets/furi_hal_include/furi_hal_bt.h
git commit -m "furi_hal_bt: add observer scan HAL skeleton

Public surface defined per spec; all entry points return NotReady or
no-op. State machine, mutexes, atomic stats, and pure validation
helpers in place. Subsequent commits wire the dispatch path.

Validation helpers (validate_config, allowlist_permits,
should_force_stop_backpressure) are exposed via furi_hal_bt_scan_internal.h
for the unit tests added in commit 11."
```

---

### Task 6: furi_hal_bt: wire dispatch path with thread-liveness check (callback variant)

Activate the callback variant. The skeleton's `furi_hal_bt_start_scan` now actually starts a scan and dispatches HCI events to the registered callback. Implements the F1 mitigation (thread-liveness check on `owner_thread`).

**Files:**
- Modify: `targets/f7/ble_glue/furi_hal_bt_scan.c`

- [ ] **Step 1: Add the HCI event handler registration**

The Unleashed BLE event dispatcher pattern is in `targets/f7/ble_glue/furi_ble/event_dispatcher.{c,h}`. Read it:
```
sed -n '1,80p' targets/f7/ble_glue/furi_ble/event_dispatcher.c
sed -n '1,80p' targets/f7/ble_glue/furi_ble/event_dispatcher.h
```
Note the exact registration API and the return-status enum (likely `BleEventNotAck`, `BleEventAckFlowEnable`, `BleEventAckFlowDisable`).

- [ ] **Step 2: Add the event handler and registration in furi_hal_bt_scan.c**

In `targets/f7/ble_glue/furi_hal_bt_scan.c`, add an event handler. The exact dispatcher API (e.g., `ble_event_dispatcher_add_callback(...)`) must match what Unleashed uses; substitute as needed:

```c
/* Forward decl */
static BleEventFlowStatus on_adv_report(void* event, void* context);

static void register_event_handler_once(void) {
    static bool registered = false;
    if(registered) return;
    /* Replace with Unleashed's actual registration API */
    ble_event_dispatcher_add_callback(HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE, on_adv_report, NULL);
    registered = true;
}

static BleEventFlowStatus on_adv_report(void* event, void* context) {
    UNUSED(context);

    if(furi_mutex_acquire(g_scan.dispatch_mutex, 0) != FuriStatusOk) {
        /* Concurrent stop_scan teardown — let it through with no-ack */
        return BleEventNotAck;
    }

    ScanPhase phase = (ScanPhase)__atomic_load_n(&g_scan.phase, __ATOMIC_ACQUIRE);
    if(phase != ScanPhaseActive) {
        __atomic_fetch_add(&g_scan.stats.events_dropped_phase, 1, __ATOMIC_RELAXED);
        furi_mutex_release(g_scan.dispatch_mutex);
        return BleEventAckFlowEnable;
    }

    /* F1 mitigation: thread-liveness check */
    if(g_scan.owner_thread != NULL && !furi_thread_is_alive(g_scan.owner_thread)) {
        if(!g_scan.pending_end_set) {
            g_scan.pending_end_reason = FuriHalBtScanEndedReasonForceStoppedForReinit;
            g_scan.pending_end_set = true;
        }
        furi_thread_flags_set(
            furi_thread_get_id(ble_event_thread_get_thread()),
            BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP);
        furi_mutex_release(g_scan.dispatch_mutex);
        return BleEventAckFlowEnable;
    }

    __atomic_fetch_add(&g_scan.stats.events_received, 1, __ATOMIC_RELAXED);

    /* Build the report from ST stack's event struct.
     * The exact ST struct name is hci_le_advertising_report_event_rp0;
     * field names follow the ST stack conventions. */
    hci_le_advertising_report_event_rp0* evt = (hci_le_advertising_report_event_rp0*)event;
    Advertising_Report_t* rpt = &evt->Advertising_Report[0]; /* one report per event in ST stack */

    FuriHalBtScanReport report;
    memset(&report, 0, sizeof(report));
    report.struct_size = sizeof(FuriHalBtScanReport);
    memcpy(report.mac, rpt->Address, 6);
    report.addr_type = rpt->Address_Type;
    report.rssi = (int8_t)rpt->RSSI;
    uint8_t adv_len = rpt->Length_Data;
    if(adv_len > FURI_HAL_BT_SCAN_ADV_MAX_LEN) adv_len = FURI_HAL_BT_SCAN_ADV_MAX_LEN;
    report.adv_data_len = adv_len;
    if(!g_scan.allowlist.list || allowlist_permits(report.mac, g_scan.allowlist.list, g_scan.allowlist.len)) {
        /* allowlist_permits returns true for empty list */
    } else {
        __atomic_fetch_add(&g_scan.stats.events_filtered, 1, __ATOMIC_RELAXED);
        furi_mutex_release(g_scan.dispatch_mutex);
        return BleEventAckFlowEnable;
    }
    if(!g_scan.allowlist.list || allowlist_permits(report.mac, g_scan.allowlist.list, g_scan.allowlist.len)) {
        memcpy(report.adv_data, rpt->Data, adv_len);
    }

    /* Dispatch — callback variant only in this commit */
    if(g_scan.mode == ScanModeCallback && g_scan.callback) {
        g_scan.dispatch_owner = furi_thread_get_current_id();
        g_scan.callback(&report, g_scan.context);
        g_scan.dispatch_owner = NULL;
    }
    __atomic_fetch_add(&g_scan.stats.events_delivered, 1, __ATOMIC_RELAXED);

    furi_mutex_release(g_scan.dispatch_mutex);
    return BleEventAckFlowEnable;
}
```

**Note:** ST stack types (`hci_le_advertising_report_event_rp0`, `Advertising_Report_t`) and the dispatcher registration API are placeholders following ST's naming. Replace with the actual symbols Unleashed uses. The path from registration → callback delivery is in `event_dispatcher.{c,h}` and the ST headers under `lib/stm32wb_copro/`.

- [ ] **Step 3: Implement furi_hal_bt_start_scan (callback variant)**

Replace the stub `furi_hal_bt_start_scan` in `furi_hal_bt_scan.c`:
```c
FuriHalBtScanError furi_hal_bt_start_scan(
    const FuriHalBtScanConfig* config,
    FuriHalBtScanCallback callback,
    void* context) {
    ensure_init();

    /* Re-entry detection: must not be called from dispatch thread */
    if(g_scan.dispatch_owner == furi_thread_get_current_id()) {
        return FuriHalBtScanErrorReentrant;
    }
    if(!callback) return FuriHalBtScanErrorInvalidParameter;

    FuriHalBtScanError verr = validate_config(config);
    if(verr != FuriHalBtScanErrorNone) return verr;

    furi_mutex_acquire(g_scan.control_mutex, FuriWaitForever);

    if((ScanPhase)__atomic_load_n(&g_scan.phase, __ATOMIC_ACQUIRE) != ScanPhaseIdle) {
        furi_mutex_release(g_scan.control_mutex);
        return FuriHalBtScanErrorBusy;
    }
    if(!furi_hal_bt_is_alive()) {
        furi_mutex_release(g_scan.control_mutex);
        return FuriHalBtScanErrorNotReady;
    }

    /* Reset last_hci_status */
    __atomic_store_n(&g_scan.last_hci_status, 0, __ATOMIC_RELAXED);

    /* Allocate allowlist copy if needed */
    if(config->allowlist_len > 0) {
        g_scan.allowlist.list = malloc((size_t)config->allowlist_len * 6U);
        memcpy(g_scan.allowlist.list, config->allowlist, (size_t)config->allowlist_len * 6U);
        g_scan.allowlist.len = config->allowlist_len;
    } else {
        g_scan.allowlist.list = NULL;
        g_scan.allowlist.len = 0;
    }

    /* Populate state */
    g_scan.mode = ScanModeCallback;
    g_scan.callback = callback;
    g_scan.context = context;
    g_scan.queue = NULL;
    g_scan.ended_callback = config->ended_callback;
    g_scan.ended_context = config->ended_context;
    g_scan.owner_thread = furi_thread_get_current_id();
    g_scan.pending_end_set = false;
    g_scan.flow_disabled = false;
    g_scan.adv_was_paused = false;

    /* Active scan: pause own advertising for the duration */
    if(config->type == FuriHalBtScanTypeActive) {
        gap_pause_advertising(&g_scan.saved_adv);
        g_scan.adv_was_paused = g_scan.saved_adv.was_active;
    }

    /* Connection-aware downshift */
    uint16_t window = config->window_ms;
    uint16_t interval = config->interval_ms;
    if(gap_has_active_connection()) {
        if(window > 20) window = 20;
        if(interval < 160) interval = 160;
    }

    /* Phase → Active BEFORE enabling the scan, so dispatched events
     * are not falsely dropped to events_dropped_phase. */
    __atomic_store_n(&g_scan.phase, ScanPhaseActive, __ATOMIC_RELEASE);

    register_event_handler_once();

    /* hci_le_set_scan_parameters values: ST stack uses 0.625 ms units */
    uint16_t scan_window_units = (uint16_t)(((uint32_t)window * 1000U) / 625U);
    uint16_t scan_interval_units = (uint16_t)(((uint32_t)interval * 1000U) / 625U);
    uint8_t scan_type = (config->type == FuriHalBtScanTypeActive) ? 1 : 0;

    tBleStatus rc = hci_le_set_scan_parameters(
        scan_type, scan_interval_units, scan_window_units,
        /* own_addr_type */ 0, /* filter_policy */ 0);
    if(rc != BLE_STATUS_SUCCESS) {
        __atomic_store_n(&g_scan.last_hci_status, (uint8_t)rc, __ATOMIC_RELAXED);
        __atomic_store_n(&g_scan.phase, ScanPhaseIdle, __ATOMIC_RELEASE);
        if(g_scan.allowlist.list) { free(g_scan.allowlist.list); g_scan.allowlist.list = NULL; }
        if(g_scan.adv_was_paused) gap_resume_advertising(&g_scan.saved_adv);
        furi_mutex_release(g_scan.control_mutex);
        return FuriHalBtScanErrorControllerReject;
    }

    rc = hci_le_set_scan_enable(1, config->filter_duplicates ? 1 : 0);
    if(rc != BLE_STATUS_SUCCESS) {
        __atomic_store_n(&g_scan.last_hci_status, (uint8_t)rc, __ATOMIC_RELAXED);
        __atomic_store_n(&g_scan.phase, ScanPhaseIdle, __ATOMIC_RELEASE);
        if(g_scan.allowlist.list) { free(g_scan.allowlist.list); g_scan.allowlist.list = NULL; }
        if(g_scan.adv_was_paused) gap_resume_advertising(&g_scan.saved_adv);
        furi_mutex_release(g_scan.control_mutex);
        return FuriHalBtScanErrorControllerReject;
    }

    furi_mutex_release(g_scan.control_mutex);
    return FuriHalBtScanErrorNone;
}
```

- [ ] **Step 4: Implement furi_hal_bt_stop_scan**

Replace the stub:
```c
FuriHalBtScanError furi_hal_bt_stop_scan(void) {
    ensure_init();

    if(g_scan.dispatch_owner == furi_thread_get_current_id()) {
        return FuriHalBtScanErrorReentrant;
    }

    furi_mutex_acquire(g_scan.control_mutex, FuriWaitForever);

    if((ScanPhase)__atomic_load_n(&g_scan.phase, __ATOMIC_ACQUIRE) != ScanPhaseActive) {
        furi_mutex_release(g_scan.control_mutex);
        return FuriHalBtScanErrorNone; /* idempotent */
    }

    __atomic_store_n(&g_scan.phase, ScanPhaseStopping, __ATOMIC_RELEASE);

    /* Synchronous HCI disable; on return, all in-flight events have drained */
    hci_le_set_scan_enable(0, 0);

    /* Join barrier */
    furi_mutex_acquire(g_scan.dispatch_mutex, FuriWaitForever);

    if(g_scan.adv_was_paused) {
        gap_resume_advertising(&g_scan.saved_adv);
        g_scan.adv_was_paused = false;
    }
    if(g_scan.allowlist.list) {
        free(g_scan.allowlist.list);
        g_scan.allowlist.list = NULL;
        g_scan.allowlist.len = 0;
    }

    /* Resolve end reason */
    FuriHalBtScanEndedReason reason =
        g_scan.pending_end_set ? g_scan.pending_end_reason : FuriHalBtScanEndedReasonStopped;

    if(g_scan.ended_callback) {
        FuriHalBtScanStats final_stats;
        furi_hal_bt_scan_stats(&final_stats);
        g_scan.dispatch_owner = furi_thread_get_current_id();
        g_scan.ended_callback(reason, &final_stats, g_scan.ended_context);
        g_scan.dispatch_owner = NULL;
    }

    g_scan.callback = NULL;
    g_scan.context = NULL;
    g_scan.queue = NULL;
    g_scan.ended_callback = NULL;
    g_scan.ended_context = NULL;
    g_scan.owner_thread = NULL;
    g_scan.pending_end_set = false;

    __atomic_store_n(&g_scan.phase, ScanPhaseIdle, __ATOMIC_RELEASE);

    furi_mutex_release(g_scan.dispatch_mutex);
    furi_mutex_release(g_scan.control_mutex);
    return FuriHalBtScanErrorNone;
}
```

- [ ] **Step 5: Implement furi_hal_bt_is_scanning and furi_hal_bt_scan_force_stop**

Replace the two stubs:
```c
bool furi_hal_bt_is_scanning(void) {
    ensure_init();
    ScanPhase p = (ScanPhase)__atomic_load_n(&g_scan.phase, __ATOMIC_ACQUIRE);
    return p == ScanPhaseActive || p == ScanPhaseStarting;
}

void furi_hal_bt_scan_force_stop(void) {
    /* Same as stop_scan but always sets pending_end_reason to ForceStoppedForReinit
     * if not already set. Skips the HCI disable when called from reinit context
     * (reinit is about to reset the controller anyway), but it's safe to issue it. */
    ensure_init();

    if(g_scan.dispatch_owner == furi_thread_get_current_id()) return;

    furi_mutex_acquire(g_scan.control_mutex, FuriWaitForever);

    if((ScanPhase)__atomic_load_n(&g_scan.phase, __ATOMIC_ACQUIRE) != ScanPhaseActive) {
        furi_mutex_release(g_scan.control_mutex);
        return;
    }

    __atomic_store_n(&g_scan.phase, ScanPhaseStopping, __ATOMIC_RELEASE);
    hci_le_set_scan_enable(0, 0);
    furi_mutex_acquire(g_scan.dispatch_mutex, FuriWaitForever);

    if(g_scan.adv_was_paused) {
        gap_resume_advertising(&g_scan.saved_adv);
        g_scan.adv_was_paused = false;
    }
    if(g_scan.allowlist.list) {
        free(g_scan.allowlist.list);
        g_scan.allowlist.list = NULL;
        g_scan.allowlist.len = 0;
    }

    if(!g_scan.pending_end_set) {
        g_scan.pending_end_reason = FuriHalBtScanEndedReasonForceStoppedForReinit;
        g_scan.pending_end_set = true;
    }
    if(g_scan.ended_callback) {
        FuriHalBtScanStats final_stats;
        furi_hal_bt_scan_stats(&final_stats);
        g_scan.dispatch_owner = furi_thread_get_current_id();
        g_scan.ended_callback(g_scan.pending_end_reason, &final_stats, g_scan.ended_context);
        g_scan.dispatch_owner = NULL;
    }

    g_scan.callback = NULL;
    g_scan.context = NULL;
    g_scan.queue = NULL;
    g_scan.ended_callback = NULL;
    g_scan.ended_context = NULL;
    g_scan.owner_thread = NULL;
    g_scan.pending_end_set = false;

    __atomic_store_n(&g_scan.phase, ScanPhaseIdle, __ATOMIC_RELEASE);

    furi_mutex_release(g_scan.dispatch_mutex);
    furi_mutex_release(g_scan.control_mutex);
}
```

- [ ] **Step 6: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds. If `hci_le_set_scan_parameters`, `hci_le_set_scan_enable`, or the dispatcher API names differ from this plan's placeholders, fix the symbol names; do not change the contract.

- [ ] **Step 7: On-device smoke test**

On `carbon`: flash. Write a tiny test FAP (10 lines) that calls `furi_hal_bt_start_scan` with a callback that increments a counter, runs for 5 s, then calls stop_scan. Verify counter > 0 (you should pick up advertising from your phone). This is the first end-to-end sanity check; do not skip.

- [ ] **Step 8: Commit**

Run:
```
./fbt format
git add targets/f7/ble_glue/furi_hal_bt_scan.c
git commit -m "furi_hal_bt: wire scan event dispatch (callback variant)

Activates the callback variant of furi_hal_bt_start_scan. State
machine transitions Idle->Starting->Active; HCI advertising reports
dispatched to the registered callback under dispatch_mutex with
release/acquire phase ordering. F1 mitigation: thread-liveness check
on owner_thread before each dispatch — orphaned scans auto-stop on
the next event."
```

---

### Task 7: furi_hal_bt: add queued variant with flow-control backpressure

Wire `furi_hal_bt_start_scan_queued` and `furi_hal_bt_scan_resume_flow`. Adds the `BleEventAckFlowDisable` backpressure path when the FAP's queue fills.

**Files:**
- Modify: `targets/f7/ble_glue/furi_hal_bt_scan.c`

- [ ] **Step 1: Implement furi_hal_bt_start_scan_queued**

Add to `furi_hal_bt_scan.c`. Most of the body is identical to `furi_hal_bt_start_scan`; refactor to share a helper:

```c
/* Internal: shared start logic */
static FuriHalBtScanError start_scan_internal(
    const FuriHalBtScanConfig* config,
    ScanMode mode,
    FuriHalBtScanCallback callback, void* context,
    FuriMessageQueue* queue) {
    /* Re-entry guard, validate, alloc, populate state, configure controller —
     * everything currently in furi_hal_bt_start_scan, parameterized by mode/dest. */
    ensure_init();
    if(g_scan.dispatch_owner == furi_thread_get_current_id()) {
        return FuriHalBtScanErrorReentrant;
    }
    if(mode == ScanModeCallback && !callback) return FuriHalBtScanErrorInvalidParameter;
    if(mode == ScanModeQueued) {
        if(!queue) return FuriHalBtScanErrorInvalidParameter;
        if(furi_message_queue_get_message_size(queue) != sizeof(FuriHalBtScanReport)) {
            return FuriHalBtScanErrorInvalidParameter;
        }
    }
    /* ... rest of furi_hal_bt_start_scan body, with:
     *   g_scan.mode = mode;
     *   g_scan.callback = (mode == ScanModeCallback) ? callback : NULL;
     *   g_scan.queue = (mode == ScanModeQueued) ? queue : NULL;
     */
}
```

Then both public entries become thin wrappers:
```c
FuriHalBtScanError furi_hal_bt_start_scan(
    const FuriHalBtScanConfig* config,
    FuriHalBtScanCallback callback,
    void* context) {
    return start_scan_internal(config, ScanModeCallback, callback, context, NULL);
}

FuriHalBtScanError furi_hal_bt_start_scan_queued(
    const FuriHalBtScanConfig* config,
    FuriMessageQueue* report_queue) {
    return start_scan_internal(config, ScanModeQueued, NULL, NULL, report_queue);
}
```

Move the body from Task 6's `furi_hal_bt_start_scan` into `start_scan_internal`. Replace the dispatch site in `furi_hal_bt_start_scan` proper with the wrapper above.

- [ ] **Step 2: Update on_adv_report to handle queued mode + backpressure**

In `on_adv_report`, replace the dispatch block with:
```c
    BleEventFlowStatus rc_flow = BleEventAckFlowEnable;
    if(g_scan.mode == ScanModeCallback && g_scan.callback) {
        g_scan.dispatch_owner = furi_thread_get_current_id();
        g_scan.callback(&report, g_scan.context);
        g_scan.dispatch_owner = NULL;
        __atomic_fetch_add(&g_scan.stats.events_delivered, 1, __ATOMIC_RELAXED);
    } else if(g_scan.mode == ScanModeQueued && g_scan.queue) {
        FuriStatus put_st = furi_message_queue_put(g_scan.queue, &report, 0);
        if(put_st == FuriStatusOk) {
            __atomic_fetch_add(&g_scan.stats.events_delivered, 1, __ATOMIC_RELAXED);
        } else {
            /* Queue full → backpressure */
            __atomic_fetch_add(&g_scan.stats.events_dropped, 1, __ATOMIC_RELAXED);
            if(!g_scan.flow_disabled) {
                g_scan.flow_disabled = true;
                g_scan.flow_disabled_since_ms = furi_get_tick();
            }
            rc_flow = BleEventAckFlowDisable;
        }
    }

    furi_mutex_release(g_scan.dispatch_mutex);
    return rc_flow;
```

- [ ] **Step 3: Implement furi_hal_bt_scan_resume_flow**

Replace the stub:
```c
void furi_hal_bt_scan_resume_flow(void) {
    ensure_init();
    furi_mutex_acquire(g_scan.dispatch_mutex, FuriWaitForever);
    if(g_scan.flow_disabled) {
        g_scan.flow_disabled = false;
        SVCCTL_ResumeUserEventFlow();
    }
    furi_mutex_release(g_scan.dispatch_mutex);
}
```

`SVCCTL_ResumeUserEventFlow()` is already defined in `targets/f7/ble_glue/ble_app.c:167` — verified during the spec phase.

- [ ] **Step 4: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds. Confirm `SVCCTL_ResumeUserEventFlow` is reachable; if not, include the appropriate header (`ble_app.h` or similar).

- [ ] **Step 5: On-device test**

On `carbon`: write a small test FAP that uses `furi_hal_bt_start_scan_queued` with a tiny depth-4 queue. Don't drain it; observe `events_dropped` increment via `furi_hal_bt_scan_stats()`. Drain the queue, call `furi_hal_bt_scan_resume_flow()`, verify reports start arriving again. End-to-end backpressure verification.

- [ ] **Step 6: Commit**

Run:
```
./fbt format
git add targets/f7/ble_glue/furi_hal_bt_scan.c
git commit -m "furi_hal_bt: add queued scan variant with flow-control backpressure

furi_hal_bt_start_scan_queued delivers reports by-value to a
caller-owned FuriMessageQueue. On queue full, returns
BleEventAckFlowDisable to controller (stops all HCI dispatch globally
until SVCCTL_ResumeUserEventFlow). FAP must drain queue and call
furi_hal_bt_scan_resume_flow to recover."
```

---

### Task 8: furi_hal_bt: ScanEnded callback + duration timer + backpressure timeout

Three integrations: (1) duration_ms auto-stop, (2) backpressure timeout auto-stop, (3) the `ScanEnded` callback fires correctly with the right reason in all paths. The `BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP` bit drives both auto-stop sources.

**Files:**
- Modify: `targets/f7/ble_glue/ble_event_thread.c` (add new flag bit + dispatch)
- Modify: `targets/f7/ble_glue/ble_event_thread.h` (export the flag bit if needed)
- Modify: `targets/f7/ble_glue/furi_hal_bt_scan.c`

- [ ] **Step 1: Add the flag bit and dispatch in ble_event_thread.c**

Edit `targets/f7/ble_glue/ble_event_thread.c`. Find the existing flag bit definitions (around line 12-18):
```c
#define BLE_EVENT_THREAD_FLAG_SHCI_EVENT  (1UL << 0)
#define BLE_EVENT_THREAD_FLAG_HCI_EVENT   (1UL << 1)
#define BLE_EVENT_THREAD_FLAG_KILL_THREAD (1UL << 2)
```
Add:
```c
#define BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP (1UL << 3)
```
Update the `_ALL` mask to include the new bit:
```c
#define BLE_EVENT_THREAD_FLAG_ALL                                              \
    (BLE_EVENT_THREAD_FLAG_SHCI_EVENT | BLE_EVENT_THREAD_FLAG_HCI_EVENT |      \
     BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP | BLE_EVENT_THREAD_FLAG_KILL_THREAD)
```

In the thread loop body (around line 26-41), add a dispatch case:
```c
        if(flags & BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP) {
            /* Forward decl in ble_event_thread.c is fine; no header dep */
            extern void furi_hal_bt_scan_force_stop(void);
            furi_hal_bt_scan_force_stop();
        }
```

- [ ] **Step 2: Export a helper to get the thread ID (if not already exposed)**

Edit `targets/f7/ble_glue/ble_event_thread.h`:
```c
/* Used by furi_hal_bt_scan.c to signal SCAN_AUTOSTOP */
FuriThread* ble_event_thread_get_thread(void);
```

Edit `targets/f7/ble_glue/ble_event_thread.c`. Add at the bottom:
```c
FuriThread* ble_event_thread_get_thread(void) {
    return event_thread;
}
```

(This may already exist; if so, skip this step.)

- [ ] **Step 3: Update on_adv_report to detect backpressure timeout**

In `furi_hal_bt_scan.c`, in `on_adv_report`, just after the phase check, add:

```c
    /* Backpressure timeout check */
    if(should_force_stop_backpressure(
        g_scan.flow_disabled, furi_get_tick(),
        g_scan.flow_disabled_since_ms,
        FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS)) {
        if(!g_scan.pending_end_set) {
            g_scan.pending_end_reason = FuriHalBtScanEndedReasonBackpressureTimeout;
            g_scan.pending_end_set = true;
        }
        furi_thread_flags_set(
            furi_thread_get_id(ble_event_thread_get_thread()),
            BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP);
    }
```

- [ ] **Step 4: Add the duration_timer infrastructure**

In `furi_hal_bt_scan.c`, add the timer callback near the top (above `start_scan_internal`):
```c
static void scan_duration_timer_callback(void* context) {
    UNUSED(context);
    /* Set pending reason under dispatch_mutex; signal autostop. */
    if(furi_mutex_acquire(g_scan.dispatch_mutex, 0) == FuriStatusOk) {
        if(!g_scan.pending_end_set) {
            g_scan.pending_end_reason = FuriHalBtScanEndedReasonDurationExpired;
            g_scan.pending_end_set = true;
        }
        furi_mutex_release(g_scan.dispatch_mutex);
    }
    furi_thread_flags_set(
        furi_thread_get_id(ble_event_thread_get_thread()),
        BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP);
}
```

In `start_scan_internal`, after the controller is enabled and before releasing control_mutex, add:
```c
    if(config->duration_ms > 0) {
        g_scan.duration_timer = furi_timer_alloc(
            scan_duration_timer_callback, FuriTimerTypeOnce, NULL);
        furi_timer_start(g_scan.duration_timer, furi_ms_to_ticks(config->duration_ms));
    } else {
        g_scan.duration_timer = NULL;
    }
```

In `furi_hal_bt_stop_scan` and `furi_hal_bt_scan_force_stop`, just after acquiring `dispatch_mutex`, add:
```c
    if(g_scan.duration_timer) {
        furi_timer_stop(g_scan.duration_timer);
        furi_timer_free(g_scan.duration_timer);
        g_scan.duration_timer = NULL;
    }
```

- [ ] **Step 5: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds.

- [ ] **Step 6: On-device test**

On `carbon`:
- Test 1 (duration auto-stop): start scan with `cfg.duration_ms = 3000` and an `ended_callback` that sets a flag. Verify the flag is set ~3 s later and the reason is `DurationExpired`.
- Test 2 (backpressure auto-timeout): start_scan_queued with depth-4 queue, never drain. Verify ended_callback fires within ~5 s with reason `BackpressureTimeout`.

- [ ] **Step 7: Commit**

Run:
```
./fbt format
git add targets/f7/ble_glue/ble_event_thread.c \
        targets/f7/ble_glue/ble_event_thread.h \
        targets/f7/ble_glue/furi_hal_bt_scan.c
git commit -m "furi_hal_bt: add ScanEnded callback, duration timer, backpressure timeout

Three asynchronous termination paths converge on a single mechanism:
the SCAN_AUTOSTOP flag bit on ble_event_thread, with end-reason
resolved via pending_end_reason (first-write-wins under dispatch_mutex).

- duration_ms timer fires in FuriTimer thread, defers HCI work to
  ble_event_thread (FuriTimer stack too small for synchronous HCI).
- Backpressure timeout: if flow-disabled state persists more than
  5000 ms, force-stop with BackpressureTimeout reason.
- ScanEnded callback always fires exactly once per session with the
  resolved reason."
```

---

### Task 9: furi_hal_bt: expose scan stats and update api_symbols.csv

Stats and the HCI status query function are already implemented in earlier tasks. This commit makes the public-API additions visible to FAPs by updating `api_symbols.csv`. Bumps version 87.7 → 87.8.

**Files:**
- Modify: `targets/f7/api_symbols.csv`

- [ ] **Step 1: Find the current version and the existing furi_hal_bt_* entry block**

Run:
```
head -3 targets/f7/api_symbols.csv
grep -n '^Function,+,furi_hal_bt_' targets/f7/api_symbols.csv | head -5
grep -n '^Function,+,furi_hal_bt_' targets/f7/api_symbols.csv | tail -5
```
Expected: Version line is `Version,+,87.7,,`. The `furi_hal_bt_*` cluster is around lines 1392-1410 (varies). Note the actual range.

- [ ] **Step 2: Bump the version**

Edit `targets/f7/api_symbols.csv`. Change:
```
Version,+,87.7,,
```
to:
```
Version,+,87.8,,
```

- [ ] **Step 3: Add the new Header entry in sorted position**

In `targets/f7/api_symbols.csv`, find the alphabetically correct position for `targets/f7/ble_glue/furi_hal_bt_scan.h` among the existing `Header,+,...` entries and insert:
```
Header,+,targets/f7/ble_glue/furi_hal_bt_scan.h,,
```

- [ ] **Step 4: Add the function entries in sorted position**

Insert these in the `furi_hal_bt_*` Function cluster, in alphabetical order:
```
Function,+,furi_hal_bt_is_scanning,_Bool,
Function,+,furi_hal_bt_scan_force_stop,void,
Function,+,furi_hal_bt_scan_last_hci_status,uint8_t,
Function,+,furi_hal_bt_scan_resume_flow,void,
Function,+,furi_hal_bt_scan_stats,void,FuriHalBtScanStats*
Function,+,furi_hal_bt_scan_stats_reset,void,
Function,+,furi_hal_bt_start_scan,FuriHalBtScanError,"const FuriHalBtScanConfig*, FuriHalBtScanCallback, void*"
Function,+,furi_hal_bt_start_scan_queued,FuriHalBtScanError,"const FuriHalBtScanConfig*, FuriMessageQueue*"
Function,+,furi_hal_bt_stop_scan,FuriHalBtScanError,
```

`gap_has_active_connection` and `gap_get_connection_interval_ms` are NOT exported (private to the patch).

- [ ] **Step 5: Verify the CSV is still valid sorted order**

Run:
```
awk -F, '/^Function,/ { print $3 }' targets/f7/api_symbols.csv > /tmp/funcs_actual.txt
sort -u /tmp/funcs_actual.txt > /tmp/funcs_sorted.txt
diff /tmp/funcs_actual.txt /tmp/funcs_sorted.txt | head -20
```
Expected: Empty diff. If non-empty, the existing CSV may not be strictly sorted — match its actual ordering rather than fighting it. Adjust your insertions to match the surrounding pattern.

- [ ] **Step 6: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds. The FAP loader should now permit FAPs to import the new symbols.

- [ ] **Step 7: Build a tiny test FAP that uses one of the new symbols**

Create a throwaway FAP at `applications_user/scan_smoke/` that calls `furi_hal_bt_is_scanning()` and prints the result. Build with `ufbt`. Verify the FAP loads on-device without symbol-resolution errors. This confirms the api_symbols.csv update is correct.

- [ ] **Step 8: Commit**

Run:
```
./fbt format
git add targets/f7/api_symbols.csv
git commit -m "furi_hal_bt: expose scan API in api_symbols.csv (v87.7 -> v87.8)

Adds the public scan API entry points so FAPs can link against them.
Additive minor version bump per Unleashed CSV scheme.

gap_has_active_connection and gap_get_connection_interval_ms remain
private (gap_internal.h, not in api_symbols.csv) — they exist for the
scan HAL's internal use only. FAPs needing connection state should
use existing furi_hal_bt_is_active()."
```

---

### Task 10: furi_hal_bt: install force_stop hook in furi_hal_bt_reinit

`furi_hal_bt_reinit()` resets the BLE coprocessor; if a scan is running, the existing scan state would point to a destroyed controller. Hook our force-stop into the reinit path.

**Files:**
- Modify: `targets/f7/furi_hal/furi_hal_bt.c`

- [ ] **Step 1: Locate furi_hal_bt_reinit**

Run: `grep -n 'void furi_hal_bt_reinit' targets/f7/furi_hal/furi_hal_bt.c`
Expected: Line ~202 per spec; confirm.

- [ ] **Step 2: Add the include**

At the top of `targets/f7/furi_hal/furi_hal_bt.c`, ensure:
```c
#include <furi_hal_bt_scan.h>
```
is present (likely already there transitively via `furi_hal_bt.h`; check first to avoid duplicates).

- [ ] **Step 3: Patch furi_hal_bt_reinit**

Find:
```c
void furi_hal_bt_reinit(void) {
```
Insert immediately after the opening brace:
```c
    /* Stop any active scan before resetting controller */
    furi_hal_bt_scan_force_stop();
```

- [ ] **Step 4: Verify build**

Run: `./fbt firmware_all`
Expected: Build succeeds.

- [ ] **Step 5: On-device test**

On `carbon`: write a test FAP that starts a scan, then triggers `furi_hal_bt_reinit()` (e.g., via `Settings → Bluetooth → Forget all paired devices` if that route uses reinit; or directly call the function). Verify the scan ends cleanly with `ended_callback` reason `ForceStoppedForReinit`, no hardfault.

- [ ] **Step 6: Commit**

Run:
```
./fbt format
git add targets/f7/furi_hal/furi_hal_bt.c
git commit -m "furi_hal_bt: install force_stop hook in furi_hal_bt_reinit

Without this, calling furi_hal_bt_reinit while a scan is active leaves
the scan state pointing to a destroyed controller — next event
dispatch hardfaults. The hook stops the scan cleanly with reason
ForceStoppedForReinit before the controller resets."
```

---

### Task 11: applications/debug: add ble_scan_test FAP and unit_tests App() block

Two artifacts in one commit since they together constitute "the test deliverable": (a) the test FAP under `applications/debug/ble_scan_test/`, (b) the new unit_tests PLUGIN App() block under `applications/debug/unit_tests/`.

**Files:**
- Create: `applications/debug/ble_scan_test/application.fam`
- Create: `applications/debug/ble_scan_test/ble_scan_test.c`
- Create: `applications/debug/ble_scan_test/ble_scan_test.h`
- Create: `applications/debug/ble_scan_test/views/menu_view.{c,h}`
- Create: `applications/debug/ble_scan_test/views/results_view.{c,h}`
- Create: `applications/debug/ble_scan_test/tests/t01_happy_callback.c` through `t26_thread_liveness_mitigation.c`
- Create: `applications/debug/ble_scan_test/README.md`
- Create: `applications/debug/unit_tests/tests/furi_hal_bt_scan/test_validate_config.c`
- Create: `applications/debug/unit_tests/tests/furi_hal_bt_scan/test_allowlist_permits.c`
- Create: `applications/debug/unit_tests/tests/furi_hal_bt_scan/test_backpressure_timeout.c`
- Modify: `applications/debug/unit_tests/application.fam`

- [ ] **Step 1: Add the unit_tests App() block**

Edit `applications/debug/unit_tests/application.fam`. Find the existing PLUGIN block for `test_furi_hal` or `test_bt` (around line 41-46). Append a new block in the same style:
```python
App(
    appid="test_furi_hal_bt_scan",
    apptype=FlipperAppType.PLUGIN,
    entry_point="test_furi_hal_bt_scan_init",
    cdefines=["APP_UNIT_TESTS"],
    requires=["unit_tests"],
    sources=["tests/furi_hal_bt_scan/*.c"],
    fap_libs=["assets"],
    fal_embedded=True,
)
```

Match the exact field names/values used by `test_furi_hal` in case Unleashed's conventions differ from the spec.

- [ ] **Step 2: Create the unit-test sources**

Create `applications/debug/unit_tests/tests/furi_hal_bt_scan/test_validate_config.c` with the body from Appendix A of the spec. All 6 `MU_TEST` cases plus a registration function:
```c
#include <furi.h>
#include "../minunit.h"
#include <furi_hal_bt_scan.h>
#include "../../../../targets/f7/ble_glue/furi_hal_bt_scan_internal.h"

MU_TEST(test_validate_null_config) { /* ... per Appendix A ... */ }
MU_TEST(test_validate_window_gt_interval) { /* ... */ }
MU_TEST(test_validate_struct_size_below_min) { /* ... */ }
MU_TEST(test_validate_struct_size_above_max) { /* ... */ }
MU_TEST(test_validate_etsi_duty_cycle) { /* ... */ }
MU_TEST(test_validate_allowlist_len_with_null_list) { /* ... */ }
MU_TEST(test_validate_default_config_ok) { /* ... */ }

MU_TEST_SUITE(test_validate_config_suite) {
    MU_RUN_TEST(test_validate_null_config);
    MU_RUN_TEST(test_validate_window_gt_interval);
    MU_RUN_TEST(test_validate_struct_size_below_min);
    MU_RUN_TEST(test_validate_struct_size_above_max);
    MU_RUN_TEST(test_validate_etsi_duty_cycle);
    MU_RUN_TEST(test_validate_allowlist_len_with_null_list);
    MU_RUN_TEST(test_validate_default_config_ok);
}
```

Create `test_allowlist_permits.c` with the 4 cases from Appendix A (empty, single match, single reject, multi-entry).

Create `test_backpressure_timeout.c` with the 5 cases from Appendix A (not_disabled, under_timeout, at_timeout, wraparound, disabled_with_zero_since).

Create the entry point in a new file `applications/debug/unit_tests/tests/furi_hal_bt_scan/test_furi_hal_bt_scan.c`:
```c
#include "../minunit.h"

MU_TEST_SUITE_DECLARE(test_validate_config_suite);
MU_TEST_SUITE_DECLARE(test_allowlist_permits_suite);
MU_TEST_SUITE_DECLARE(test_backpressure_timeout_suite);

void test_furi_hal_bt_scan(void) {
    MU_RUN_SUITE(test_validate_config_suite);
    MU_RUN_SUITE(test_allowlist_permits_suite);
    MU_RUN_SUITE(test_backpressure_timeout_suite);
}

const PluginApi test_furi_hal_bt_scan_plugin_api = {
    .name = "furi_hal_bt_scan",
    .test = test_furi_hal_bt_scan,
};

const FlipperAppPluginDescriptor* test_furi_hal_bt_scan_init(void) {
    static const FlipperAppPluginDescriptor desc = {
        .appid = "test_furi_hal_bt_scan",
        .ep_api_version = 1,
        .entry_point = &test_furi_hal_bt_scan_plugin_api,
    };
    return &desc;
}
```

(Match the exact plugin-registration boilerplate that `test_furi_hal` uses; this is a template.)

- [ ] **Step 3: Verify unit tests compile**

Run: `./fbt firmware_all`
Expected: Build succeeds. The unit_tests app picks up the new tests via the App() block.

- [ ] **Step 4: Run unit tests on-device**

Flash on `carbon`. Open the unit_tests app, find the `furi_hal_bt_scan` suite, run it. Expected: all 16 tests pass (7 + 4 + 5).

- [ ] **Step 5: Create the test FAP scaffold**

Create `applications/debug/ble_scan_test/application.fam`:
```python
App(
    appid="ble_scan_test",
    name="BLE Scan Test",
    apptype=FlipperAppType.EXTERNAL,
    entry_point="ble_scan_test_app",
    requires=["gui"],
    stack_size=4 * 1024,
    fap_category="Debug",
    fap_icon="icon.png",
    fap_author="timFinn",
    fap_version="0.1",
    fap_description="Verification harness for furi_hal_bt_scan API.",
)
```

Create `applications/debug/ble_scan_test/ble_scan_test.h` with the test-runner type definitions and forward declarations.

Create `applications/debug/ble_scan_test/ble_scan_test.c` with:
- App entry point
- ViewDispatcher + SceneManager initialization
- Menu scene (test selection)
- Results scene (pass/fail display, log scroll)
- **Critical: scene-exit handler MUST call `furi_hal_bt_stop_scan()`** to prevent F1 hardfault if the user exits mid-test

- [ ] **Step 6: Implement the 26 test cases**

For each test T1–T26 (per spec Appendix B), create a separate `tests/tNN_*.c` file with the test logic. Each file exports a function:
```c
TestResult t01_happy_callback(TestContext* ctx);
```
where `TestResult` is `{passed: bool, message: char*}` and `TestContext` provides logging + a way to report manual-verification prompts ("verify on phone scanner, press OK if observed").

The spec's section 5.2 table is the source of truth for procedure and expected result for each test. Follow it verbatim.

This step is large; break it into sub-steps if needed (each test file is ~30-100 lines). Recommended TDD ordering:
- Implement T1 (simplest happy path) first
- Verify the test FAP framework works end-to-end with just T1
- Then implement T2-T9 (simple automated cases)
- Then T10-T13 (manual-verification cases with prompts)
- Then T14-T16 (more involved cases)
- Then T17-T19 (the critical ones from agent review)
- Then T20-T26 (added during revision pass)

- [ ] **Step 7: Build and verify the test FAP**

Run from inside the unleashed repo:
```
./fbt fap_ble_scan_test
```
or use `ufbt` from inside `applications/debug/ble_scan_test/`. Expected: `.fap` artifact produced.

- [ ] **Step 8: Run the full test suite on `carbon`**

Flash the test FAP. Run all 26 tests. Document any failures. **All tests must pass before this commit lands** — failing tests indicate either a bug in the patch (Tasks 1-10) or a bug in the test itself.

The most likely-to-fail tests are:
- T17 (race stress): if mutex ordering has a subtle bug
- T18 (concurrent BLE consumer): if backpressure timeout interacts badly with BLE Serial
- T19 (controller edge values): may surface ETSI validation issues
- T26 (thread-liveness mitigation): if the F1 mitigation has bugs

If any test fails, fix the underlying issue in the appropriate earlier task's commit (use `git commit --amend` on the relevant commit) before completing this task.

- [ ] **Step 9: Create the test FAP README**

Create `applications/debug/ble_scan_test/README.md`:
```markdown
# ble_scan_test

Verification harness for `furi_hal_bt_scan` (the BLE observer-mode HAL).

## Scope

Strict test harness — NOT a user-facing scanner. The end-user scanner
is a separate FAP project at `applications_user/ble_scanner` (downstream).

## Important

This FAP MUST call `furi_hal_bt_stop_scan()` in its scene-exit handler.
Failing to do so will leave a scan running with stale callback pointers
and hardfault the device on the next adv packet (mitigated by the
in-dispatcher thread-liveness check, but still poor hygiene).

## Tests

T1–T26 per the design spec. Some tests require external state
verification (paired phone, qFlipper open, WiFi load source); these
display "verify and press OK" prompts.
```

- [ ] **Step 10: Final build + size check**

Run:
```
./fbt firmware_all
python3 scripts/fwsize.py 2>&1 | tee /tmp/fwsize_final.txt
```

Compare `__free_flash_start__` against the baseline from Task 0:
```
diff ~/baseline_flash.txt /tmp/fwsize_final.txt
```

Expected: total M4 flash growth is in the 1.5-4 KB range (per spec estimate). If growth exceeds 5 KB (1.25× upper estimate), investigate before committing — the spec's hard threshold is "estimate + 25%".

- [ ] **Step 11: Run full regression checklist**

Per Section 5.3 of the spec, on `carbon` (~15-20 min first pass):
1. qFlipper desktop pairing — connect, browse SD, disconnect.
2. Mobile app pairing — pair, verify GATT services.
3. BLE HID profiles (if used).
4. Extra beacon — AirTag emulation, verify external Apple device sees it.
5. Bluetooth on/off toggle — recovers cleanly.
6. Reboot recovery — BLE comes up cleanly.
7. `bt_debug_app` carrier/packet test.
8. One known third-party BLE FAP (e.g., BLE Spam) — verify no regression.

All checks must pass.

- [ ] **Step 12: Commit**

Run:
```
./fbt format
git add applications/debug/ble_scan_test/ \
        applications/debug/unit_tests/application.fam \
        applications/debug/unit_tests/tests/furi_hal_bt_scan/
git commit -m "applications/debug: add ble_scan_test FAP and furi_hal_bt_scan unit tests

Test FAP (FlipperAppType.EXTERNAL, applications/debug/ble_scan_test/)
implements the 26 on-device verification tests T1-T26 from the design
spec. Strict test harness; not a user-facing scanner.

Unit tests register a new test_furi_hal_bt_scan PLUGIN App() block
under applications/debug/unit_tests/, matching the test_furi_hal
precedent. Tests cover validate_config, allowlist_permits, and
should_force_stop_backpressure pure helpers."
```

- [ ] **Step 13: Push the completed branch**

Run:
```
git push origin feature/ble-scan-api
```

The branch is now ready for the optional separate-PR commits (CI workflow, loader-teardown hook) and/or upstream PR submission.

---

## Self-review checklist

After completing the plan, before declaring done:

- [ ] **Spec coverage:** every section of the spec is addressed by at least one task. Specifically verify: 11-commit series matches spec section 1.6; API surface in section 2.1 fully implemented; data flow in section 3 reflected in the implementation; error handling in section 4 implemented; tests in section 5 implemented.
- [ ] **All TDD steps actually present:** every task's "Implement" step is preceded by a "Write the failing test / Run to verify it fails" step where applicable. The pure helpers (Task 5+11) genuinely have unit tests; the integration code is verified via on-device test FAP (Task 11) since unit-testing concurrent BLE state would require a mock framework outside scope.
- [ ] **No placeholder text:** no TBD/TODO in any task body. Every code block is complete.
- [ ] **Type/symbol consistency:** types defined in earlier tasks (e.g., `FuriHalBtScanConfig`) are used consistently in later tasks. Function signatures match between header and implementation.
- [ ] **Build green at every commit:** Task 1 leaves a buildable system; so does Task 2 through Task 11. No commit ships dead code that won't compile.
- [ ] **Backout plan:** any commit can be reverted with `git revert` without breaking commits earlier in the series. (Reverting a later commit doesn't break an earlier one.)

---

## Out of scope for this plan

Explicitly NOT in this plan, deferred to separate PRs or future work:

1. **GitHub Actions CI workflow** — Unleashed has no firmware-build CI today. A separate optional commit could add `.github/workflows/build-and-test.yml`. Submit as good-faith offering alongside the upstream PR; reviewers may decline.
2. **flipper_application.c teardown hook** — single-line change to call `furi_hal_bt_scan_force_stop` from the FAP loader. Catches orphaned scans synchronously instead of waiting for the next event. Complementary to the in-dispatcher thread-liveness check (which is the always-on defense in this plan). Submit as separate optional PR.
3. **Host-portable unit-test runner** — current plan uses Unleashed's on-device PLUGIN test framework. A host-gcc runner (with FURI stubs) would speed development iteration but isn't promised in upstream PR scope. Fork-internal value; out of this plan.
4. **The actual user-facing BLE scanner FAP** — this plan only implements the HAL and its test harness. The user-facing scanner FAP (with classifier, capture, UI) is a separate downstream project at `applications_user/ble_scanner` (or similar), governed by its own spec.

---

## Implementation summary

11 commits, mapping to spec section 1.6 commit plan. ~80-110 individual TDD steps. Realistic implementation timeline: 2-4 evenings of focused work, plus on-device verification time on `carbon`.

Branch: `timFinn/unleashed-firmware:feature/ble-scan-api`, based on the latest `unlshd-*` release tag.

End of implementation plan.
