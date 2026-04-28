# Unleashed BLE Observer-Mode Scan API — Design Spec

**Date:** 2026-04-28
**Author:** timFinn
**Status:** Draft, ready for implementation
**Target:** [DarkFlippers/unleashed-firmware](https://github.com/DarkFlippers/unleashed-firmware) (private fork at `timFinn/unleashed-firmware`, designed for upstream PR consideration)
**Consults integrated:** embedded-engineer ×4 (API shape, guarding, data flow, errors+limitations, testing), rf-engineer ×1 (scan parameter defaults), source-verification ×2 (Momentum infrastructure, Unleashed CI/loader)

---

## Summary

Add a BLE observer-mode scanning HAL to Unleashed firmware. Exposes `furi_hal_bt_start_scan` (callback variant) and `furi_hal_bt_start_scan_queued` (queue variant) plus supporting functions. Surgical patch (~2-4 KB M4 code, no copro changes), 9 commits in a logical PR-ready series, with a test FAP and on-device unit tests bundled.

The patch unblocks downstream FAP development for BLE scanner / classifier / capture applications. ST's `BLE_Stack_light` already supports observer mode in the bundled coprocessor binary; this patch wires that capability through `furi_hal_bt` to FAP authors.

---

## 1. Architecture

### 1.1. Execution contexts

| Context | Owner | Role in patch |
|---|---|---|
| **ble_event_thread** | FURI (`ble_event_thread.c`, FuriThreadPriorityHigh, 1280 B stack) | Dispatches `HCI_LE_ADVERTISING_REPORT` events to our handler; holds `dispatch_mutex` during dispatch |
| **FAP control thread** | FAP author | Calls `start_scan` / `stop_scan`; serialized by `control_mutex` |
| **FuriTimer thread** | FURI (system timer service, ~1 KB stack) | Fires `duration_ms` auto-stop timer; **does not call stop_scan directly** (would blow stack on synchronous HCI); sets a flag observed by ble_event_thread |
| **Diagnostics readers** | Anyone | Reads `furi_hal_bt_scan_stats()` — lock-free via word-aligned `__atomic_load` |

### 1.2. Files added

```
targets/f7/ble_glue/furi_hal_bt_scan.c          NEW — state machine, dispatch
targets/f7/ble_glue/furi_hal_bt_scan.h          NEW — public API
targets/f7/ble_glue/gap_internal.h              NEW — private gap.c extensions
applications/debug/ble_scan_test/               NEW — test FAP (commit 8)
applications/debug/unit_tests/tests/furi_hal_bt_scan/   NEW — unit tests (commit 9)
```

### 1.3. Files modified

```
targets/furi_hal_include/furi_hal_bt.h          MODIFIED — include new header
targets/f7/ble_glue/gap.c                       MODIFIED — add GAP_OBSERVER_ROLE; add accessors; add private pause/resume
targets/f7/ble_glue/gap.h                       MODIFIED — declare public read accessors
targets/f7/furi_hal/furi_hal_bt.c               MODIFIED — call force_stop_for_reinit at top of furi_hal_bt_reinit
targets/f7/api_symbols.csv                      MODIFIED — version 87.7 → 87.8; add new entries
```

Total touched files: **10** (5 new, 5 modified). The `api_symbols.csv` touch is required by Unleashed's FAP ABI — every public symbol must be listed or FAPs can't link against it at load time.

### 1.4. Scan state machine

```
   Idle ──start_scan──▶ Starting ──controller OK──▶ Active
    ▲                      │                          │
    │                      └──controller reject──┐    │
    │                                            ▼    │
    └─────────────── Idle ◀──stop_scan────── Stopping
                              (dispatch_mutex join + HCI disable sync)
```

- **Idle** — no scan registered. `callback`, `context`, `queue`, `allowlist.list` are NULL.
- **Starting** — `start_scan` has acquired `control_mutex`, populated state, is waiting for HCI command-complete.
- **Active** — scan running. Event handler dispatches via callback or queue.
- **Stopping** — `stop_scan` has issued HCI disable, waiting for `dispatch_mutex` (the join barrier). Event handler drops events cleanly during this window.

Phase transitions use `__atomic_store_n(..., __ATOMIC_RELEASE)` and reads use `__atomic_load_n(..., __ATOMIC_ACQUIRE)`. Standard release/acquire pattern; ARMv7-M generates the right barriers via `arm-none-eabi-gcc`.

### 1.5. Lock hierarchy invariant

```
control_mutex MAY be acquired without holding any other scan lock.
dispatch_mutex MAY be acquired without holding any other scan lock.
When acquiring both: ALWAYS control_mutex first, then dispatch_mutex.
stop_scan and force_stop_for_reinit are the only functions that acquire both.
```

Any new code touching scan state must obey this ordering.

### 1.6. Commit plan (9 commits, PR-ready)

```
1. gap: add GAP_OBSERVER_ROLE to aci_gap_init role mask
2. gap: expose has_active_connection / get_connection_interval_ms accessors
3. gap: add internal advertising pause/resume helpers
4. furi_hal_bt: add observer scan HAL skeleton (state machine, no dispatch)
5. furi_hal_bt: implement scan event dispatch with flow control backpressure
6. furi_hal_bt: wire scan into existing BLE event dispatcher; add ScanEnded callback
7. furi_hal_bt: expose scan stats; update api_symbols.csv; force_stop_for_reinit hook
8. applications/debug: add ble_scan_test FAP for end-to-end verification
9. applications/debug/unit_tests: add furi_hal_bt_scan host/device unit tests
```

Each commit compiles standalone (`./fbt firmware_all` succeeds). Each commit run through `./fbt format` per Unleashed's `CODING_STYLE.md`. Bisect-friendly.

**Optional separate PR commits (good-faith offerings, not part of main series):**

- GitHub Actions workflow for build + tests (Unleashed has no firmware-build CI today)
- One-line `flipper_application.c` FAP-teardown hook (mitigates F1; loader API doesn't currently exist)

---

## 2. Components & API

### 2.1. Public HAL surface — `targets/f7/ble_glue/furi_hal_bt_scan.h`

```c
/**
 * @file furi_hal_bt_scan.h
 * BLE observer-mode scanning API.
 *
 * Included transitively via furi_hal_bt.h; FAP authors should #include <furi_hal_bt.h>.
 */

#pragma once

#include <furi.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Errors ──────────────────────────────────────────────────────── */

typedef enum {
    FuriHalBtScanErrorNone = 0,
    FuriHalBtScanErrorBusy,                /**< Scan already active */
    FuriHalBtScanErrorNotReady,            /**< BLE radio stack not started */
    FuriHalBtScanErrorInvalidParameter,    /**< Config validation failed */
    FuriHalBtScanErrorControllerReject,    /**< HCI command returned non-zero status; see out_hci_status */
    FuriHalBtScanErrorReentrant,           /**< Caller is the dispatch thread */
} FuriHalBtScanError;

/* ── Scan configuration ──────────────────────────────────────────── */

typedef enum {
    FuriHalBtScanTypePassive = 0,  /**< RX-only; no SCAN_REQ; lower power */
    FuriHalBtScanTypeActive = 1,   /**< Sends SCAN_REQ; captures scan-response; forces advertising pause */
} FuriHalBtScanType;

typedef enum {
    FuriHalBtScanEndedReasonStopped,                /**< User called stop_scan */
    FuriHalBtScanEndedReasonDurationExpired,        /**< duration_ms timer fired */
    FuriHalBtScanEndedReasonForceStoppedForReinit,  /**< furi_hal_bt_reinit during scan */
    FuriHalBtScanEndedReasonControllerFault,        /**< Controller error mid-session */
    FuriHalBtScanEndedReasonBackpressureTimeout,    /**< Queue stayed full > FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS */
} FuriHalBtScanEndedReason;

typedef struct FuriHalBtScanStats FuriHalBtScanStats;  /* fwd; see below */

/**
 * Optional terminal callback. Fires exactly once per session, on the
 * dispatch thread, after the last advertisement report callback (if any).
 * Same constraints as the report callback.
 */
typedef void (*FuriHalBtScanEndedCallback)(
    FuriHalBtScanEndedReason reason,
    const FuriHalBtScanStats* final_stats,
    void* context);

/**
 * Scan parameters.
 *
 * struct_size MUST be set to sizeof(FuriHalBtScanConfig). The HAL
 * validates this for forward-compat: future struct extensions will
 * read additional fields only if struct_size is large enough.
 *
 * If a GATT connection is active at start_scan time, window_ms is
 * internally clamped to min(window_ms, 20) and interval_ms is clamped
 * to max(interval_ms, 160) to avoid starving the connection.
 * Parameters are static for the session.
 *
 * filter_duplicates operates on advertiser address only at the
 * controller level; rotating-RPA devices (iPhones, AirTags) re-report
 * regardless.
 *
 * The HAL copies the allowlist at start_scan time; caller's memory
 * may be freed after start_scan returns.
 */
typedef struct {
    uint8_t  struct_size;            /**< = sizeof(FuriHalBtScanConfig) */
    FuriHalBtScanType type;
    uint16_t window_ms;              /**< Controller listen time per interval */
    uint16_t interval_ms;            /**< Period between scan windows; must be >= window_ms */
    uint32_t duration_ms;            /**< Auto-stop after this many ms; 0 = until stop_scan */
    bool     filter_duplicates;
    bool     rssi_only;              /**< If true, adv_data unpopulated (saves CPU) */
    const uint8_t (*allowlist)[6];   /**< NULL = accept all */
    uint8_t  allowlist_len;
    FuriHalBtScanEndedCallback ended_callback;  /**< NULL = no terminal notification */
    void*    ended_context;
} FuriHalBtScanConfig;

#define FURI_HAL_BT_SCAN_CONFIG_DEFAULT            \
    ((FuriHalBtScanConfig){                        \
        .struct_size = sizeof(FuriHalBtScanConfig),\
        .type = FuriHalBtScanTypePassive,          \
        .window_ms = 30,                           \
        .interval_ms = 100,                        \
        .duration_ms = 0,                          \
        .filter_duplicates = false,                \
        .rssi_only = false,                        \
        .allowlist = NULL,                         \
        .allowlist_len = 0,                        \
        .ended_callback = NULL,                    \
        .ended_context = NULL,                     \
    })

/* ── Scan reports ────────────────────────────────────────────────── */

#define FURI_HAL_BT_SCAN_ADV_MAX_LEN 31

/**
 * One advertising report delivered to consumers.
 *
 * Pure value type: no internal pointers. Safe to memcpy, store, or
 * send through message queues.
 *
 * For Resolvable Random Addresses (addr_type == 1, top two bits of
 * mac[5] == 0b10), mac is the RPA bytes verbatim. Cross-rotation
 * correlation requires the device's IRK (not exposed by this HAL).
 *
 * struct_size enables forward-compat: future fields appended at the
 * end will not break old FAP readers.
 */
typedef struct {
    uint8_t  struct_size;            /**< = sizeof(FuriHalBtScanReport) */
    uint8_t  mac[6];
    uint8_t  addr_type;              /**< 0 = public, 1 = random */
    int8_t   rssi;                   /**< dBm, ±6 dB uncalibrated */
    uint8_t  adv_data_len;           /**< 0..31 */
    uint8_t  adv_data[FURI_HAL_BT_SCAN_ADV_MAX_LEN];
} FuriHalBtScanReport;

/**
 * Scan report callback type.
 *
 * Invoked on the BLE event dispatch thread. MUST return in microseconds
 * (target < 1 ms p99, < 10 ms absolute). Must not:
 *   - call furi_hal_bt_start_scan / start_scan_queued / stop_scan
 *     (returns FuriHalBtScanErrorReentrant to protect you)
 *   - perform file I/O
 *   - use FURI_LOG_* macros
 *   - consume more than ~256 bytes of stack
 *
 * For non-trivial work, use the queued variant.
 *
 * report pointer is valid only for callback duration. memcpy if needed.
 */
typedef void (*FuriHalBtScanCallback)(
    const FuriHalBtScanReport* report, void* context);

/* ── Statistics ──────────────────────────────────────────────────── */

struct FuriHalBtScanStats {
    uint32_t events_received;        /**< Total HCI reports arrived at HAL */
    uint32_t events_filtered;        /**< Dropped by allowlist */
    uint32_t events_delivered;       /**< Passed to callback or queue */
    uint32_t events_dropped;         /**< Queue full (triggers backpressure) */
    uint32_t events_dropped_phase;   /**< Dropped due to state (starting/stopping) */
};

/**
 * Read current scan statistics.
 *
 * Individual uint32_t fields are atomically read. Cross-field
 * consistency is not guaranteed for a single call.
 */
FuriHalBtScanStats furi_hal_bt_scan_stats(void);

/**
 * Reset all scan statistics to zero.
 */
void furi_hal_bt_scan_stats_reset(void);

/* ── Start / stop ────────────────────────────────────────────────── */

/**
 * Start a BLE scan, delivering reports via callback on BLE event thread.
 *
 * @param config           Scan parameters. Required (non-NULL).
 * @param callback         Report handler.
 * @param context          Opaque pointer passed to callback.
 * @param out_hci_status   Optional out-param. On FuriHalBtScanErrorControllerReject,
 *                         populated with the underlying ST/Bluetooth-SIG status code.
 *                         May be NULL.
 * @return                 FuriHalBtScanErrorNone on success.
 */
FuriHalBtScanError furi_hal_bt_start_scan(
    const FuriHalBtScanConfig* config,
    FuriHalBtScanCallback callback,
    void* context,
    uint8_t* out_hci_status);

/**
 * Start a BLE scan, delivering reports by-value into a message queue.
 *
 * Preferred for FAPs doing non-trivial work per report. Queue receives
 * FuriHalBtScanReport values copied into its internal storage — no
 * lifetime concerns.
 *
 * If queue fills, HAL returns BleEventAckFlowDisable to controller;
 * reports resume after furi_hal_bt_scan_resume_flow().
 *
 * @param report_queue    Caller-owned. Element size must equal
 *                        sizeof(FuriHalBtScanReport).
 */
FuriHalBtScanError furi_hal_bt_start_scan_queued(
    const FuriHalBtScanConfig* config,
    FuriMessageQueue* report_queue,
    uint8_t* out_hci_status);

/**
 * Stop in-progress scan. Synchronous post-barrier.
 *
 * After this returns:
 *   - No callback (scan or ended) will be invoked again for this session.
 *   - No further pushes will occur to the report queue.
 *   - Caller may free context, ended_context, queue, and allowlist memory.
 *   - Any auto-paused advertising has been restored.
 *   - Stats remain queryable until next start_scan; not auto-reset.
 *
 * Idempotent: returns None if no scan is active.
 * Returns Reentrant if called from within a scan callback.
 */
FuriHalBtScanError furi_hal_bt_stop_scan(void);

/**
 * Resume event delivery after backpressure.
 *
 * Queued-variant consumers should call this after draining their queue
 * if events_dropped has incremented since the last call. Harmless to
 * call when not backpressured.
 */
void furi_hal_bt_scan_resume_flow(void);

/**
 * Query whether a scan is currently active.
 */
bool furi_hal_bt_is_scanning(void);

/**
 * Force-stop any active scan owned by the given thread.
 *
 * Intended for FAP-loader integration: when a FAP unloads, the loader
 * may call this with the FAP's thread ID to clean up scans the FAP
 * forgot to stop. Currently unused (loader doesn't expose unload
 * hooks); exposed for future loader-side patches.
 *
 * Fires ended_callback with reason ForceStoppedForReinit-equivalent.
 */
void furi_hal_bt_scan_force_stop_for_thread(FuriThreadId thread_id);

#ifdef __cplusplus
}
#endif
```

### 2.2. Private extensions to gap — `targets/f7/ble_glue/gap_internal.h`

```c
/**
 * @file gap_internal.h
 * Private GAP helpers used only by furi_hal_bt_scan.c.
 * Not part of the public HAL.
 */

#pragma once

#include <gap.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     was_active;
    bool     was_connectable;
    uint16_t saved_min_adv_interval;
    uint16_t saved_max_adv_interval;
} GapAdvertisingSavedState;

void gap_pause_advertising(GapAdvertisingSavedState* out_saved);
void gap_resume_advertising(const GapAdvertisingSavedState* saved);
```

### 2.3. Public additions to gap.h

```c
/** Whether a GATT peer is currently connected. */
bool gap_has_active_connection(void);

/** Active connection interval in ms. Returns false if no connection. */
bool gap_get_connection_interval_ms(uint16_t* out_interval_ms);
```

### 2.4. Internal scan module state — `furi_hal_bt_scan.c`

```c
typedef enum {
    ScanStatePhaseIdle = 0,
    ScanStatePhaseStarting,
    ScanStatePhaseActive,
    ScanStatePhaseStopping,
} ScanStatePhase;

typedef enum {
    ScanModeCallback,
    ScanModeQueued,
} ScanMode;

typedef struct {
    volatile uint32_t phase;
    ScanMode          mode;
    FuriHalBtScanCallback callback;
    void*             context;
    FuriMessageQueue* queue;
    FuriHalBtScanEndedCallback ended_callback;
    void*             ended_context;
    FuriThreadId      owner_thread;
    struct {
        uint8_t (*list)[6];
        uint8_t   len;
    } allowlist;
    GapAdvertisingSavedState saved_adv;
    bool              adv_was_paused;
    bool              flow_disabled;
    uint32_t          flow_disabled_since_ms;
    FuriMutex*        control_mutex;
    FuriMutex*        dispatch_mutex;
    FuriThreadId      dispatch_owner;
    FuriTimer*        duration_timer;
    FuriHalBtScanStats stats;
} ScanModule;

static ScanModule g_scan;
```

### 2.5. api_symbols.csv additions

```
Header,+,targets/f7/ble_glue/furi_hal_bt_scan.h,,
Function,+,furi_hal_bt_start_scan,FuriHalBtScanError,"const FuriHalBtScanConfig*, FuriHalBtScanCallback, void*, uint8_t*"
Function,+,furi_hal_bt_start_scan_queued,FuriHalBtScanError,"const FuriHalBtScanConfig*, FuriMessageQueue*, uint8_t*"
Function,+,furi_hal_bt_stop_scan,FuriHalBtScanError,
Function,+,furi_hal_bt_scan_resume_flow,void,
Function,+,furi_hal_bt_is_scanning,_Bool,
Function,+,furi_hal_bt_scan_stats,FuriHalBtScanStats,
Function,+,furi_hal_bt_scan_stats_reset,void,
Function,+,furi_hal_bt_scan_force_stop_for_thread,void,FuriThreadId
Function,+,gap_has_active_connection,_Bool,
Function,+,gap_get_connection_interval_ms,_Bool,uint16_t*
```

Plus version bump:
```diff
-Version,+,87.7,,
+Version,+,87.8,,
```

### 2.6. Compile-time configuration

```c
/* In furi_hal_bt_scan.c */

#define FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS  5000
```

If queue stays full longer than this, HAL force-stops the session and fires `ScanEnded(BackpressureTimeout)`. Prevents permanent-stall scenarios when a FAP hits a debugger breakpoint or otherwise stops draining.

---

## 3. Data flow

### 3.1. Happy path — full scan session lifecycle (queued variant)

```
Thread       Phase        Step
──────────────────────────────────────────────────────────────────────────
FAP          Idle         1. FAP allocates FuriMessageQueue (depth 32, element = sizeof(FuriHalBtScanReport))
FAP          Idle         2. FAP calls furi_hal_bt_start_scan_queued(&cfg, q, NULL)
FAP          Idle         3. → Re-entry check: dispatch_owner != current_thread_id → OK
FAP          Idle         4. → Acquire control_mutex (FuriWaitForever)
FAP          Idle         5. → Phase atomic-load → Idle, OK to proceed
FAP          Idle         6. → Validate config (struct_size, window<=interval, allowlist consistent) → OK
FAP          Idle         7. → Verify radio stack ready: furi_hal_bt_is_alive() → true
FAP          Idle         8. → Allocate allowlist copy (panic-on-fail per heap hook)
FAP          Idle         9. → Allocate duration_timer if cfg.duration_ms > 0 (panic-on-fail)
FAP          Idle        10. → Phase atomic-store → Starting (release barrier)
FAP          Starting    11. → Populate g_scan: mode=Queued, queue=q, ended_callback=cfg.ended_callback,
                              owner_thread=current_thread_id, etc.
FAP          Starting    12. → Check gap_has_active_connection() → false. Use cfg as-is.
FAP          Starting    13. → cfg.type==Passive → no advertising pause needed
FAP          Starting    14. → Add GAP_OBSERVER_ROLE to current role mask if needed (one-time init)
FAP          Starting    15. → hci_le_set_scan_parameters(passive, window=30ms, interval=100ms, ...) sync
FAP          Starting    16. → hci_le_set_scan_enable(1, filter_dup=0) sync → command-complete OK
FAP          Starting    17. → If cfg.duration_ms != 0: start FuriTimer
FAP          Starting    18. → Phase atomic-store → Active (release barrier)
FAP          Active      19. → Release control_mutex; return FuriHalBtScanErrorNone

(controller starts scanning channel 37, then 38, then 39, repeating)

ble_event   Active       20. ST stack receives ADV_IND; HCI dispatches HCI_LE_ADVERTISING_REPORT
ble_event   Active       21. → on_adv_report() invoked
ble_event   Active       22. → Try-acquire dispatch_mutex (timeout=0) → OK
ble_event   Active       23. → Phase plain-read (mutex provides ordering) → Active
ble_event   Active       24. → stats.events_received atomic-inc
ble_event   Active       25. → Build FuriHalBtScanReport on stack from evt fields
ble_event   Active       26. → allowlist_permits(report.mac) → true (no allowlist set)
ble_event   Active       27. → mode==Queued: furi_message_queue_put(q, &report, 0) → OK
ble_event   Active       28. → stats.events_delivered atomic-inc
ble_event   Active       29. → Release dispatch_mutex; return BleEventAckFlowEnable
ble_event   Active       30. → ST stack frees its event buffer

FAP         Active       31. (FAP's reader thread does furi_message_queue_get(q, &report, ...))
FAP         Active       32. (FAP processes: parse, classify, capture — all in FAP-thread context)

(steps 20-32 repeat for each adv packet)

FAP         Active       33. FAP decides to stop. Calls furi_hal_bt_stop_scan()
FAP         Active       34. → Re-entry check: dispatch_owner != current_thread_id → OK
FAP         Active       35. → Acquire control_mutex (FuriWaitForever)
FAP         Active       36. → Phase atomic-load → Active, OK to proceed
FAP         Active       37. → Phase atomic-store → Stopping (release)
FAP         Stopping    38. → Stop duration_timer if set; free it
FAP         Stopping    39. → hci_le_set_scan_enable(0, 0) sync — controller drains its buffers
FAP         Stopping    40. → Acquire dispatch_mutex (FuriWaitForever) — JOIN BARRIER
                              (no callback can be running; dispatch_mutex is the proof)
FAP         Stopping    41. → If adv_was_paused: gap_resume_advertising(&saved_adv)
FAP         Stopping    42. → Free allowlist copy
FAP         Stopping    43. → If ended_callback: invoke with reason=Stopped
FAP         Stopping    44. → NULL-out callback, context, queue, ended_callback (defensive)
FAP         Stopping    45. → Phase atomic-store → Idle (release)
FAP         Idle        46. → Release dispatch_mutex
FAP         Idle        47. → Release control_mutex; return FuriHalBtScanErrorNone

FAP         Idle        48. (FAP can now safely free its queue, context, ended_context, allowlist memory)
```

**Mutex hold times in happy path:**
- `control_mutex`: ~5 ms during start (HCI command sync); ~5-50 ms during stop (HCI sync + dispatch_mutex acquire).
- `dispatch_mutex`: 10–50 µs per event; held briefly by stop_scan during teardown.

### 3.2. Backpressure — queue full

```
Thread       Phase        Step
──────────────────────────────────────────────────────────────────────────
ble_event   Active       1. on_adv_report() invoked
ble_event   Active       2. → Acquire dispatch_mutex; phase==Active
ble_event   Active       3. → furi_message_queue_put(q, &report, 0) → ErrorResource (queue full)
ble_event   Active       4. → stats.events_dropped atomic-inc
ble_event   Active       5. → If !flow_disabled: set flow_disabled=true, flow_disabled_since_ms=now
ble_event   Active       6. → Release dispatch_mutex
ble_event   Active       7. → Return BleEventAckFlowDisable

(ST stack stops dispatching HCI events to us until SVCCTL_ResumeUserEventFlow)

FAP        Active        8. FAP's reader drains queue, processes reports
FAP        Active        9. FAP polls furi_hal_bt_scan_stats(); sees events_dropped > previous
FAP        Active       10. FAP calls furi_hal_bt_scan_resume_flow()
FAP        Active       11. → Acquire dispatch_mutex (brief)
FAP        Active       12. → flow_disabled = false
FAP        Active       13. → Call SVCCTL_ResumeUserEventFlow()
FAP        Active       14. → Release dispatch_mutex; return

(controller resumes dispatching HCI events to ble_event_thread)
```

### 3.3. Auto-stop on duration

FuriTimer thread has only ~1 KB stack — calling synchronous HCI directly would risk overflow and stalls the system Timer Service Task. Instead, the timer callback signals via thread flag.

```
Thread          Phase        Step
──────────────────────────────────────────────────────────────────────────
duration_timer  Active       1. FuriTimer fires after cfg.duration_ms
                Active       2. Timer callback: furi_thread_flags_set(ble_event_thread,
                                BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP)

ble_event_thread Active     3. Wakes on flag; observes SCAN_AUTOSTOP bit
                Active       4. → Calls furi_hal_bt_stop_scan() — runs the full stop sequence
                                (steps 33-47 from 3.1)
                Stopping    5. → ended_callback fires with reason=DurationExpired
                Idle        6. Returns
```

### 3.4. Backpressure timeout — auto-terminate

If `flow_disabled` has been set for longer than `FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS` (default 5000), the HAL force-stops the session.

```
Thread       Phase        Step
──────────────────────────────────────────────────────────────────────────
ble_event   Active       1. Each on_adv_report() invocation checks:
                              if (flow_disabled && (now - flow_disabled_since_ms) > TIMEOUT_MS)
ble_event   Active       2. → Set BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP
ble_event   Active       3. → Continue dispatch normally for this packet

ble_event_thread Active   4. Wakes on flag; calls force-stop sequence
                Stopping  5. → ended_callback fires with reason=BackpressureTimeout
                Idle      6. → Returns
```

Without this, a FAP that hits a debugger breakpoint or otherwise stops draining would stall all BLE indefinitely.

### 3.5. Race — stop_scan during event delivery

```
Thread       Phase        Step
──────────────────────────────────────────────────────────────────────────
ble_event   Active       1. on_adv_report() invoked
ble_event   Active       2. → Acquire dispatch_mutex; phase==Active
ble_event   Active       3. → mode==Callback: about to invoke FAP callback
ble_event   Active       4. → dispatch_owner = current_thread_id
ble_event   Active       5. → callback(&report, context) — runs synchronously, FAP code (50 µs)

(meanwhile, on FAP control thread:)
FAP         Active      6a. → furi_hal_bt_stop_scan()
FAP         Active      6b. → Acquire control_mutex — gets it
FAP         Active      6c. → Phase atomic-store → Stopping
FAP         Stopping    6d. → hci_le_set_scan_enable(0, 0) sync
FAP         Stopping    6e. → Try acquire dispatch_mutex... BLOCKS (event handler holds it)

ble_event   Stopping    7. → callback returns
ble_event   Stopping    8. → dispatch_owner = NULL
ble_event   Stopping    9. → Release dispatch_mutex

FAP         Stopping   6f. → dispatch_mutex acquired
FAP         Stopping   6g. → Free allowlist, fire ended_callback, NULL fields, phase → Idle
FAP         Idle       6h. → Release dispatch_mutex, control_mutex; return
```

**Invariant maintained**: between 6e and 9, the FAP's callback runs to completion. Context pointer is valid throughout (FAP can only free it after stop_scan returns at 6h).

### 3.6. Race — start_scan called twice

```
FAP-A      Idle      1. start_scan_queued() → succeeds → phase=Active
FAP-B      Active    2. start_scan() →
                         acquire control_mutex (waits if A still inside its start, then gets it),
                         phase==Active → release control_mutex,
                         return FuriHalBtScanErrorBusy
```

### 3.7. FAP exits without stop_scan (PATHOLOGICAL — DOCUMENTED LIMIT)

**Behavior:** callback function pointer or context pointer becomes stale; next event handler invocation jumps to invalid memory → **hard fault on ble_event_thread → device reboots**.

**Severity:** Critical. Hostile to debug — no breadcrumb pointing at the offending FAP. Next BLE-using app appears broken until reboot.

**v1 mitigation:** documented prohibition only. Loader API does not expose unload hooks (verified). The optional separate-PR upstream commit for a one-line `flipper_application.c` hook would resolve this.

### 3.8. Connection establishes mid-scan

Static-at-start scan parameters. If a phone connects to the Flipper's GATT server mid-scan, running 30/100 params may stress the connection. Documented limitation. FAP can detect via `gap_has_active_connection()` polling and restart with downshifted params if it cares.

### 3.9. Controller rejects scan setup

```
FAP        Starting   1. → hci_le_set_scan_parameters(...) sync → returns BLE_STATUS_INVALID_PARAMS
FAP        Starting   2. → If out_hci_status: *out_hci_status = status_code
FAP        Starting   3. → Free duration_timer if allocated
FAP        Starting   4. → Free allowlist copy
FAP        Starting   5. → Phase atomic-store → Idle
FAP        Idle       6. → Release control_mutex
FAP        Idle       7. → Return FuriHalBtScanErrorControllerReject
```

### 3.10. furi_hal_bt_reinit() called mid-scan

`furi_hal_bt_reinit()` calls `hci_reset()` and tears down the gap thread. Without our fix, scan state would point to a destroyed controller.

**Fix:** `furi_hal_bt_reinit()` calls `furi_hal_bt_scan_force_stop_for_reinit()` BEFORE `hci_reset()`.

```c
/* Patch to furi_hal_bt.c */
void furi_hal_bt_reinit(void) {
    furi_hal_bt_scan_force_stop_for_reinit();  /* NEW */
    /* ... existing code ... */
}
```

`force_stop_for_reinit()` is similar to `stop_scan()` but **skips the HCI disable** (controller is about to be reset) and just cleans up HAL state, then fires `ended_callback(ForceStoppedForReinit)`.

---

## 4. Error handling, failure modes, and limitations

### 4.1. Error code reference

| Error | Returned by | Cause | FAP recovery |
|---|---|---|---|
| `None` | All | Success | Proceed |
| `Busy` | `start_scan*` | Another scan session active | Wait, retry, or surface to user |
| `NotReady` | `start_scan*` | `furi_hal_bt_is_alive() == false` | Caller should ensure radio stack started; usually a startup-order bug |
| `InvalidParameter` | `start_scan*` | Validation failed: window > interval, allowlist_len with NULL list, struct_size mismatch, queue NULL or wrong element size | Bug in caller config; not retryable |
| `ControllerReject` | `start_scan*` | HCI command returned non-zero status. `out_hci_status` (if provided) carries the ST/Bluetooth-SIG code | Surface to user; retry with FURI_HAL_BT_SCAN_CONFIG_DEFAULT |
| `Reentrant` | `start_scan*`, `stop_scan` | Caller is the dispatch thread (callback context) | Bug; restructure to post a message and call from FAP's own thread |

### 4.2. Non-error observable conditions

- **events_dropped > 0**: queue full, controller backpressured. FAP must call `furi_hal_bt_scan_resume_flow()` after draining.
- **events_filtered > 0**: allowlist working as intended.
- **events_dropped_phase > 0**: events arriving during state transitions; near-zero in steady state.
- **No advertisements observed**: not an error. Could be empty environment, allowlist over-restrictive, scan duty cycle insufficient.
- **Resolvable Random Address rotation**: documented BLE privacy feature; FAP cannot correlate across rotations without IRK.
- **RSSI ±6 dB absolute uncalibrated**: relative comparisons valid; distance estimates noisy.

### 4.3. Failure modes that bypass the API

#### F1: FAP exits without calling stop_scan (Critical)

Callback or context pointer stale → hard fault on next event → device reboots. **Mitigation:** documented prohibition only in v1.

#### F2: Callback exceeds runtime budget (Medium)

Blocks ble_event_thread; concurrent BLE work latency. `FURI_DEBUG` builds: `furi_check(elapsed < 50ms)` triggers panic.

#### F3: Stack overflow in ble_event_thread (Critical)

Callback uses too much stack (>~256 B). Memory corruption far from call site. **Mitigation:** documented stack budget.

#### F4: Cross-handler flow control coupling (HIGH)

`BleEventAckFlowDisable` stops ALL HCI events globally. Other concurrent BLE consumers (BLE Serial used by qFlipper, custom GATT services) stall when our queue fills. **Mitigation:** backpressure timeout at 5000 ms force-stops session; recommendation to size queue ≥ 32. Avoid using queued scan alongside BLE Serial concurrently.

#### F5: furi_hal_bt_reinit() mid-scan (Resolved)

`force_stop_for_reinit` hook installed in `furi_hal_bt_reinit()`. State cleaned up before controller reset.

#### F6: Heap allocation failure (Process-fatal)

`malloc()` panics via Flipper's heap hook. Matches codebase norms. No API-level mitigation.

### 4.4. Limitations (accepted v1 compromises)

- Scan parameters static at start; no mid-session adaptation to connection state changes.
- No native RPA resolution.
- No extended advertising / Coded PHY (BLE_Stack_light limit).
- No raw promiscuous packet capture.
- Single global scan session (second `start_scan` returns `Busy`).
- ±6 dB RSSI absolute accuracy.
- No ad-payload deduplication beyond controller's MAC-level filter.
- Timestamp source: `furi_get_tick()` at dispatch (not controller-provided); jitter envelope ~10–50 µs typical, up to dispatch_mutex hold time under contention.
- Allowlist size cap: 255 entries (uint8_t length field).
- Behavior during USB-attached/charging state: unaffected; radio is independent.

### 4.5. Recovery procedures (user-facing)

#### R1: Device hard-faults (F1)

Wait for automatic reboot. Boot is normal; BLE reinitializes. Identify and avoid the offending FAP.

#### R2: BLE seems broken after a scan session

1. Settings → Bluetooth → toggle off and on.
2. Reboot Flipper if still broken.
3. Re-flash via qFlipper if still broken (preserves user data).

#### R3: Custom firmware build appears bricked

1. Open qFlipper. Try "Install official firmware."
2. **If qFlipper cannot reach the device** (boot hangs before USB initializes): hold **OK** during boot to enter the ST ROM bootloader (Hardware DFU mode). qFlipper can flash via this mode.
3. *Caveat:* if option bytes are damaged, even ST DFU may not work — recovery requires an ST-Link debugger.

#### R4: Radio stack reported missing/corrupt

Reinstall via qFlipper, which invokes FUS through SHCI to manage radio stack uninstall/install. FUS itself lives in protected M0+ flash and survives M4 corruption. (If FUS is truly gone, an ST-Link is required — not recoverable in-field.)

### 4.6. Backwards compatibility commitment

- API version 87.7 → **87.8** (additive minor bump per Unleashed's CSV scheme; verified).
- Promise: **no field removal AND no field reordering** for `FuriHalBtScanReport`, `FuriHalBtScanConfig`, `FuriHalBtScanStats`. Field additions at end-of-struct safe via `struct_size` field.
- Error codes stable once shipped; don't renumber.
- New error codes additive (FAPs treat unknown errors as generic failures).

---

## 5. Testing strategy

### 5.1. Host-portable unit tests

Located at `applications/debug/unit_tests/tests/furi_hal_bt_scan/`, using Unleashed's existing `minunit.h` framework. Compile both on-device (via existing unit_tests FAP) and host-portable (via `#ifdef`-guarded code).

**Targets:**
1. `validate_config()` — config struct sanity (window > interval, allowlist consistency, struct_size mismatch, NULL config, all-defaults config).
2. `allowlist_permits(mac, list, len)` — empty allowlist accepts all; single-entry exact match; multi-entry; non-matching MAC rejected.
3. `should_force_stop_backpressure(now_ms, since_ms, timeout_ms)` — table-driven; not flow-disabled returns false; elapsed < timeout returns false; elapsed >= timeout returns true; uint32_t millisecond wraparound handled correctly.

Drop malloc-failure tests: Flipper's heap hook panics rather than returning NULL (cited contract).

### 5.2. On-device test FAP — `applications/debug/ble_scan_test/`

Menu-driven test harness, results persisted to `/ext/apps_data/ble_scan_test/results.txt`. Tests T1–T19:

| ID | Test | Procedure | Expected |
|---|---|---|---|
| T1 | Happy path callback | Default config + counter callback; 10 s; stop | Counter > 0; clean stop |
| T2 | Happy path queued | Default config + depth-32 queue; 10 s; stop | events_received ≈ events_delivered |
| T3 | Backpressure manual recovery | Depth-4 queue; don't drain 5 s; observe drops; drain; resume_flow | events_dropped > 0; events resume after recovery |
| T4 | Backpressure auto-timeout | T3 but never recover | ScanEnded fires with BackpressureTimeout after ~5 s |
| T5 | Duration auto-stop | duration_ms=3000 | ScanEnded fires with DurationExpired at correct time |
| T6 | stop_scan idempotency | Call stop_scan twice | Both return None; no crash |
| T7 | Reentrancy detection | Inside callback, call start/stop_scan | Both return Reentrant |
| T8 | Busy detection | Start scan; second start from another thread | Returns Busy |
| T9 | NotReady detection | Stop radio stack; start_scan | Returns NotReady |
| T10 | Allowlist filtering | Single-MAC allowlist (known nearby phone) | Only matching reports; events_filtered increments |
| T11 | Active scan auto-pauses adv | Verify Flipper advertising; type=Active; verify pause; stop; verify restore | Phone observes pause/restore |
| T12 | Passive scan does NOT pause adv | Same as T11 with type=Passive | Advertising visible throughout |
| T13 | Connection-aware downshift | Connect phone first; start scan with default 30/100 | Connection stable; HAL applied 20/160 |
| T14 | force_stop_for_reinit | Start scan; from other thread call furi_hal_bt_reinit() | ScanEnded(ForceStoppedForReinit); no hardfault |
| T15 | Heap leak | Start/stop scan ×100 | Heap delta near zero |
| T16 | Stats accuracy | 60 s in busy environment; cross-reference phone scanner | events_received within ~10% |
| **T17** | **Race stress** | 60 s loop: start/stop on thread A while 1 kHz timer posts mock reports | No use-after-free, no mutex assertion failures |
| **T18** | **Concurrent BLE consumer** | Run scan with BLE Serial (qFlipper open) | No deadlock; GATT stays responsive |
| **T19** | **Controller edge values** | Iterate interval = {0x0004, 0x4000}; window <= interval; window > interval (rejection) | Document silent-clamping behavior; rejection path returns ControllerReject |

T17 is critical for catching race bugs that two-mutex designs ship with. T18 is the single biggest upstream-rejection risk to mitigate before submission.

### 5.3. Regression checklist (manual on `carbon`)

**Realistic time: 15–20 minutes first pass, ~8 once familiar.**

1. **qFlipper desktop pairing** — connect, browse SD, disconnect.
2. **Mobile app pairing** — pair Flipper Mobile, verify GATT services visible.
3. **BLE HID profiles** (if used) — Flipper-as-keyboard.
4. **Extra beacon** — enable AirTag emulation, verify another Apple device finds it.
5. **Bluetooth on/off toggle** — recovers cleanly.
6. **Reboot recovery** — BLE comes up cleanly with no scan state lingering.
7. **`bt_debug_app` carrier/packet test** — exercises radio path our patch touches.
8. **One known third-party BLE FAP** (BLE Spam) — hammers `extra_beacon`; verify no regression.

Run before tagging any release of the patched fork.

### 5.4. Build-time checks

`./fbt firmware_all` reports M4 flash via `scripts/fwsize.py` (one-shot reporter; not a regression script — we write our own ~40 LOC baseline-diff for fork CI).

- **Pre-patch baseline**: build current Unleashed `dev` or release tag; record `__free_flash_start__`-derived headroom.
- **Post-patch**: build patched fork; record same.
- **Acceptance**: post-patch headroom delta ≤ estimate (1.5–4 KB) + 25%. Hard fail at +25%; warn at +10%.
- LTO: do **not** propose for upstream PR; stay COMPACT=0 to match Unleashed default. Fork-internal optimization only.

### 5.5. CI integration

**Unleashed currently has only `.github/workflows/codeql.yml` (no firmware-build CI).** Two options:

- **Optional separate commit alongside upstream PR**: minimal `build-and-test.yml` workflow running `./fbt firmware_all` and host-gcc unit tests. Frame as "your repo doesn't currently have build CI; here's a minimal one." Reviewers may decline.
- **Fork-only**: `timFinn/unleashed-firmware` uses Forgejo Actions internally; not part of upstream PR.

Test FAP build verification (`ufbt` build of the test FAP) is cheap and catches API drift. Add as CI job step.

### 5.6. Spec appendices

This document IS the spec. Appendices follow:

- **Appendix A**: Unit test cases (5.1)
- **Appendix B**: On-device test procedures (5.2)
- **Appendix C**: Regression checklist (5.3)
- **Appendix D**: Recovery procedures (4.5)

### 5.7. Test FAP framing for upstream PR

In PR description: "Test FAP `ble_scan_test` ships in `applications/debug/`, follows `bt_debug_app` precedent (existing debug-only FAP shipped in firmware), removable via single .fam edit if not desired. Strict test harness — not a user-facing scanner. End-user scanner is a separate FAP project at `applications_user/ble_scanner` (downstream)."

### 5.8. Comment scrubbing

Test FAP commit (#8) is highest risk for "this looks AI-generated." Keep comments terse, engineer-voiced. No multi-paragraph docstrings, no disclaimers. Run `./fbt format` before each commit. Verify against Unleashed's `CODING_STYLE.md` (4-space tabs, brace-on-same-line, snake_case functions, PascalCase types).

---

## Appendix A: Unit test cases (host-portable)

Implemented in `applications/debug/unit_tests/tests/furi_hal_bt_scan/`:

```c
/* test_validate_config.c */
MU_TEST(test_validate_null_config) {
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(NULL));
}
MU_TEST(test_validate_window_gt_interval) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    cfg.window_ms = 200; cfg.interval_ms = 100;
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(&cfg));
}
MU_TEST(test_validate_struct_size_mismatch) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    cfg.struct_size = 1;
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(&cfg));
}
MU_TEST(test_validate_allowlist_len_with_null_list) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    cfg.allowlist = NULL; cfg.allowlist_len = 5;
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(&cfg));
}
MU_TEST(test_validate_default_config_ok) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    mu_assert_int_eq(FuriHalBtScanErrorNone,
                     validate_config(&cfg));
}

/* test_allowlist_permits.c */
MU_TEST(test_allowlist_empty_permits_all) {
    uint8_t mac[6] = {0xDE,0xAD,0xBE,0xEF,0x12,0x34};
    mu_check(allowlist_permits(mac, NULL, 0));
}
MU_TEST(test_allowlist_single_match) {
    uint8_t list[1][6] = {{0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}};
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    mu_check(allowlist_permits(mac, list, 1));
}
MU_TEST(test_allowlist_single_reject) {
    uint8_t list[1][6] = {{0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}};
    uint8_t mac[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    mu_check(!allowlist_permits(mac, list, 1));
}

/* test_backpressure_timeout.c */
MU_TEST(test_backpressure_not_disabled) {
    /* flow_disabled_since_ms=0 means not flow-disabled */
    mu_check(!should_force_stop_backpressure(10000, 0, 5000));
}
MU_TEST(test_backpressure_under_timeout) {
    mu_check(!should_force_stop_backpressure(8000, 5000, 5000));
}
MU_TEST(test_backpressure_at_timeout) {
    mu_check(should_force_stop_backpressure(10000, 5000, 5000));
}
MU_TEST(test_backpressure_wraparound) {
    /* now wrapped past UINT32_MAX, since_ms close to UINT32_MAX */
    mu_check(should_force_stop_backpressure(1000, 0xFFFFE000, 5000));
}
```

## Appendix B: On-device test FAP layout

```
applications/debug/ble_scan_test/
├── application.fam                    # appid=ble_scan_test, FAP, fap_category="Debug"
├── ble_scan_test.c                    # entry point, scene navigation
├── tests/
│   ├── t01_happy_callback.c
│   ├── t02_happy_queued.c
│   ├── t03_backpressure_manual.c
│   ├── t04_backpressure_auto.c
│   ├── t05_duration_autostop.c
│   ├── t06_stop_idempotent.c
│   ├── t07_reentrancy.c
│   ├── t08_busy.c
│   ├── t09_not_ready.c
│   ├── t10_allowlist.c
│   ├── t11_active_pauses_adv.c
│   ├── t12_passive_no_pause.c
│   ├── t13_connection_downshift.c
│   ├── t14_force_stop_reinit.c
│   ├── t15_heap_leak.c
│   ├── t16_stats_accuracy.c
│   ├── t17_race_stress.c
│   ├── t18_concurrent_ble_consumer.c
│   └── t19_controller_edge_values.c
├── views/
│   ├── menu_view.c                    # test selection
│   └── results_view.c                 # pass/fail display, log scroll
└── README.md                          # framing for upstream PR
```

Tests T1-T9, T15-T19 are fully automated. T10, T11-T13, T16 require human verification of external state (visible advertising, paired phone, etc.); FAP displays "verify and press OK" prompts.

## Appendix C: Regression checklist

See section 5.3.

## Appendix D: Recovery procedures

See section 4.5.

---

## Open items for plan-writing phase

These are implementation details that will surface during the writing-plans phase, not design questions:

- **Exact `gap.c` line numbers** for inserting the GAP_OBSERVER_ROLE addition (verified `gap.c:361` per rf-engineer consult).
- **Exact placement** of `furi_hal_bt_scan_force_stop_for_reinit()` call in `furi_hal_bt.c::furi_hal_bt_reinit()` (verified `furi_hal_bt.c:202`).
- **CFG_TLBLE_EVT_QUEUE_LENGTH** bump from 5 to 8 in `app_conf.h` for observer event headroom (rf-engineer recommendation; verify in plan phase whether actually needed via T17 stress test).
- **`./fbt format`** output review for the entire patch before each commit.

---

## Footnotes / consult attribution

- API shape, guarding, data flow, errors+limitations, testing: 5 separate embedded-engineer consults (separate sessions per FURI/superpowers convention).
- Scan parameter defaults, RPA semantics, coexistence: 1 rf-engineer consult.
- Flash budget verification (refuted WillyJL's "20 KB" claim as conflating M4 vs M0+ budgets; observer-only mode confirmed feasible at ~1.5-4 KB M4): 1 embedded-engineer consult on Momentum source.
- Source verifications: Momentum HAL surface (no scan API), Unleashed HAL surface (parity), Unleashed CI platform (GitHub Actions, not Forgejo), `SVCCTL_ResumeUserEventFlow` symbol (exists at `ble_app.c:167`), FAP loader unload hook (does not exist in `flipper_application.h`).

End of design spec.
