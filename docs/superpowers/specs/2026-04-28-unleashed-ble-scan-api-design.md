# Unleashed BLE Observer-Mode Scan API — Design Spec

**Date:** 2026-04-28 (revised 2026-04-29)
**Author:** timFinn
**Status:** Draft, ready for implementation
**Target:** [DarkFlippers/unleashed-firmware](https://github.com/DarkFlippers/unleashed-firmware) (private fork at `timFinn/unleashed-firmware`, designed for upstream PR consideration)
**Consults integrated:** embedded-engineer ×6 (API shape, guarding, data flow, errors+limitations, testing, full-spec audit), rf-engineer ×2 (scan parameter defaults, full-spec audit), architect ×1 (full-spec audit), code-reviewer ×1 (full-spec audit), source-verification ×3 (Momentum infrastructure, Unleashed CI/loader, Unleashed test framework)

---

## Summary

Add a BLE observer-mode scanning HAL to Unleashed firmware. Exposes `furi_hal_bt_start_scan` (callback variant) and `furi_hal_bt_start_scan_queued` (queue variant) plus supporting functions. Surgical patch (~2-4 KB M4 code, no copro changes), 11 commits in a logical PR-ready series, with a test FAP and on-device unit tests bundled.

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
targets/f7/ble_glue/gap.c                       MODIFIED — add GAP_OBSERVER_ROLE; add private accessors and pause/resume helpers
targets/f7/ble_glue/app_conf.h                  MODIFIED — bump CFG_TLBLE_EVT_QUEUE_LENGTH 5 → 8
targets/f7/furi_hal/furi_hal_bt.c               MODIFIED — call furi_hal_bt_scan_force_stop at top of furi_hal_bt_reinit
targets/f7/api_symbols.csv                      MODIFIED — version 87.7 → 87.8; add new entries (sorted-position insertion)
applications/debug/unit_tests/application.fam   MODIFIED — add new App() block for test_furi_hal_bt_scan PLUGIN
```

Total touched files: **11** (5 new, 6 modified). The `api_symbols.csv` touch is required by Unleashed's FAP ABI — every public symbol must be listed or FAPs can't link against it at load time.

**Note:** `gap.h` is **not** modified. The accessors (`gap_has_active_connection`, `gap_get_connection_interval_ms`) are private to the patch — declared in `gap_internal.h`, not exported via `api_symbols.csv`. They exist for the scan HAL's internal use only; FAPs that need connection state must use existing public APIs like `furi_hal_bt_is_active()`.

**Note:** `CFG_TLBLE_EVT_QUEUE_LENGTH` increase from 5 to 8 is required for observer-mode event headroom (rf-engineer recommendation; verified during T17 stress test). This costs ~1.5 KB of SRAM2A but prevents IPCC starvation under busy RF environments.

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
stop_scan and furi_hal_bt_scan_force_stop are the only functions that acquire both.
```

Any new code touching scan state must obey this ordering.

`furi_hal_bt_scan_force_stop()` (used by both the FAP-thread-liveness mitigation and the `furi_hal_bt_reinit()` integration) acquires `control_mutex` internally, so it must not be called from contexts that already hold it.

**Phase access discipline:** The phase variable is always accessed via `__atomic_load_n(..., __ATOMIC_ACQUIRE)` for reads and `__atomic_store_n(..., __ATOMIC_RELEASE)` for writes — even when reading inside `dispatch_mutex`. Mutexes order data fields they protect; they do not order phase reads issued from outside the writer's critical section.

### 1.6. Commit plan (11 commits, PR-ready)

```
 1. gap: add GAP_OBSERVER_ROLE to aci_gap_init role mask
 2. gap: add internal connection-state accessors (gap_internal.h, not exported)
 3. gap: add internal advertising pause/resume helpers
 4. ble_glue: bump CFG_TLBLE_EVT_QUEUE_LENGTH for observer headroom
 5. furi_hal_bt: add observer scan HAL skeleton (start_scan returns NotReady)
 6. furi_hal_bt: wire dispatch path with thread-liveness check (callback variant)
 7. furi_hal_bt: add queued variant with flow-control backpressure
 8. furi_hal_bt: add ScanEnded callback + duration timer + backpressure timeout
 9. furi_hal_bt: expose scan stats; update api_symbols.csv (sorted insertion)
10. furi_hal_bt: install force_stop hook in furi_hal_bt_reinit
11. applications/debug: add ble_scan_test FAP + unit_tests App() block + tests
```

Each commit compiles standalone (`./fbt firmware_all` succeeds). Each commit run through `./fbt format` per Unleashed's `CODING_STYLE.md`. Bisect-friendly.

**Restructuring rationale (per architect audit):** the original 9-commit plan had commit 4 leaving dead code (state machine with no dispatch wiring) and commit 7 overloaded with three orthogonal concerns. The 11-commit plan resolves these: commit 5 ships a working-but-disabled HAL (returns NotReady); commits 6–8 incrementally add capability with each commit fully functional; commits 9–10 split stats/CSV from the reinit hook (different files, different review concerns).

**Optional separate PR commits (good-faith offerings, not part of main series):**

- GitHub Actions workflow for build + tests (Unleashed has no firmware-build CI today)
- One-line `flipper_application.c` FAP-teardown hook (additional F1 mitigation; the in-dispatcher thread-liveness check in commit 6 is the always-on defense, this would be a cleaner upstream-accepted alternative if the loader maintainers agree)

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
    FuriHalBtScanErrorControllerReject,    /**< HCI command returned non-zero status; query furi_hal_bt_scan_last_hci_status() */
    FuriHalBtScanErrorReentrant,           /**< Caller is the dispatch thread */
} FuriHalBtScanError;

/* ── Scan configuration ──────────────────────────────────────────── */

typedef enum {
    FuriHalBtScanTypePassive = 0,  /**< RX-only; no SCAN_REQ; lower power */
    FuriHalBtScanTypeActive = 1,   /**< Sends SCAN_REQ; captures scan-response; forces advertising pause.
                                        WARNING: pairs/bonded peers (qFlipper, mobile companion app) will see
                                        the Flipper disappear from the BLE peripheral list during the scan
                                        and may auto-disconnect (typical supervision timeouts 4–6s; iOS often
                                        drops faster). Use Passive unless you specifically need scan-response
                                        data (e.g., full local name, additional service UUIDs). */
} FuriHalBtScanType;

typedef enum {
    FuriHalBtScanEndedReasonStopped,                /**< User called stop_scan */
    FuriHalBtScanEndedReasonDurationExpired,        /**< duration_ms timer fired */
    FuriHalBtScanEndedReasonForceStoppedForReinit,  /**< furi_hal_bt_reinit during scan */
    FuriHalBtScanEndedReasonControllerFault,        /**< Controller error mid-session */
    FuriHalBtScanEndedReasonBackpressureTimeout,    /**< Queue stayed full > FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS */
} FuriHalBtScanEndedReason;

typedef struct {
    uint32_t events_received;        /**< Total HCI reports arrived at HAL */
    uint32_t events_filtered;        /**< Dropped by allowlist */
    uint32_t events_delivered;       /**< Passed to callback or queue */
    uint32_t events_dropped;         /**< Queue full (triggers backpressure) */
    uint32_t events_dropped_phase;   /**< Dropped due to state (starting/stopping) */
} FuriHalBtScanStats;

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
 * struct_size MUST be set to sizeof(FuriHalBtScanConfig) at the version the
 * caller was compiled against. The HAL validates struct_size against the
 * HAL's known min and max sizes, supporting forward compatibility:
 *
 *   - struct_size < FURI_HAL_BT_SCAN_CONFIG_MIN_SIZE  → InvalidParameter
 *   - struct_size > sizeof(current)                  → InvalidParameter
 *                                                       (older HAL doesn't know newer fields)
 *   - else: HAL reads up to caller's struct_size; missing tail fields
 *           treated as zero/default
 *
 * Field ordering is alignment-natural (uintptr/ptr → u32 → u16 → u8 → bool)
 * to ensure stable sizeof across compiler versions; the struct_size scheme
 * depends on this.
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
 *
 * window_ms must satisfy window_ms <= 0.95 * interval_ms (ETSI EN 300 328
 * adaptive frequency hopping duty-cycle compliance for active scans;
 * enforced for both types for consistency).
 */
typedef struct {
    /* Pointers first (8-byte aligned on M4 ABI; 4-byte on AAPCS) */
    const uint8_t (*allowlist)[6];   /**< NULL = accept all */
    FuriHalBtScanEndedCallback ended_callback;  /**< NULL = no terminal notification */
    void*    ended_context;
    /* uint32_t fields */
    uint32_t duration_ms;            /**< Auto-stop after this many ms; 0 = until stop_scan */
    /* uint16_t fields */
    uint16_t struct_size;            /**< = sizeof(FuriHalBtScanConfig); first non-pointer field */
    uint16_t window_ms;              /**< Controller listen time per interval */
    uint16_t interval_ms;            /**< Period; must be >= window_ms; window_ms must be <= 0.95 * interval_ms */
    /* uint8_t / enum / bool fields (no padding needed at end) */
    FuriHalBtScanType type;          /**< 1 byte enum on AAPCS */
    uint8_t  allowlist_len;
    bool     filter_duplicates;
    bool     rssi_only;              /**< If true, adv_data unpopulated (saves CPU) */
} FuriHalBtScanConfig;

/**
 * Minimum recognized struct_size for forward-compat validation. Bumped
 * only when fields are added BEFORE the v1 set (which doesn't happen in
 * practice — additions go at the end). Effectively: lower bound for
 * "is this a sane v1 or later config struct."
 */
#define FURI_HAL_BT_SCAN_CONFIG_MIN_SIZE sizeof(FuriHalBtScanConfig)

/**
 * Default config, suitable for general-purpose scanning.
 *
 * Usage: FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
 * Do NOT take the address of this macro directly (compound literal has
 * automatic storage and goes out of scope at the end of the statement).
 */
#define FURI_HAL_BT_SCAN_CONFIG_DEFAULT            \
    ((FuriHalBtScanConfig){                        \
        .allowlist = NULL,                         \
        .ended_callback = NULL,                    \
        .ended_context = NULL,                     \
        .duration_ms = 0,                          \
        .struct_size = sizeof(FuriHalBtScanConfig),\
        .window_ms = 30,                           \
        .interval_ms = 100,                        \
        .type = FuriHalBtScanTypePassive,          \
        .allowlist_len = 0,                        \
        .filter_duplicates = false,                \
        .rssi_only = false,                        \
    })

/* ── Scan reports ────────────────────────────────────────────────── */

#define FURI_HAL_BT_SCAN_ADV_MAX_LEN 31

/**
 * BLE address type values (mirrors BT Core spec Vol 6, Part B).
 *
 * Note: addr_type alone does NOT indicate whether the address rotates.
 *   - Public + Random Static → stable
 *   - Random Resolvable Private (RPA) → rotates ~15 min; identify via
 *     mac[5] >> 6 == 0b10
 *   - Random Non-Resolvable → typically stable per session
 */
typedef enum {
    FuriHalBtAddrTypePublic = 0,
    FuriHalBtAddrTypeRandom = 1,
    FuriHalBtAddrTypeRandomNonResolvable = 2,  /**< BT spec subtype; reported as 1 by ST stack */
    FuriHalBtAddrTypeRandomStatic = 3,         /**< BT spec subtype; reported as 1 by ST stack */
} FuriHalBtAddrType;

/**
 * One advertising report delivered to consumers.
 *
 * Pure value type: no internal pointers. Safe to memcpy, store, or
 * send through message queues.
 *
 * RSSI accuracy: ±6 dB absolute (uncalibrated against trace antenna);
 * ±2 dB relative for same-advertiser packets within a few seconds.
 * Average ≥8 reports for a stable reading.
 *
 * For Resolvable Random Addresses (addr_type == 1 AND mac[5] >> 6 == 0b10),
 * mac is the RPA bytes verbatim. Cross-rotation correlation requires the
 * device's IRK (not exposed by this HAL).
 *
 * struct_size enables forward-compat: future fields appended at the
 * end will not break old FAP readers.
 */
typedef struct {
    /* Largest fields first to avoid implicit padding */
    uint8_t  adv_data[FURI_HAL_BT_SCAN_ADV_MAX_LEN];
    uint8_t  mac[6];
    uint16_t struct_size;            /**< = sizeof(FuriHalBtScanReport) */
    uint8_t  addr_type;              /**< FuriHalBtAddrType; ST stack only reports 0 or 1 */
    int8_t   rssi;                   /**< dBm; see accuracy notes above */
    uint8_t  adv_data_len;           /**< 0..31 */
    uint8_t  _reserved;              /**< natural alignment to 4-byte boundary */
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

/* FuriHalBtScanStats type defined above (above ended_callback typedef). */

/**
 * Read current scan statistics into caller-provided struct.
 *
 * Individual uint32_t fields are atomically read. Cross-field
 * consistency is not guaranteed for a single call (fields may be
 * sampled across an update boundary).
 *
 * Out-param signature (rather than return-by-value) preserves ABI
 * stability if the struct grows in future versions.
 *
 * @param out  Caller-allocated. Required (non-NULL).
 */
void furi_hal_bt_scan_stats(FuriHalBtScanStats* out);

/**
 * Reset all scan statistics to zero. Stats persist across stop_scan
 * and are NOT auto-reset on start_scan; call this explicitly if
 * desired. No-op when no scan is or was active.
 */
void furi_hal_bt_scan_stats_reset(void);

/**
 * After FuriHalBtScanErrorControllerReject, query the underlying ST/
 * Bluetooth-SIG status code that caused the rejection.
 *
 * Returns 0 if no controller rejection has occurred since the last
 * successful start_scan. Stable until the next start_scan attempt.
 *
 * Status codes are opaque (ST stack / BT spec layer); no stability
 * promise on their values across firmware updates.
 */
uint8_t furi_hal_bt_scan_last_hci_status(void);

/* ── Start / stop ────────────────────────────────────────────────── */

/**
 * Start a BLE scan, delivering reports via callback on BLE event thread.
 *
 * @param config       Scan parameters. Required (non-NULL).
 * @param callback     Report handler.
 * @param context      Opaque pointer passed to callback. HAL does not free;
 *                     caller must keep valid until stop_scan returns OR
 *                     until ended_callback fires (whichever comes first).
 * @return             FuriHalBtScanErrorNone on success.
 *                     On ControllerReject: query furi_hal_bt_scan_last_hci_status().
 */
FuriHalBtScanError furi_hal_bt_start_scan(
    const FuriHalBtScanConfig* config,
    FuriHalBtScanCallback callback,
    void* context);

/**
 * Start a BLE scan, delivering reports by-value into a message queue.
 *
 * Preferred for FAPs doing non-trivial work per report. Queue receives
 * FuriHalBtScanReport values copied into its internal storage — no
 * lifetime concerns.
 *
 * If queue fills, HAL returns BleEventAckFlowDisable to controller and
 * pauses dispatch. Reports resume after furi_hal_bt_scan_resume_flow().
 *
 * NOTE on cross-handler effects: BleEventAckFlowDisable stops ALL HCI
 * events globally, not just adv reports. While backpressured, BLE Serial
 * (used by qFlipper), GATT custom services in user FAPs, and other BLE
 * consumers will stall. Size your queue ≥ 32 to make backpressure rare,
 * and avoid the queued variant if running concurrently with BLE Serial.
 * The HAL force-stops the session if backpressure persists more than
 * FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS (default 5000).
 *
 * @param report_queue    Caller-owned. Element size must equal
 *                        sizeof(FuriHalBtScanReport).
 */
FuriHalBtScanError furi_hal_bt_start_scan_queued(
    const FuriHalBtScanConfig* config,
    FuriMessageQueue* report_queue);

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
 * Worst-case latency: bounded by the longest in-flight callback
 * runtime (target < 10 ms) plus HCI command-complete sync (~5–50 ms);
 * total typically < 100 ms.
 *
 * Idempotent: returns None if no scan is active.
 *
 * Returns Reentrant if called from within a scan callback OR ended_callback.
 * Both run on the dispatch thread; you cannot stop the scan from inside
 * its own dispatch path. Restructure to post a flag/message and call
 * stop_scan from the FAP's own thread.
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
 * Force-stop any active scan, regardless of which thread owns it.
 *
 * Used internally by furi_hal_bt_reinit() before resetting the
 * controller. Also exposed publicly so a future flipper_application.c
 * teardown hook (separate optional PR) can call it on FAP unload.
 *
 * Fires ended_callback with reason ForceStoppedForReinit if a scan
 * was active.
 *
 * Thread-safe: acquires control_mutex internally. MUST NOT be called
 * from contexts that already hold control_mutex.
 */
void furi_hal_bt_scan_force_stop(void);

#ifdef __cplusplus
}
#endif
```

### 2.2. Private extensions to gap — `targets/f7/ble_glue/gap_internal.h`

```c
/**
 * @file gap_internal.h
 * Private GAP helpers used only by furi_hal_bt_scan.c.
 *
 * NOT part of the public HAL. Not declared in any installed header.
 * Not exported via api_symbols.csv. FAPs cannot reach this header.
 *
 * Exposing these as public would leak GAP internal state to FAP authors
 * with no precedent in the codebase (gap.c's existing gap_get_state is
 * not in api_symbols.csv either). Scan HAL is the sole consumer.
 */

#pragma once

#include <gap.h>
#include <stdbool.h>
#include <stdint.h>

/* ── Connection-state read accessors ─────────────────────────────── */

/** Whether a GATT peer is currently connected. */
bool gap_has_active_connection(void);

/** Active connection interval in ms. Returns false if no connection. */
bool gap_get_connection_interval_ms(uint16_t* out_interval_ms);

/* ── Advertising pause/resume ────────────────────────────────────── */

typedef struct {
    bool     was_active;
    bool     was_connectable;
    uint16_t saved_min_adv_interval;
    uint16_t saved_max_adv_interval;
} GapAdvertisingSavedState;

void gap_pause_advertising(GapAdvertisingSavedState* out_saved);
void gap_resume_advertising(const GapAdvertisingSavedState* saved);
```

### 2.3. Internal scan module state — `furi_hal_bt_scan.c`

Field ordering groups locks-with-state-they-protect for review readability,
and uses alignment-natural ordering to minimize implicit padding.

```c
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
    /* Locks (declared near the state they protect) */
    FuriMutex*        control_mutex;     /* serializes start/stop/force_stop callers */
    FuriMutex*        dispatch_mutex;    /* held by event handler; stop_scan acquires as join barrier */

    /* Phase — accessed via __atomic_load_n / __atomic_store_n with release/acquire ordering.
     * NOT volatile (would imply different semantics than __atomic provides).
     * MUST be u32-aligned; static_assert at module scope verifies. */
    uint32_t          phase;             /* ScanPhase */

    /* Reentrancy + ownership tracking */
    FuriThreadId      owner_thread;      /* set by start_scan; read by force_stop_for_thread variants */
    FuriThreadId      dispatch_owner;    /* non-NULL during ANY callback (scan or ended); set/cleared in callback AND queued paths */

    /* Active session state (protected: written under control_mutex, read under dispatch_mutex) */
    ScanMode          mode;
    FuriHalBtScanCallback callback;
    void*             context;
    FuriMessageQueue* queue;
    FuriHalBtScanEndedCallback ended_callback;
    void*             ended_context;
    struct {
        uint8_t (*list)[6];               /* HAL-owned copy */
        uint8_t   len;
    } allowlist;
    GapAdvertisingSavedState saved_adv;
    bool              adv_was_paused;

    /* Backpressure state (protected by dispatch_mutex; checked by event handler each dispatch) */
    bool              flow_disabled;
    uint32_t          flow_disabled_since_ms;

    /* Auto-stop scheduling */
    FuriTimer*        duration_timer;    /* NULL if cfg.duration_ms == 0 */

    /* End-reason resolution: first writer wins. Protected by dispatch_mutex.
     * Resolves the duration-vs-backpressure-timeout-vs-reinit race deterministically. */
    FuriHalBtScanEndedReason pending_end_reason;
    bool              pending_end_set;

    /* Last HCI status from a controller-reject path; queryable via furi_hal_bt_scan_last_hci_status().
     * Atomic u8 store/load; reset on each new start_scan attempt. */
    uint8_t           last_hci_status;

    /* Stats — touched by event handler via __atomic_fetch_add; read via stats() helper. */
    FuriHalBtScanStats stats;
} ScanModule;

static ScanModule g_scan;
```

### 2.4. api_symbols.csv additions

Inserted in sorted alphabetical position (Unleashed's CSV is sorted):

```
Header,+,targets/f7/ble_glue/furi_hal_bt_scan.h,,
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

`gap_has_active_connection` and `gap_get_connection_interval_ms` are NOT exported (private to the patch via `gap_internal.h`).

Plus version bump:
```diff
-Version,+,87.7,,
+Version,+,87.8,,
```

**Symbol versioning rollback policy:** if commit 9 (api_symbols.csv update) is reverted, the CSV reverts to 87.7 and the new symbols become unresolvable for FAPs that linked against 87.8. Consumers must redeploy. Reverts of any earlier commit (1–8) leave 87.8 in place but break compilation; a partial revert is not supported.

### 2.5. Compile-time configuration

```c
/* In furi_hal_bt_scan.c */

#define FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS  5000
```

If queue stays full longer than this, HAL force-stops the session and fires `ScanEnded(BackpressureTimeout)`. Prevents permanent-stall scenarios when a FAP hits a debugger breakpoint or otherwise stops draining.

```c
/* In targets/f7/ble_glue/app_conf.h — bumped from existing value 5 */
#define CFG_TLBLE_EVT_QUEUE_LENGTH  8
```

ST stack's IPCC event queue depth. The default 5 is too shallow for observer mode under busy RF (verified via T17 stress test). Costs ~1.5 KB SRAM2A. Required for the patch; not a tunable.

---

## 3. Data flow

### 3.1. Happy path — full scan session lifecycle (queued variant)

```
Thread       Phase        Step
──────────────────────────────────────────────────────────────────────────
FAP          Idle         1. FAP allocates FuriMessageQueue (depth 32, element = sizeof(FuriHalBtScanReport))
FAP          Idle         2. FAP calls furi_hal_bt_start_scan_queued(&cfg, q)
FAP          Idle         3. → Re-entry check: dispatch_owner != current_thread_id → OK
FAP          Idle         4. → Acquire control_mutex (FuriWaitForever)
FAP          Idle         5. → Phase atomic-load → Idle, OK to proceed
FAP          Idle         6. → Validate config: struct_size in [MIN_SIZE, sizeof(current)],
                              window <= 0.95*interval, allowlist consistent → OK
FAP          Idle         7. → Verify radio stack ready: furi_hal_bt_is_alive() → true
FAP          Idle         8. → Reset last_hci_status to 0
FAP          Idle         9. → Allocate allowlist copy (panic-on-fail per heap hook)
FAP          Idle        10. → Allocate duration_timer if cfg.duration_ms > 0 (panic-on-fail)
FAP          Idle        11. → Populate g_scan: mode=Queued, queue=q, ended_callback=cfg.ended_callback,
                              owner_thread=current_thread_id, pending_end_set=false, etc.
FAP          Idle        12. → Check gap_has_active_connection() → false. Use cfg as-is.
FAP          Idle        13. → cfg.type==Passive → no advertising pause needed
FAP          Idle        14. → Phase atomic-store → Starting (release barrier)
FAP          Starting    15. → Add GAP_OBSERVER_ROLE to current role mask if needed (one-time init)
FAP          Starting    16. → hci_le_set_scan_parameters(passive, window=30ms, interval=100ms, ...) sync
FAP          Starting    17. → Phase atomic-store → Active (release barrier)
                              CRITICAL: this MUST happen before the next step. Events arriving
                              between scan_enable and Active store would observe Starting and
                              be dropped to events_dropped_phase. Storing Active first means
                              any event received after the controller starts is dispatched.
FAP          Active      18. → hci_le_set_scan_enable(1, filter_dup=0) sync → command-complete OK
                              ON FAILURE: phase atomic-store → Idle, free duration_timer if alloc'd,
                              free allowlist, store last_hci_status, release control_mutex,
                              return ControllerReject. (Note: phase was already Active; the brief
                              window of "Active without controller running" causes only spurious
                              dropped_phase increments — harmless.)
FAP          Active      19. → If cfg.duration_ms != 0: start FuriTimer
FAP          Active      20. → Release control_mutex; return FuriHalBtScanErrorNone

(controller starts scanning channel 37, then 38, then 39, repeating)

ble_event   Active       21. ST stack receives ADV_IND; HCI dispatches HCI_LE_ADVERTISING_REPORT
ble_event   Active       22. → on_adv_report() invoked
ble_event   Active       23. → Try-acquire dispatch_mutex (timeout=0) → OK
ble_event   Active       24. → Phase atomic-load-acquire → Active (mutex doesn't order phase writes
                              from outside its critical section; always atomic-load phase)
ble_event   Active       25. → If owner_thread valid AND !furi_thread_is_alive(owner_thread):
                              schedule auto-stop (set BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP),
                              drop event, release dispatch_mutex, return BleEventAckFlowEnable.
                              This is the F1 mitigation — converts FAP-already-unloaded from
                              hardfault into clean teardown.
ble_event   Active       26. → stats.events_received atomic-inc
ble_event   Active       27. → Build FuriHalBtScanReport on stack from evt fields
ble_event   Active       28. → allowlist_permits(report.mac) → true (no allowlist set)
ble_event   Active       29. → mode==Queued: dispatch_owner = current_thread_id
ble_event   Active       30. → furi_message_queue_put(q, &report, 0) → OK
ble_event   Active       31. → dispatch_owner = NULL
ble_event   Active       32. → stats.events_delivered atomic-inc
ble_event   Active       33. → Release dispatch_mutex; return BleEventAckFlowEnable
ble_event   Active       34. → ST stack frees its event buffer

FAP         Active       35. (FAP's reader thread does furi_message_queue_get(q, &report, ...))
FAP         Active       36. (FAP processes: parse, classify, capture — all in FAP-thread context)

(steps 21-36 repeat for each adv packet)

FAP         Active       37. FAP decides to stop. Calls furi_hal_bt_stop_scan()
FAP         Active       38. → Re-entry check: dispatch_owner != current_thread_id → OK
FAP         Active       39. → Acquire control_mutex (FuriWaitForever)
FAP         Active       40. → Phase atomic-load-acquire → Active, OK to proceed
FAP         Active       41. → Phase atomic-store-release → Stopping
FAP         Stopping    42. → Stop duration_timer if set; free it
FAP         Stopping    43. → hci_le_set_scan_enable(0, 0) sync — controller drains its buffers
                              (synchronous via hci_send_req → hci_cmd_resp_wait; on return,
                              all events queued before this command have been processed by
                              ble_event_thread — that's our drain barrier)
FAP         Stopping    44. → Acquire dispatch_mutex (FuriWaitForever) — JOIN BARRIER
                              (no callback can be running; dispatch_mutex is the proof)
FAP         Stopping    45. → If adv_was_paused: gap_resume_advertising(&saved_adv)
FAP         Stopping    46. → Free allowlist copy; set list=NULL, len=0
FAP         Stopping    47. → Resolve end reason: if pending_end_set, use pending_end_reason;
                              else use Stopped (this is the explicit stop_scan caller path)
FAP         Stopping    48. → If ended_callback: dispatch_owner=current_thread_id;
                              invoke with resolved reason; dispatch_owner=NULL
FAP         Stopping    49. → NULL-out callback, context, queue, ended_callback (defensive)
FAP         Stopping    50. → Phase atomic-store-release → Idle
FAP         Idle        51. → Release dispatch_mutex
FAP         Idle        52. → Release control_mutex; return FuriHalBtScanErrorNone

FAP         Idle        53. (FAP can now safely free its queue, context, ended_context, allowlist memory)
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

FuriTimer thread has only ~1 KB stack — calling synchronous HCI directly would risk overflow and stalls the system Timer Service Task. Instead, the timer callback records the reason and signals via thread flag.

```
Thread          Phase        Step
──────────────────────────────────────────────────────────────────────────
duration_timer  Active       1. FuriTimer fires after cfg.duration_ms
                Active       2. Timer callback: under dispatch_mutex, if !pending_end_set:
                                  pending_end_reason = DurationExpired; pending_end_set = true;
                              furi_thread_flags_set(ble_event_thread,
                                BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP)

ble_event_thread Active     3. Wakes on flag; observes SCAN_AUTOSTOP bit
                Active       4. → Calls furi_hal_bt_stop_scan() — runs the full stop sequence
                                (steps 37-52 from 3.1)
                Stopping    5. → Step 47 resolves end reason from pending_end_reason → DurationExpired
                Stopping    6. → ended_callback fires with reason=DurationExpired
                Idle        7. Returns
```

**End-reason precedence (first writer wins).** If both duration timer and backpressure timeout fire in the same window, whichever sets `pending_end_set=true` first determines the reason reported to ended_callback. Both paths take dispatch_mutex briefly; the second writer observes pending_end_set==true and leaves it alone. This deterministically resolves the dual-auto-stop race.

If a user-initiated `stop_scan` runs concurrently with an auto-stop, the explicit caller wins reason-wise (step 47 of 3.1 uses `Stopped` only if pending_end_set is false; otherwise honors the pre-set reason). Practically speaking, this gives sensible behavior: if duration expired and the user *also* called stop_scan, the user sees their explicit stop.

### 3.4. Backpressure timeout — auto-terminate

If `flow_disabled` has been set for longer than `FURI_HAL_BT_SCAN_BACKPRESSURE_TIMEOUT_MS` (default 5000), the HAL force-stops the session.

```
Thread       Phase        Step
──────────────────────────────────────────────────────────────────────────
ble_event   Active       1. Each on_adv_report() invocation checks:
                              if (flow_disabled && (now - flow_disabled_since_ms) > TIMEOUT_MS)
                              (unsigned arithmetic naturally handles tick wraparound)
ble_event   Active       2. → Under dispatch_mutex (already held), if !pending_end_set:
                              pending_end_reason = BackpressureTimeout; pending_end_set = true
ble_event   Active       3. → Set BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP
ble_event   Active       4. → Continue dispatch normally for this packet

ble_event_thread Active   5. Wakes on flag; calls force-stop sequence
                Stopping  6. → Step 47 of 3.1 reads pending_end_reason → BackpressureTimeout
                Stopping  7. → ended_callback fires with reason=BackpressureTimeout
                Idle      8. → Returns
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

### 3.7. FAP exits without stop_scan — MITIGATED via thread-liveness check

**Without mitigation:** callback function pointer or context pointer becomes stale; next event handler invocation jumps to invalid memory → hard fault → device reboots.

**v1 mitigation (in-dispatcher liveness check, always-on):** the event handler checks `furi_thread_is_alive(owner_thread)` before invoking the callback or pushing to the queue (step 25 of 3.1). If the owner thread is gone, the HAL records pending_end_reason=ForceStoppedForReinit (semantically "owner gone"; reuse this enum value to avoid adding a new one for v1) and signals SCAN_AUTOSTOP.

```
ble_event   Active   1. on_adv_report() invoked
ble_event   Active   2. → Acquire dispatch_mutex; phase==Active
ble_event   Active   3. → Liveness check: owner_thread set AND
                          !furi_thread_is_alive(owner_thread)
ble_event   Active   4. → Under dispatch_mutex (already held), if !pending_end_set:
                          pending_end_reason = ForceStoppedForReinit; pending_end_set = true
ble_event   Active   5. → Set BLE_EVENT_THREAD_FLAG_SCAN_AUTOSTOP
ble_event   Active   6. → Drop event; release dispatch_mutex; return BleEventAckFlowEnable

ble_event   Active   7. (next loop iter) Wakes on flag; calls force-stop
            Stopping 8. → ended_callback fires (if registered) — but ended_context likely
                          dangles too at this point. Convention: HAL nulls the callback
                          fields BEFORE invoking ended_callback in the orphaned case
                          — i.e., orphaned cleanup skips ended_callback invocation
                          since both context and the FAP's code pages are gone.
            Idle     9. Returns. Stats remain queryable from any other thread.
```

Cost: one branch + one `furi_thread_is_alive` call per event in the hot path (~5 cycles on M4). Worth it.

**v1.x optional improvement:** the optional separate-PR commit adds a one-line `flipper_application.c` teardown hook that calls `furi_hal_bt_scan_force_stop()` directly when a FAP unloads. This is cleaner than the in-dispatcher check (catches the FAP exit synchronously) but requires upstream loader changes. The two are complementary: liveness check is the always-on defense; loader hook is the cleaner alternative if upstream maintainers accept it.

### 3.8. Connection establishes mid-scan

Static-at-start scan parameters. If a phone connects to the Flipper's GATT server mid-scan, running 30/100 params may stress the connection. Documented limitation. FAPs can observe connection state via the existing public `furi_hal_bt_is_active()` (true if connected OR advertising) and restart the scan with manually-downshifted params if needed. The HAL's internal downshift is one-time at start_scan; it does not adapt to mid-session changes.

### 3.9. Controller rejects scan setup

```
FAP        Starting   1. → hci_le_set_scan_parameters(...) sync → returns BLE_STATUS_INVALID_PARAMS
FAP        Starting   2. → Atomic-store last_hci_status = status_code (queryable post-return)
FAP        Starting   3. → Free duration_timer if allocated
FAP        Starting   4. → Free allowlist copy; reset list/len
FAP        Starting   5. → Phase atomic-store → Idle (release)
FAP        Idle       6. → Release control_mutex
FAP        Idle       7. → Return FuriHalBtScanErrorControllerReject
```

The FAP can call `furi_hal_bt_scan_last_hci_status()` after the return to read the underlying ST status code.

### 3.10. furi_hal_bt_reinit() called mid-scan

`furi_hal_bt_reinit()` calls `hci_reset()` and tears down the gap thread. Without our fix, scan state would point to a destroyed controller.

**Fix:** `furi_hal_bt_reinit()` calls `furi_hal_bt_scan_force_stop()` BEFORE `hci_reset()`.

```c
/* Patch to furi_hal_bt.c */
void furi_hal_bt_reinit(void) {
    furi_hal_bt_scan_force_stop();  /* NEW */
    /* ... existing code ... */
}
```

`furi_hal_bt_scan_force_stop()`:
- Acquires `control_mutex` internally — caller must NOT already hold it.
- Sets `pending_end_reason = ForceStoppedForReinit` if not already set.
- **Skips the HCI disable** (controller is about to be reset).
- Acquires `dispatch_mutex` as the join barrier.
- Resumes paused advertising, frees allowlist, fires `ended_callback`, transitions to Idle.
- No-op if no scan is active.

**Concurrency:** if `furi_hal_bt_reinit()` and `stop_scan()` are called concurrently from different threads, both acquire `control_mutex` first per 1.5 — they serialize cleanly. Whichever wins observes phase==Active, runs its teardown; the loser observes phase==Idle and is a no-op.

---

## 4. Error handling, failure modes, and limitations

### 4.1. Error code reference

| Error | Returned by | Cause | FAP recovery |
|---|---|---|---|
| `None` | All | Success | Proceed |
| `Busy` | `start_scan*` | Another scan session active | Wait, retry, or surface to user. Different from `Reentrant`: `Busy` means "try again later"; `Reentrant` means "structural bug, retry won't help" |
| `NotReady` | `start_scan*` | `furi_hal_bt_is_alive() == false` | Caller should ensure radio stack started; usually a startup-order bug |
| `InvalidParameter` | `start_scan*` | Validation failed: `struct_size` outside `[MIN_SIZE, sizeof(current)]`; `window_ms > 0.95 * interval_ms` (ETSI duty-cycle); `allowlist_len > 0` with NULL list; `queue == NULL` or queue element size != `sizeof(FuriHalBtScanReport)`; type out of enum range | Bug in caller config; not retryable. Common fix: use `FURI_HAL_BT_SCAN_CONFIG_DEFAULT` and modify only what you need. |
| `ControllerReject` | `start_scan*` | HCI command returned non-zero status | Query `furi_hal_bt_scan_last_hci_status()` for ST/Bluetooth-SIG code. Surface to user; retry with `FURI_HAL_BT_SCAN_CONFIG_DEFAULT`. |
| `Reentrant` | `start_scan*`, `stop_scan` | Caller is the dispatch thread (any callback context — scan callback OR ended_callback) | Restructure: post a message/flag from the callback to the FAP's own thread, and call the scan API from there. |

### 4.2. Non-error observable conditions

- **events_dropped > 0**: queue full, controller backpressured. FAP must call `furi_hal_bt_scan_resume_flow()` after draining.
- **events_filtered > 0**: allowlist working as intended.
- **events_dropped_phase > 0**: events arriving during state transitions; near-zero in steady state.
- **No advertisements observed**: not an error. Could be empty environment, allowlist over-restrictive, scan duty cycle insufficient.
- **Resolvable Random Address rotation**: documented BLE privacy feature; FAP cannot correlate across rotations without IRK.
- **RSSI ±6 dB absolute uncalibrated**: relative comparisons valid; distance estimates noisy.

### 4.3. Failure modes that bypass the API

#### F1: FAP exits without calling stop_scan — MITIGATED

Without mitigation: callback or context pointer stale → hard fault → reboot.

**v1 mitigation (always-on):** in-dispatcher `furi_thread_is_alive(owner_thread)` check before invoking callback or queue push. Detects orphaned scans on the next event and triggers a clean force-stop. Cost: ~5 cycles/event in the hot path.

**v1.x optional improvement:** loader-side teardown hook (separate optional PR). Catches orphaned scans synchronously at FAP unload rather than on the next event. Complementary to the in-dispatcher check.

Severity downgraded from Critical to Medium with the liveness-check mitigation in place.

#### F2: Callback exceeds runtime budget (Medium)

Blocks ble_event_thread; concurrent BLE work latency. `FURI_DEBUG` builds: `furi_check(elapsed < 50ms)` triggers panic.

#### F3: Stack overflow in ble_event_thread (Critical)

Callback uses too much stack (>~256 B). Memory corruption far from call site. **Mitigation:** documented stack budget. Add `configCHECK_FOR_STACK_OVERFLOW`-equivalent assertion if FreeRTOS config supports it; verify during T17.

#### F4: Cross-handler flow control coupling (HIGH)

`BleEventAckFlowDisable` stops ALL HCI events globally. Other concurrent BLE consumers (BLE Serial used by qFlipper, custom GATT services) stall when our queue fills.

**Mitigations:**
- Backpressure timeout (5000 ms default) force-stops the session and surfaces `BackpressureTimeout` to the FAP.
- Recommendation: size queue ≥ 32 to make backpressure rare.
- **Strong recommendation: avoid the queued variant if running concurrently with BLE Serial / qFlipper-active sessions.** No architectural fix at this layer; ST's stack design is what it is.

This is the single biggest upstream-rejection risk. T18 specifically validates the BLE Serial coexistence behavior.

#### F5: furi_hal_bt_reinit() mid-scan (Resolved)

`furi_hal_bt_scan_force_stop()` hook installed in `furi_hal_bt_reinit()`. State cleaned up before controller reset.

#### F6: Heap allocation failure (Process-fatal)

`malloc()` panics via Flipper's heap hook (`memmgr.c`). Matches codebase norms. No API-level mitigation. Spec: implementations should use `malloc()` (panic-on-fail) consistently rather than mixing `malloc` with handwritten `try_malloc` patterns; this matches every other allocation site in the firmware.

### 4.4. Limitations (accepted v1 compromises)

- **Scan parameters static at start.** No mid-session adaptation to connection-state changes. FAP can poll `furi_hal_bt_is_active()` and restart with downshifted params if it cares.
- **No native RPA resolution.** RPAs (`addr_type==1` AND `mac[5] >> 6 == 0b10`) pass through verbatim. Distinguish from non-resolvable random (`mac[5] >> 6 == 0b00`) and static random (`mac[5] >> 6 == 0b11`), which don't rotate. Cross-rotation correlation requires the device's IRK (out of scope).
- **No extended advertising / Coded PHY.** BLE_Stack_light supports legacy 1M-PHY advertising only. Switching coprocessor binaries to support BT 5.x extended modes is out of scope (M4 flash budget).
- **PHY: 1M only.** 2M-PHY advertising (rare, BT 5.0 peripherals) won't be received.
- **TX power for active scan: controller default (~0 dBm).** Not user-tunable via this HAL. Use `aci_hal_set_tx_power_level` directly if needed (advanced; affects all transmit operations).
- **No raw promiscuous packet capture.** Observer mode delivers parsed advertising reports only.
- **Single global scan session.** Second `start_scan` from any FAP returns `Busy`. Concurrent-FAP semantics: B observes phase==Active and gets `Busy`; A's session is unaffected.
- **RSSI accuracy: ±6 dB absolute, ±2 dB relative same-advertiser within seconds.** Average ≥8 reports for stable readings. RSSI is per-packet, not averaged.
- **No ad-payload deduplication.** Controller's `filter_duplicates` is MAC-only; rotating-RPA devices re-report regardless. FAP-side classification can dedup at the payload level.
- **Timestamp source: `furi_get_tick()` at dispatch.** Not controller-provided. Jitter envelope ~10–50 µs typical, up to dispatch_mutex hold time under contention.
- **Allowlist size cap: 255 entries** (`uint8_t` length field). Practical RAM-budget limit is far below this.
- **Behavior during USB-attached/charging:** unaffected. WB55 radio is independent of USB peripheral.
- **Power impact:** passive scan at 30/100 default has measurable but small battery cost; not quantified in this spec. Measurement during T15 is recommended; reviewers may request data.

### 4.5. Tuning guide (for FAP authors)

| Scenario | window_ms | interval_ms | min duration_ms | rationale |
|---|---|---|---|---|
| **General-purpose** (default) | 30 | 100 | 0 (forever) | Catches typical phones/wearables (100–1280 ms ADV_IND); 30% RX duty |
| **Beacon hunt** (iBeacon, AirTag, Tile) | 100 | 1280 | 4000 | Beacon advertisers use 1.28–2 s intervals; long window matches their cadence; lower CPU |
| **Fast-moving target** (drone, vehicle, mesh) | 80 | 100 | 30000 | High duty for sub-100 ms advertisers; expect higher power draw |
| **Dense WiFi 2.4 GHz environment** | 20 | 80 | 0 | Short window minimizes collision-window exposure; same effective duty |
| **Connected to qFlipper while scanning** | (auto-downshift to 20/160) | (auto-downshift) | 0 | HAL applies downshift automatically when GATT connection is active |

**Channel hopping:** ST controller hops 37/38/39 automatically. With default 30/100, each channel is reached every ~300 ms. **Scan duration should be at least 3× the slowest expected advertiser interval** to guarantee a hit on each channel.

**Active scan caveat:** captures scan-response data (full local name, additional service UUIDs) at the cost of TX'ing SCAN_REQ packets and pausing the Flipper's own advertising. Use Passive unless you specifically need scan-response data.

### 4.6. Recovery procedures (user-facing)

#### R1: Device hard-faults (rare with v1 mitigation)

With the in-dispatcher liveness check, F1 is mitigated. If a hard fault occurs anyway (other class of bug — F3 stack overflow, etc.):

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

### 4.7. Backwards compatibility commitment

- API version 87.7 → **87.8** (additive minor bump per Unleashed's CSV scheme; verified in `scripts/fbt/sdk/cache.py`).
- Promise: **no field removal AND no field reordering** for `FuriHalBtScanReport`, `FuriHalBtScanConfig`. Field additions at end-of-struct are safe via the `struct_size` validation scheme (`size >= MIN_SIZE && size <= sizeof(current)`; missing tail fields treated as zero-init).
- `FuriHalBtScanStats` uses out-param signature precisely so it can grow without ABI breakage; new fields appended at the end. Callers' `sizeof()` may be smaller; HAL writes only `min(caller_size, sizeof(current))` bytes — but to keep the API simple, we promise to write the full current struct and require callers to pass storage of at least `sizeof(FuriHalBtScanStats)`. If we add fields, we bump API minor; old FAPs continue to allocate the smaller storage and silently overflow into adjacent stack — so this is *not* fully forward-compatible without a `struct_size` mirror. **Future-proofing path:** if Stats ever grows, add `furi_hal_bt_scan_stats_v2()` rather than mutating in place.
- Error codes stable once shipped; don't renumber.
- New error codes additive (FAPs treat unknown errors as generic failures).
- `FuriHalBtAddrType` enum values stable; new values may be added (3, 4, ...) without breaking callers that switch on the existing values.

---

## 5. Testing strategy

### 5.1. On-device unit tests (PLUGIN, in unit_tests app)

Unleashed's existing on-device unit tests are PLUGINs registered in `applications/debug/unit_tests/application.fam`. We add a new App() block matching the `test_furi_hal` / `test_bt` precedent (`applications/debug/unit_tests/application.fam` lines ~41-46) and the test sources at `applications/debug/unit_tests/tests/furi_hal_bt_scan/`.

```python
# Added to applications/debug/unit_tests/application.fam:
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

**Targets** (extracted as non-static helpers in `furi_hal_bt_scan_internal.h` so the test plugin can link them):
1. `validate_config()` — null config, struct_size below MIN_SIZE, struct_size above sizeof(current), window > interval, window > 0.95*interval (ETSI), allowlist_len > 0 with NULL list, type out of range, all-defaults config.
2. `allowlist_permits(mac, list, len)` — empty allowlist accepts all; single-entry match/reject; **multi-entry** (3-entry list, verify second matches and third rejects); MAC-byte ordering correctness.
3. `should_force_stop_backpressure(flow_disabled, now_ms, since_ms, timeout_ms)` — uses explicit boolean rather than 0-sentinel on since_ms (the implementation has both a `flow_disabled` bool and `flow_disabled_since_ms` u32; the test must reflect the real contract). Table-driven: not flow-disabled returns false; elapsed < timeout returns false; elapsed >= timeout returns true; uint32_t wraparound handled (large since_ms, small now_ms).

OOM rollback paths are not tested: Flipper's heap hook (`memmgr.c`) panics rather than returning NULL. Documented contract.

**Optional separate commit (not part of main PR):** a host-gcc unit test runner. The main spec ships on-device tests; host-portable runner is fork-internal value (faster iteration during dev). Not promised in upstream PR scope.

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
| T14 | force_stop via reinit | Start scan; from other thread call furi_hal_bt_reinit() | ScanEnded(ForceStoppedForReinit); no hardfault |
| T15 | Heap leak | Start/stop scan ×1000; assert FreeRTOS task-stack high-water mark on ble_event_thread | Heap delta near zero; stack high-water within budget |
| T16 | Stats accuracy | 60 s in busy environment; cross-reference phone scanner. NOTE: ±10% only on a desk; metal-shelf RF env can cause larger variance | events_received within ~10% (open env) |
| **T17** | **Race stress** | 60 s loop: start/stop on thread A while 1 kHz timer posts mock reports | No use-after-free, no mutex assertion failures |
| **T18** | **Concurrent BLE consumer** | Run scan with BLE Serial (qFlipper open) | No deadlock; GATT stays responsive |
| **T19** | **Controller edge values** | Iterate interval = {0x0004, 0x4000}; window <= 0.95*interval; window > interval (rejection) | Document silent-clamping behavior; rejection path returns ControllerReject; ETSI duty rejection returns InvalidParameter |
| **T20** | **Co-channel WiFi interference** | Scan with active WiFi iPerf load on 2.4 GHz channel 6 (overlaps BLE channel 38) | events_received drop < 30% vs clean baseline |
| **T21** | **Starting-window early event race** | Hook into HCI dispatch to inject an event during the [phase=Active store] → [scan_enable returns] window | Event delivered, NOT dropped to events_dropped_phase |
| **T22** | **Dual auto-stop precedence** | Configure cfg.duration_ms=5500 + manually backpressure to fire BackpressureTimeout~5000ms; verify which reason wins | First-write reason reported; pending_end_reason determines outcome deterministically |
| **T23** | **ended_callback reentrancy** | Inside ended_callback, attempt start_scan and stop_scan | Both return Reentrant |
| **T24** | **force_stop public API** | Call furi_hal_bt_scan_force_stop from a non-owner thread | Scan stops cleanly; ScanEnded fires with ForceStoppedForReinit |
| **T25** | **Stats persistence** | start → produce events → stop → query stats; verify retained until next start_scan | Stats values persist; stats_reset() zeros them |
| **T26** | **Thread-liveness mitigation (F1)** | FAP-style: spawn thread that calls start_scan, then exits without stop_scan | Within ~100 ms of next event, scan auto-stops; no hardfault; ScanEnded fires with ForceStoppedForReinit |

T17 is critical for catching race bugs that two-mutex designs ship with. T18 is the single biggest upstream-rejection risk to mitigate before submission. T26 verifies the F1 mitigation works under the actual orphaned-FAP scenario.

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

The test FAP ships as `FlipperAppType.EXTERNAL` (a real FAP), located at `applications/debug/ble_scan_test/`. NOT `FlipperAppType.DEBUG` — that would bake it into the firmware image and increase binary size, which we want to avoid. (Note: `bt_debug_app` is `FlipperAppType.DEBUG`, so it's not the right precedent for what we're shipping; cite `applications_user/` FAP precedent instead.)

In PR description: "Test FAP `ble_scan_test` ships as a sideloadable FAP under `applications/debug/`. Strict test harness — not a user-facing scanner. End-user scanner is a separate FAP project at `applications_user/ble_scanner` (downstream). Removable from the firmware tree via single .fam-directory deletion if not desired."

The test FAP itself MUST call `furi_hal_bt_stop_scan()` in its scene-exit handler. If the test FAP is the first to trigger F1, users testing the test will reboot their devices. Stated as an explicit invariant in the test FAP's README.

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
MU_TEST(test_validate_struct_size_below_min) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    cfg.struct_size = 1;
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(&cfg));
}
MU_TEST(test_validate_struct_size_above_max) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    cfg.struct_size = sizeof(FuriHalBtScanConfig) + 16;  /* future version */
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(&cfg));
}
MU_TEST(test_validate_etsi_duty_cycle) {
    FuriHalBtScanConfig cfg = FURI_HAL_BT_SCAN_CONFIG_DEFAULT;
    cfg.window_ms = 100; cfg.interval_ms = 100;  /* 100% duty: rejected */
    mu_assert_int_eq(FuriHalBtScanErrorInvalidParameter,
                     validate_config(&cfg));
    cfg.window_ms = 95; cfg.interval_ms = 100;  /* exactly 95%: accepted */
    mu_assert_int_eq(FuriHalBtScanErrorNone,
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
MU_TEST(test_allowlist_multi_entry) {
    uint8_t list[3][6] = {
        {0x11,0x11,0x11,0x11,0x11,0x11},
        {0x22,0x22,0x22,0x22,0x22,0x22},
        {0x33,0x33,0x33,0x33,0x33,0x33},
    };
    uint8_t hit_second[6]   = {0x22,0x22,0x22,0x22,0x22,0x22};
    uint8_t miss_outside[6] = {0x44,0x44,0x44,0x44,0x44,0x44};
    mu_check(allowlist_permits(hit_second, list, 3));
    mu_check(!allowlist_permits(miss_outside, list, 3));
}

/* test_backpressure_timeout.c
 *
 * Function under test:
 *   bool should_force_stop_backpressure(
 *       bool flow_disabled,
 *       uint32_t now_ms,
 *       uint32_t since_ms,
 *       uint32_t timeout_ms);
 *
 * The flow_disabled bool is the authoritative "is backpressure on" signal;
 * since_ms by itself can validly be 0 (boot-time tick).
 */
MU_TEST(test_backpressure_not_disabled) {
    mu_check(!should_force_stop_backpressure(false, 10000, 5000, 5000));
}
MU_TEST(test_backpressure_under_timeout) {
    mu_check(!should_force_stop_backpressure(true, 8000, 5000, 5000));
}
MU_TEST(test_backpressure_at_timeout) {
    mu_check(should_force_stop_backpressure(true, 10000, 5000, 5000));
}
MU_TEST(test_backpressure_wraparound) {
    /* now wrapped past UINT32_MAX; since_ms close to UINT32_MAX.
     * Unsigned (now - since) gives correct elapsed time across wraparound. */
    mu_check(should_force_stop_backpressure(true, 1000, 0xFFFFE000, 5000));
}
MU_TEST(test_backpressure_disabled_with_zero_since) {
    /* since_ms == 0 is a valid boot-time timestamp; flow_disabled controls. */
    mu_check(should_force_stop_backpressure(true, 6000, 0, 5000));
}
```

## Appendix B: On-device test FAP layout

```
applications/debug/ble_scan_test/
├── application.fam                    # apptype=FlipperAppType.EXTERNAL, fap_category="Debug"
├── ble_scan_test.c                    # entry point, scene navigation
│                                       # MUST call furi_hal_bt_stop_scan() in scene-exit / app-exit
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
│   ├── t15_heap_leak.c                # 1000 iterations + stack high-water assertion
│   ├── t16_stats_accuracy.c
│   ├── t17_race_stress.c
│   ├── t18_concurrent_ble_consumer.c
│   ├── t19_controller_edge_values.c
│   ├── t20_co_channel_wifi.c
│   ├── t21_starting_window_race.c
│   ├── t22_dual_autostop_precedence.c
│   ├── t23_ended_callback_reentrancy.c
│   ├── t24_force_stop_public.c
│   ├── t25_stats_persistence.c
│   └── t26_thread_liveness_mitigation.c
├── views/
│   ├── menu_view.c                    # test selection
│   └── results_view.c                 # pass/fail display, log scroll
└── README.md                          # framing for upstream PR
```

Tests T1-T9, T15, T17, T21-T26 are fully automated. T10-T13, T16, T18-T20 require human verification of external state (visible advertising, paired phone, WiFi load source, etc.); FAP displays "verify and press OK" prompts.

## Appendix C: Regression checklist

See section 5.3.

## Appendix D: Recovery procedures

See section 4.6.

## Appendix E: Tuning guide

See section 4.5.

---

## Open items for plan-writing phase

These are implementation details that will surface during the writing-plans phase, not design questions:

- **Exact `gap.c` line numbers** for inserting the GAP_OBSERVER_ROLE addition (verified `gap.c:361`).
- **Exact placement** of `furi_hal_bt_scan_force_stop()` call in `furi_hal_bt.c::furi_hal_bt_reinit()` (verified `furi_hal_bt.c:202`).
- **`./fbt format`** output review for the entire patch before each commit.
- **api_symbols.csv sorted insertion**: identify insertion line numbers for the new entries during plan phase.

---

## Footnotes / consult attribution

This spec integrates findings from the following review pass:

- **Brainstorming-phase consults (sections 2–5):** 5 embedded-engineer (API shape, guarding, data flow, errors+limitations, testing) + 1 rf-engineer (scan parameter defaults).
- **Verification consults:** 1 embedded-engineer on Momentum flash budget; 3 source verifications (Momentum HAL surface, Unleashed CI platform, FAP loader unload hook).
- **Whole-spec audit (revision pass):** architect (module structure, commit composition, forward-compat), embedded-engineer (cross-section consistency, race holism, struct_size validation logic), rf-engineer (defaults, RPA wording, RSSI accuracy, regulatory), code-reviewer (naming, type design, test code).

Key audit-driven changes folded in: ScanEnded callback addition; struct_size forward-compat with proper validation contract (≥MIN_SIZE && ≤sizeof(current), not strict equality); private gap accessors via gap_internal.h (not public api_symbols.csv); F1 mitigation via in-dispatcher thread-liveness check; CFG_TLBLE_EVT_QUEUE_LENGTH bump; ETSI duty-cycle hard validation; Starting-window event ordering fix; dual-auto-stop precedence resolution; first-write-wins pending_end_reason; furi_hal_bt_scan_last_hci_status() query function (replacing out-param); commit series restructured 9→11 commits; test infrastructure relocated to PLUGIN-style App() block under unit_tests; T20–T26 added; Appendix E tuning guide added.

End of design spec.
