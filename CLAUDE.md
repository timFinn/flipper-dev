# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Purpose

Monorepo of **FAP applications** (external C apps) for the Flipper Zero. Development happens on this Linux dev box; flashing and on-device testing happen on a separate machine (`carbon`) with the physical Flipper attached.

## Layout

```
apps/<appid>/           One FAP per subdirectory. Each is a self-contained
                        ufbt project with its own application.fam manifest.
apps/<appid>/images/    Icons + fap_icon_assets (1-bit PNGs, strict sizes).
scripts/sdk-update.sh   Pins the ufbt SDK channel/version; run on any host
                        that builds, so dev box and carbon stay in sync.
Makefile                Monorepo driver — iterates apps/, shells out to ufbt.
```

A new app is always created by copying an existing app directory (start from `hello_world`) and renaming `appid`, `entry_point`, and the C source file. Do not try to share code across apps via relative includes — each FAP links independently.

## Common Commands

All commands assume `ufbt` is installed (`pipx install ufbt`). The first build on a fresh machine downloads the SDK to `~/.ufbt/`.

```bash
# One-time / after pulling: pin the SDK to the repo's expected version.
./scripts/sdk-update.sh

# Build every app in the monorepo.
make

# Build one app.
make APP=hello_world
# …or the shortcut:
make hello_world

# Build + flash a specific app. ONLY works on a host with a Flipper attached
# (i.e. carbon, not this dev box). Requires the Flipper in normal mode with
# qFlipper closed so /dev/ttyACM0 is free.
make launch APP=hello_world

# Clean all build dirs.
make clean

# Direct ufbt usage (inside an app dir) — equivalent to the make targets:
cd apps/hello_world
ufbt                    # build → dist/hello_world.fap
ufbt launch             # build + deploy (carbon only)
ufbt vscode_dist        # generate .vscode/ with working clangd + intellisense
ufbt cli                # open Flipper CLI over USB
```

## Dev-box vs. Carbon Split

- **This dev box (remote):** edit, build, run static analysis. `ufbt` builds fine here; the `.fap` artifact in `apps/<name>/dist/` is what you copy to `carbon` (via git push/pull or scp) for flashing. Hardware-dependent targets (`ufbt launch`, `ufbt cli`, `ufbt devboard_flash`) will fail here because there's no Flipper on USB — that's expected, not a bug to fix.
- **Carbon:** clone the repo, run `./scripts/sdk-update.sh` once to match SDK versions, then `make launch APP=<name>` for flash-and-test cycles. Keep qFlipper *closed* while flashing — it holds `/dev/ttyACM0`.
- **SDK drift is the main sync hazard.** If a build succeeds on one host and fails on the other with unresolved symbols or struct-size errors, re-run `sdk-update.sh` on both before debugging further.

## FAP Manifest (`application.fam`) Essentials

`application.fam` is Python syntax evaluated by `fbt`, not JSON/TOML. The fields that actually matter for an external app:

- `apptype=FlipperAppType.EXTERNAL` — this is what makes it a loadable `.fap` rather than built-in firmware. Do not change it.
- `entry_point="foo_app"` — must match a `int32_t foo_app(void*)` symbol in the C sources.
- `requires=[...]` — runtime service records (`"gui"`, `"storage"`, `"notification"`, etc.). Missing entries cause `furi_record_open` to return NULL at runtime, not at build time.
- `stack_size` — default 1 KiB is tight; bump to `2 * 1024` or `4 * 1024` for anything that allocates GUI scenes or parses files.
- `fap_icon` — **must be exactly 10x10, 1-bit PNG**. ufbt will reject other dimensions or bit depths with a cryptic error.
- `fap_category` — string used as the subfolder name under `Apps/` on the device. Use existing categories (`Tools`, `GPIO`, `Sub-GHz`, `NFC`, `Examples`) so the app shows up in the expected menu.

## Coding Conventions for FAPs

- **FURI first, libc second.** Use `furi_message_queue_*`, `furi_thread_*`, `furi_record_open/close`, `furi_delay_ms`, etc. Stock libc calls work but don't integrate with the scheduler — avoid `malloc`/`free` in favor of `furi_alloc` for anything freed from a different context than it was allocated.
- **Always pair `furi_record_open` with `furi_record_close`** on every exit path. Leaking a record handle is the #1 cause of apps that work once then hang on second launch.
- **View hierarchy:** simple apps use a raw `ViewPort` + draw/input callbacks (see `apps/hello_world/`). Multi-screen apps should use `ViewDispatcher` + `SceneManager` from `gui/scene_manager.h` — don't roll your own state machine on top of ViewPort.
- **Input handling belongs in a queue, not the callback.** The input callback runs in the GUI thread; do the work in your app's main loop after `furi_message_queue_get`. The `hello_world` template shows the minimum pattern.

## When Adding a New App

1. `cp -r apps/hello_world apps/<new_appid>`
2. Rename the `.c` file and update `appid` + `entry_point` + `fap_description` + `fap_icon` path in `application.fam`.
3. Replace the icon PNG (still 10x10, 1-bit).
4. `make APP=<new_appid>` — if it builds on the dev box, it will build on carbon.

## Known Gotchas

- `ufbt devboard_flash` prints a spurious `missing manifest (application.fam)` warning when run outside an app dir. It is harmless for the WiFi devboard flash path; the actual flash uses `wifi_board.py`, not the app manifest.
- Build artifacts in `apps/*/dist/` and `apps/*/.ufbt/` are gitignored. Don't commit `.fap` files — they're reproducible from source.
- `ufbt -c` (clean) wipes the per-app `.ufbt/` cache, not the global SDK. Use `rm -rf ~/.ufbt` only if the SDK itself is corrupted.
