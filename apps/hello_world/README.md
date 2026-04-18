# hello_world

Minimal FAP that opens a GUI viewport, draws a greeting, and exits on BACK.

Serves as the template for new apps in this monorepo — copy this directory,
rename the `appid` / `entry_point` / source file, and you have a starting
point.

## Build

From this directory:

```bash
ufbt
# produces dist/hello_world.fap
```

## Flash (from carbon, with Flipper connected)

```bash
ufbt launch
```

## Icon

`images/hello_10px.png` must be exactly **10x10 px, 1-bit PNG** (white = lit
pixel, black = off). `ufbt` rejects other sizes for `fap_icon`.
