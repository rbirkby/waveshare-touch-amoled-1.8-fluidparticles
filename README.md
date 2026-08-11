# 3D fluid simulation on the Waveshare Touch-AMOLED-1.8 boards

A tilt/shake reactive 3D particle fluid running bare-metal, rendered with a custom
software rasteriser straight to a 368×448 AMOLED over QSPI.

It builds and runs on **two different boards, from one shared source tree**:

- **Waveshare ESP32-S3-Touch-AMOLED-1.8** (Xtensa LX7, ESP-IDF)
- **Waveshare RP2350-Touch-AMOLED-1.8** (Cortex-M33, Pico SDK)

The screen is the front glass of a virtual box. The box is 368 px wide, 448 px tall
and 96 px deep — depth 0 is the glass, depth 96 is the back of the case. Roughly 600
fluid particles live inside it. The accelerometer tells the box which way is down, the
gyroscope tells it how it is spinning, and the fluid responds. Particles are deep blue
when still and shift through cyan and amber to hot orange-red as they speed up.

Measured on hardware:

| | physics | rendering | RAM free |
|---|---|---|---|
| ESP32-S3 @ 240 MHz | 30 Hz (30.3 ms) | 53 fps | ~182 KB internal |
| RP2350 @ 200 MHz | 30 Hz (30.7 ms) | 53 fps | ~316 KB |

Both land in the same place, for different reasons. The RP2350 does the solver maths
faster despite the lower clock, because the Cortex-M33's FPv5 unit has **hardware
single-precision divide and square root**, which the Xtensa LX7 lacks — and the solver
is almost entirely `sqrt` and reciprocal work. Its rendering, meanwhile, is limited by
the PIO bus rather than the CPU, and is deliberately capped just under the panel's own
60 Hz refresh, because drawing faster than the panel refreshes achieves nothing.

---

## Part 0 — One source tree, two chips

Nothing above the driver layer knows which chip it is on:

```
fluid3d/
  src/                    portable C — zero platform headers
    app.c    orchestration: what happens each tick
    fluid.c  the 3D Position Based Fluids solver
    render.c the software rasteriser
    config.h tunables shared by both boards
    hal.h    the platform contract  (time, log, random, alloc, mutex)
    drivers.h the hardware contract (display, IMU, button, touch)
  platform/esp32s3/       ESP-IDF project  — board_config.h + 6 files
  platform/rp2350/        Pico SDK project — board_config.h + 6 files
```

`src/` is ~1,600 lines and is the entire value of the project. It compiles unmodified
for both targets. Each `platform/` directory supplies a `board_config.h` (pins, I²C
addresses, clock, IMU axis mapping) and implements the two headers.

**Scheduling is deliberately *not* abstracted.** That is the one design decision worth
explaining. It is tempting to hide the RTOS behind a `task_create()` shim, but the two
chips genuinely differ:

- The ESP32-S3 has FreeRTOS, and the *right* answer there is four tasks pinned to
  specific cores with specific priorities, plus an explicit yield so the idle task can
  feed the watchdog.
- The RP2350 has no RTOS at all. The right answer there is a bare superloop on each of
  the two cores.

A shim would have to model priorities, core affinity, watchdogs and tick rates — and
would be a leaky imitation of FreeRTOS that the RP2350 pays for and does not want. So
instead the portable `app.c` exposes five hooks and each platform's `main.c` decides
when to call them:

```c
app_init();          // bring up drivers, seed the fluid
app_input_poll();    // read IMU + touch, feed the solver   (~200 Hz)
app_sim_step();      // advance the fluid one fixed step    (30 Hz)
app_render_frame();  // draw and stream one frame           (free-running)
app_stats_tick();    // print the timing line               (every 3 s)
```

`app.c` owns the *policy* (what a frame consists of), each `main.c` owns the
*mechanism* (which core and when). The ESP32-S3's `main.c` is 78 lines of task wiring;
the RP2350's is 70 lines of superloop. That is the whole platform-specific scheduling
story.

---

## Part 1 — How you program these boards

### What the boards actually are

| | ESP32-S3-Touch-AMOLED-1.8 | RP2350-Touch-AMOLED-1.8 |
|---|---|---|
| MCU | ESP32-S3R8, dual Xtensa LX7 @ 240 MHz | RP2350A, dual Cortex-M33 @ 200 MHz |
| FPU | single precision, **no** div/sqrt | FPv5-SP-D16, **hardware div + sqrt** |
| RAM | 512 KB SRAM + 8 MB octal PSRAM | 520 KB SRAM, **no PSRAM** |
| Flash | 16 MB QIO | 16 MB QSPI |
| Display | AMOLED 368×448 RGB565, `esp_lcd` | AMOLED 368×448 RGB565, PIO |
| Display transport | `esp_lcd` hardware QSPI | **PIO** QSPI (synthesised) |
| IMU | QMI8658 @ 0x6B | QMI8658 @ 0x6B |
| Touch | FT3168 @ 0x38 | FT3168 @ 0x38 |
| PMIC | AXP2101 @ 0x34 | AXP2101 @ 0x34 |
| IO expander | **TCA9554 @ 0x20** | **none** |
| Toolchain | ESP-IDF 5.5.1 | Pico SDK 2.1.1 |
| Flashing | native USB-Serial-JTAG | native USB, `picotool` |

Verified pin maps, both confirmed on hardware rather than taken from a wiki:

```
ESP32-S3   QSPI  CS=12 PCLK=11 D0=4 D1=5 D2=6 D3=7      I2C SDA=15 SCL=14
           panel reset + touch reset are on the TCA9554, not GPIOs
           touch INT = GPIO 21

RP2350     QSPI  CS=9  PCLK=10 D0=11 D1=12 D2=13 D3=14  I2C1 SDA=6 SCL=7
           panel reset = GPIO 15, PWR_EN = GPIO 17 (must be high first)
           panel TE = GPIO 16 (undocumented; found by scanning for a 60 Hz pulse)
           touch reset = GPIO 5, touch INT = GPIO 4
```

Three differences do all the damage, and all three are handled inside
`platform/*/board.c` so the portable code never sees them:

1. **The RP2350 board has no IO expander.** Touch and panel reset are plain GPIOs. On
   the ESP32-S3 they are expander bits that are asserted out of power-on-reset, so the
   expander must be released *before* the display is initialised.
2. **The RP2350 board has a `PWR_EN` line on GPIO 17** that must be driven high before
   the panel will answer at all. Miss it and the display is simply dead with no error.
3. **The RP2350 has no hardware QSPI available** — its QMI block is committed to the
   flash — so the display bus is synthesised with PIO (see Part 2.5).

### Neither board has a GPIO user button

This surprised me on both boards, so it is worth stating plainly: **the user button is
not wired to a GPIO on either device.** It goes to the AXP2101's PWRKEY pin.

On the RP2350 board I confirmed this the hard way. Waveshare's schematic labels GPIO 18
"SYS_OUT", which reads like a candidate, but it sits permanently low even with a
pull-up — it is a power-latch output. A sweep of *every* free GPIO with pull-up and
pull-down found no pin that floats the way an idle active-low button would. There is no
GPIO button to find.

So both boards do the same thing: **poll** the PMIC's `INTSTS2` register (0x49) for the
PWRKEY edge flags, and act on the release edges. Polling rather than claiming the PMIC's
interrupt line matters, because it leaves the PMIC's own long-press power-off intact —
holding the button for ~6 s always kills the board no matter what the firmware is doing,
which is your guaranteed way out of a bad flash.

One trap: the PMIC latches a power-on event during boot, so `INTSTS2` already reads
non-zero before the firmware has looked at it. Clear the status registers at init or the
very first poll fires a phantom reset. They are write-1-to-clear.

### The established workflow

There are three realistic ways to program these Waveshare AMOLED boards. They are all
in active use, so it is worth knowing why you would pick each:

1. **Arduino / PlatformIO with `Arduino_GFX` or `LovyanGFX`.** Fastest to first pixel.
   This is what most of the demos you see on social media use. You lose fine control
   over DMA and task placement, which matters once you are pushing a full frame every
   16 ms.
2. **ESP-IDF + the vendor BSP + LVGL.** This is what Waveshare ships in their examples
   repo. Excellent for UI (buttons, sliders, menus). LVGL is a retained-mode widget
   toolkit though — for a full-screen particle field that changes every pixel every
   frame it is pure overhead.
3. **ESP-IDF bare metal.** You talk to `esp_lcd` directly and own the frame loop.
   More work, but it is the only option that gives you the whole CPU and full control
   of the DMA pipeline.

**This project uses option 3.** Every particle moves every frame, so there is nothing
for a widget toolkit to cache, and the CPU budget is the entire problem.

### macOS setup — ESP32-S3

The machine already had ESP-IDF v5.2.2, but **Waveshare's board support requires
IDF ≥ 5.5** (their `idf_component.yml` declares `idf: ">=5.5,<6.1"`). So:

```bash
mkdir -p ~/esp/v5.5.1 && cd ~/esp/v5.5.1
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
```

Then, in every new shell:

```bash
. ~/esp/v5.5.1/esp-idf/export.sh
cd fluid3d/platform/esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor      # Ctrl-] to exit
```

Because the board uses **native USB-Serial-JTAG**, `idf.py flash` resets it into the
bootloader by itself. You never need to hold BOOT. `sdkconfig.defaults` keeps the
console on USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`); if you switch the
console to UART you lose the auto-reset and have to hold BOOT for every flash.

If the port ever refuses to appear, hold BOOT, tap RESET, release BOOT — that forces
ROM download mode.

### macOS setup — RP2350

Two pieces: a compiler and the SDK. Homebrew's `arm-none-eabi-gcc` (10.3 here) is new
enough — it accepts `-mcpu=cortex-m33 -mfloat-abi=hard -mfpu=fpv5-sp-d16`, which is what
gets you the hardware sqrt. There is no need for the newer standalone toolchain cask.

```bash
brew install arm-none-eabi-gcc cmake
git clone -b 2.1.1 --recursive https://github.com/raspberrypi/pico-sdk ~/pico-sdk
```

Build:

```bash
export PICO_SDK_PATH=~/pico-sdk
cd fluid3d/platform/rp2350
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build -j8
```

Flash **without touching any buttons** — `picotool` can force the board into BOOTSEL
over USB, which is much nicer than the usual hold-BOOTSEL-and-replug dance:

```bash
picotool load -f -x build/fluid3d.uf2
```

`-f` forces the reboot into BOOTSEL, `-x` runs the firmware afterwards. The SDK builds
its own `picotool` if you have not installed one; the copy at
`~/.pico-sdk/picotool/*/picotool` (installed by the VS Code extension) also works.

Two practical notes:

- **Back the factory firmware up before you overwrite it.** `picotool save -a
  backup/rp2350-factory-firmware.uf2` dumps the whole 16 MB flash, and restoring it is
  just another `picotool load`. The board ships with a Waveshare demo you cannot
  download again. That dump is deliberately **not** tracked in git -- it belongs to one
  physical board rather than to this project, and it is Waveshare's firmware rather than
  ours -- so keep a copy somewhere off this machine. It is mostly `0xFF` padding, so
  `gzip -9` takes it from 33 MB to about 700 KB, and `gunzip -k` restores it byte for
  byte.
- **Reading the serial output races the USB re-enumeration.** After `picotool load -x`
  the CDC port disappears and comes back a second or so later, so a naive `sleep` and
  open will hang on a stale device node. Poll `/dev/cu.usbmodem*` and retry the open
  until it succeeds.

### Finding out what is on the board

Rather than trusting a wiki page, `tools/hwprobe/` is a small diagnostic firmware that scans
the I²C bus, identifies each chip, brings the IMU up and dumps live readings, and
watches the PMIC's interrupt registers for button presses. Flash it if you ever need to
re-confirm a hardware assumption:

```bash
make hwprobe-esp32s3 && cd tools/hwprobe && idf.py -p /dev/cu.usbmodem1101 flash monitor

make hwprobe-rp2350
picotool load -f -x tools/hwprobe-rp2350/build/hwprobe.uf2

make hwprobe-rp2350-display
picotool load -f -x tools/hwprobe-rp2350-display/build/dispprobe.uf2
```

`tools/hwprobe-rp2350-display/` is the one to reach for if the screen is blank: it runs the
panel init, hunts for the TE pulse, alternates white and black frames while watching the
PMIC, and leaves the screen filled red. It tells you whether the panel is alive without
anyone having to interpret what they are looking at.

It is how the pin map, the I²C device list, the IMU axis orientation and the button
wiring in this project were all established. Two useful findings:

- **The touch controller is held in reset at power-on, and it is not a GPIO doing it.**
  A plain I²C scan of this board reports no touch hardware at all, on either the CST816
  (0x15) or the FT3168 (0x38) address. The reset line hangs off the **TCA9554 IO
  expander at 0x20, bit 2**, asserted out of power-on-reset. Release it and an FT3168
  answers at 0x38 (chip ID 0x64). The same expander drives the panel reset on bit 0, so
  the release has to happen *before* the display is initialised. This is by far the
  easiest thing to get wrong on this board.
- **The touch controller sleeps when idle.** Once nothing has touched it for ~10 s the
  FT3168 stops acknowledging its own address, and polling it blindly produces a steady
  stream of `I2C transaction unexpected nack` errors. Its interrupt line, **GPIO 21**, is
  the gate: pulled up when idle, driven low while a finger is down. Check the pin before
  touching the bus.
- **Screen-up, flat, the accelerometer reads `az ≈ −9.5 m/s²`**, so the IMU's +Z axis
  points *into* the screen — the same direction as the simulation's +Z. That is why
  gravity needs no correction on Z.
- **The IMU is mounted rotated 90° relative to the panel**, which is not documented
  anywhere in the vendor's ESP-IDF examples. The Arduino side of the repo gives it away:
  `examples/arduino-v2/examples/13_LVGL_Widgets` switches the display to its default
  upright orientation when `accel.x > +0.8 g`. An accelerometer at rest reads the "up"
  direction, so **IMU +X points at the top of the screen** and, for a right-handed set
  with +Z into the glass, **IMU +Y points to the right**. Hence
  `sim x = +imu y`, `sim y = −imu x`, `sim z = +imu z`. That matrix has determinant +1,
  so it is a proper rotation and the gyro's pseudo-vector transforms identically.
  Guessing this wrong by one sign makes the fluid fall *upwards*, so it is worth
  deriving rather than trying.

### Where the vendor sources live

The reference material is a 606 MB clone, so it is not kept in this workspace. To get
it back:

```bash
git clone https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8
git clone https://github.com/waveshareteam/RP2350-Touch-AMOLED-1.75    # RP2350
```

Waveshare have not published a *git repo* for the 1.8" RP2350 board, but they do ship a
demo pack for it, and **that pack is the authoritative reference — use it, not the
1.75" repo**:

```bash
curl -LO https://files.waveshare.com/wiki/RP2350-Touch-AMOLED-1.8/RP2350-Touch-AMOLED-1.8.zip
```

Read `C/01-LCD/lib/AMOLED/AMOLED_1in8.c` (panel init), `C/01-LCD/lib/QSPI_PIO/`
(the PIO bus) and `C/01-LCD/lib/Config/DEV_Config.h` (the pin map).

The 1.75" board is superficially similar and it is tempting to work from its repo, but
it differs in ways that produce a **silently blank screen**: it wants `0x3A = 0x05`
where the 1.8" wants `0x55`, it needs a 6-pixel column offset where the 1.8" needs none,
and its pin map is shifted by one. See 2.5c.

The parts worth reading are `examples/esp-idf/13_display_colorbar` (panel init),
`examples/esp-idf/92_qmi8658_imu` (IMU), and — after running `idf.py reconfigure` in any
example — the resolved `managed_components/waveshare__esp32_s3_touch_amoled_1_8/` and
`managed_components/espressif__esp_lcd_co5300/`, which are the authoritative pin map and
panel init sequence. This project pulls `espressif/esp_lcd_co5300` from the component
registry directly (see `platform/esp32s3/main/idf_component.yml`) and skips the BSP.

---

## Part 2 — How the application works

```
src/app.c        what happens on each tick        <- portable
  ├─ src/fluid.c    the 3D PBF solver             <- portable
  └─ src/render.c   software particle rasteriser  <- portable
        |
        |  hal.h / drivers.h
        v
platform/<target>/
     main.c      scheduling: which core, how often
     board.c     shared I2C bus, power rails, resets
     sensors.c   QMI8658 IMU + AXP2101 button
     touch.c     FT3168 touch
     display.c   panel init + DMA band streaming
     hal_*.c     clock, log, random, allocators, mutex
```

**ESP32-S3** — four FreeRTOS tasks:

| task | core | priority | period |
|---|---|---|---|
| `sim` | 1 | 5 | 33.3 ms (30 Hz) |
| `render` | 0 | 5 | free-running (~19 ms) |
| `input` | 0 | 6 | 5 ms |
| `stats` | 0 | 1 | 3 s |

**RP2350** — no RTOS, two superloops:

| core | does |
|---|---|
| 1 | `app_sim_step()` at a fixed 30 Hz, `app_input_poll()` every 5 ms in the gaps |
| 0 | `app_render_frame()` flat out, `app_stats_tick()` every 3 s |

Input lives on core 1 on the RP2350 for a specific reason: a frame takes longer than an
input tick, so polling from the render loop would quietly drop the IMU rate to the frame
rate. Core 1 is idle between solver steps and has the time going spare. It also means
I²C is only ever touched from core 1 and the display only from core 0, so the two cores
never contend for a peripheral and no bus lock is needed.

The simulation and the renderer are fully decoupled. The solver writes a snapshot
(position + normalised speed per particle) under a mutex; the renderer copies it and
draws. They run at different rates and neither waits for the other.

### 2.1 Turning motion into gravity

This is the part people usually over-engineer. You do **not** need sensor fusion, a
Kalman filter, or a quaternion attitude estimate.

An accelerometer does not measure acceleration — it measures **specific force**, which is
gravity *minus* the linear acceleration of the case. So the apparent gravity felt by
anything inside the box is simply the negation of what the accelerometer reads:

```c
g_sim = -a_measured
```

Two lines, and you get both behaviours for free:

- **Tilt** — the gravity vector rotates in the board frame, and the fluid runs downhill.
- **Shake** — jerking the board produces a large specific-force reading, which becomes a
  large apparent gravity in the opposite direction, and the fluid slams the other way.
  This is exactly right: it is why coffee jumps out of a mug when you stop suddenly.

Rotation needs two extra terms, because the box frame is non-inertial:

```c
a_fictitious = -2·(ω × v)            // Coriolis  — makes spinning stir the fluid
             - ω × (ω × r)           // centrifugal — throws it to the outside
```

Spin the board flat and the fluid winds up into a vortex.

The gyroscope has a large zero bias (~9 dps on X on this unit), so `sensors.c` averages
100 samples at boot and subtracts that offset forever after. **Keep the board still for
the first second after power-on** or the fluid will drift.

Both signals get a light low-pass filter — the raw IMU output is noisy enough to make
the fluid buzz.

### 2.2 The fluid — Position Based Fluids

The solver is **PBF** (Macklin & Müller 2013), not classic SPH. The reason is stability.
Weakly-compressible SPH computes a pressure force from a stiff equation of state, and
stiff forces need tiny timesteps — you would be looking at 1/500 s or smaller. On a
240 MHz microcontroller that is a non-starter.

PBF reframes incompressibility as a *constraint* and solves it by moving particles
directly instead of applying forces. It is unconditionally stable at 1/30 s. One step:

1. **Predict.** Apply gravity and the fictitious forces to velocity, then advance
   positions: `q = p + v·dt`. All later work happens on the predicted positions `q`.
2. **Find neighbours** (below).
3. **Solve, twice:**
   - *Density pass.* For each particle, sum the smoothing kernel over its neighbours to
     get density `ρ`, then compute a Lagrange multiplier `λ` that says how far the local
     packing is from rest density.
   - *Position pass.* Push each particle along the gradient of the constraint, scaled by
     `λ`. Dense clumps expand, thin regions contract.
   - Two iterations is the sweet spot here — one leaves the fluid visibly springy, three
     costs 8 ms with no visible benefit.
4. **Derive velocity** from how far the solver actually moved each particle:
   `v = (q − p)/dt`. This is the trick that makes PBF work: velocity is an *output* of
   the constraint solve, not an input to it.
5. **XSPH viscosity.** Blend each particle's velocity toward its neighbourhood average.
   Without it the result looks like a bag of bouncing marbles rather than a liquid.
6. **Walls.** The position clamp in step 3 already absorbed all wall-normal velocity, so
   a fraction of the pre-solve velocity is put back as a bounce, and the tangential
   component is scaled down to fake friction.

Two details that matter:

- **Density is an inequality constraint.** PBF enforces `ρ ≤ ρ₀` — it resists compression
  but never pulls. If you solve it symmetrically, every particle with a deficient
  neighbourhood (i.e. every particle on the surface) gets sucked violently inward, and
  since velocity is derived as `(q − p)/dt` that shove is multiplied by 30 on its way
  into the velocity field. The result is a surface layer of specks skittering around at
  high speed instead of flowing. Clamping `C` to be non-negative is what makes the
  surface behave like a surface.
- **The positional correction is capped** (`MAX_CORRECTION`) for the same reason: at
  30 Hz, every unit of position change becomes 30 units of velocity, so one bad shove
  turns into a particle that rockets across the box.
- **Artificial pressure (`s_corr`).** The spiky kernel has zero gradient at zero
  distance, so particles in sparse regions pull themselves into strings. A small
  repulsive term proportional to `(W(r)/W(Δq))⁴` keeps the spacing even and, as a bonus,
  produces surface tension.
- **Rest density is measured, not guessed.** At init the code lays out a perfect lattice
  and sums the kernel over it. That means you can change `SPH_SPACING` freely without
  having to re-derive a magic constant.

**Everything internal is in units of the smoothing radius H (40 px).** So H = 1, the
kernel constants stay near 1.0, and the arithmetic keeps its precision. Positions are
converted to pixels only when publishing the snapshot.

### 2.3 Finding neighbours fast

Checking all pairs would be 600² = 360,000 distance tests per pass. Instead:

1. Space is diced into a uniform grid of cells exactly H on a side, so any neighbour
   within H must be in one of the 27 cells around you.
2. Particles are **counting-sorted** into grid order and then physically permuted — all
   13 attribute arrays are reordered. This costs a copy but means that from then on
   every neighbour loop walks memory almost linearly instead of chasing pointers.
3. Cells are laid out contiguously along x, so the three x-cells of a row are one
   contiguous index range. **27 cell lookups collapse to 9.**
4. The neighbour list is built **once per step and cached** (max 28 each). All five
   passes that follow reuse it. That turns five expensive searches into one.

### 2.4 Making the maths fast — the biggest win in the project

The first working version ran the solver at **166 ms per step**, 10× too slow. Per-phase
timers pointed at the constraint solve, and disassembling the object file explained why:
the LX7 FPU can multiply and fused-multiply-add in hardware, but **it has no divide and
no square root**. Both compile into library calls, and the inner loop had one `sqrtf` and
three divisions per particle pair.

The fix was to remove all four:

```c
// Two Newton-Raphson steps on the classic bit-trick seed. ~1e-6 relative
// accuracy using nothing but multiplies, which is far more than a fluid needs.
static inline float rsqrt_fast(float x)
{
    union { float f; int32_t i; } u = { .f = x };
    const float half = 0.5f * x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - half * y * y);
    y = y * (1.5f - half * y * y);
    return y;
}
```

With `inv_r = rsqrt_fast(r2)` in hand, `r` becomes `r2 * inv_r` and every `/r` becomes
`* inv_r`. The remaining loop-invariant divisions (`1/ρ₀`, `1/W(Δq)`) were hoisted out.
**166 ms → 77 ms.**

The rest of the gap was closed by:

- **Packing predicted positions.** `qx[j]`, `qy[j]`, `qz[j]` in three separate arrays
  means every neighbour fetch touches three different cache lines. Packing them as
  `{x, y, z, pad}` with a 16-byte stride makes it one.
- **Rebalancing the particle scale.** Solver cost scales with the *cube* of particle
  spacing, because that is how neighbour count grows. Going from 0.50 H to 0.55 H
  spacing cuts the neighbour count from ~33 to ~25 — a 25 % saving — and the particles
  are drawn slightly larger to compensate, which looks no worse.
- **600 particles at 30 Hz** rather than 1100 at 60 Hz. **77 ms → 29.6 ms**, inside the
  33.3 ms budget.

**A negative result worth recording:** splitting the solver across both cores was
implemented and measured. It did not help. The renderer already saturates core 0, so
handing the helper half the solver simply stole frames back — the sim went 24 → 26 Hz
while rendering fell 50 → 28 fps. The chip is CPU-bound *as a whole*, not core-bound.
The range-based function signatures are still in `fluid.c` and `parallel_for()` is a
one-line inline, so the experiment is easy to repeat, but the split is off.

### 2.5 Rendering

**A full framebuffer does not fit.** 368 × 448 × 2 bytes = 330 KB, and the ESP32-S3 only
has 512 KB of internal SRAM. PSRAM would fit it but is far too slow to rasterise into.

So the frame is drawn as **14 horizontal bands of 368 × 32**, and the bands are streamed
to the panel with **ping-pong DMA**: while band *n* is in flight over QSPI, the CPU is
already rasterising band *n+1*. A counting semaphore fed by the panel's
`on_color_trans_done` ISR keeps the two in lockstep. Three buffers total (~70 KB) instead
of 330 KB, and the transfer time hides almost entirely behind the drawing time.

The hard ceiling: QSPI at 40 MHz in quad mode is 20 MB/s, so a 330 KB frame takes
~16.5 ms no matter what. That caps the display at roughly 60 fps.

Inside a band:

- **Projection.** Perspective divide with the focal length in `PERSPECTIVE_F`, so
  particles near the glass are bigger. Depth also darkens them, which is most of what
  sells the 3D.
- **Depth sorting without a depth buffer.** Particles are counting-sorted into 32 depth
  buckets, walked near-to-far, and *prepended* onto each band's linked list. Prepending
  reverses the order, so each list ends up sorted far-to-near — exactly the painter's
  algorithm draw order, for the price of a counting sort.
- **Sprites are precomputed spans.** For each radius, each row stores an opaque
  half-width and an outer half-width. A particle body is therefore a tight run of stores
  with half-alpha blending on only the one or two edge pixels — no per-pixel distance
  test.
- **Colour is a 16 × 16 lookup table** indexed by (speed level, depth-shade level), so
  the inner loop never does channel arithmetic. The ramp is deep blue (25, 70, 235) →
  cyan at 35 % → amber at 70 % → hot orange-red (255, 55, 25). The speed driving it is
  temporally smoothed (`COLOUR_SMOOTHING`) — instantaneous speed is twitchy enough that
  mapping it straight to hue makes the colours strobe even when the flow is smooth.
- **The byte swap is unavoidable.** The panel wants big-endian RGB565 and the CPU is
  little-endian, and RGB565's green channel straddles the byte boundary so you cannot
  blend in swapped space. Bands are rasterised in native order and swapped on the way
  into the DMA buffer, two pixels at a time via 32-bit word operations — about 1 ms per
  frame. `display_needs_byte_swap()` lets a transport opt out of it; both of these
  panels want the swap, which the vendor's `LV_COLOR_16_SWAP 1` confirms.

### 2.5b Getting pixels out — two very different transports

This is the one place where the two boards have essentially nothing in common, and it is
the most interesting part of the port.

**ESP32-S3.** There is a real QSPI peripheral, and `esp_lcd` drives it. You hand
`esp_lcd_panel_draw_bitmap()` a band and it DMAs it out. Easy.

**RP2350.** There is no hardware QSPI available — the QMI block is committed to the
external flash — so the bus has to be *synthesised in PIO*. PIO is a pair of tiny
programmable state machines with their own instruction set that run independently of the
CPU, and this is exactly what they exist for. The whole driver is six instructions:

```
.program qspi_data4          ; pixels: 4 bits per clock across D0..D3
.side_set 1
.wrap_target
    out pins, 4    side 0
    nop            side 1
.wrap
```

`out pins, 4` shifts four bits from the output shift register onto D0..D3, and the
`side 0` / `side 1` toggles the clock pin *in the same cycle*, for free. Two cycles per
nibble, so the bus runs at sys_clk / 2 = 100 MHz, or 50 MB/s. A second, near-identical
program does one bit per clock on D0 alone for commands and addresses, which the panel
requires. The two state machines share the clock and D0 pins and are enabled one at a
time.

The configuration that matters is `sm_config_set_out_shift(&c, false, true, 8)`:
shift **left** (MSB first, which is what SPI wants), autopull **on**, threshold **8** so
the state machine consumes exactly one byte per FIFO word. That last part is what lets
DMA stream bytes straight out of the band buffer in memory order with no repacking.

**The subtle bit — which byte lane?** Shifting left means the state machine takes bits
from the *top* of the shift register, so a CPU write has to be `put(value << 24)`. But
the DMA path uses `DMA_SIZE_8` writing to a word-aligned FIFO address, and the obvious
reading of the datasheet says a byte write lands in bits 7:0 — which would transmit
zeros. The two appear to contradict each other, yet the vendor's code demonstrably
works.

Rather than guess, I settled it on the hardware. Running the state machine at ~1.6 kHz
instead of 100 MHz makes each output nibble last hundreds of microseconds, which is slow
enough for the CPU to simply watch the pins with `gpio_get_all()`. Sending `0x9F`:

```
cpu <<24 : nibbles seen 9, F      <- byte reached bits 31:24
cpu plain: nibbles seen 0         <- byte sat in bits 7:0, transmitted nothing
dma  8bit: nibbles seen 9, F      <- same as <<24
```

Decisive: DMA_SIZE_8 does deliver into the top lane. That probe took ten minutes and
removed the single largest risk in the port — a byte-lane error would not have stalled
anything, it would just have painted convincing-looking garbage at full frame rate.

### 2.5c The blank screen, and how it was found without looking at it

The first RP2350 build ran perfectly by every measure the serial log could offer — 30 Hz
solver, 80 fps renderer, every driver initialised — and displayed **nothing at all**.
Four separate faults were involved, and the way each was settled is the useful part.

**Fault 1: two state machines instead of one.** I had written a 4-bit PIO program for
pixels and a 1-bit program for commands, selecting between them per transfer. Waveshare's
driver looks like it does the same — until you notice the 1-bit program is *commented
out*. They use one state machine for everything and fake single-wire mode by expanding
each command byte into four bytes in which only D0 is ever set:

```c
for (int i = 3; i >= 0; i--)
    put_byte((val >> (2*i) & 1) | ((val >> (2*i+1) & 1) << 4));
```

Each output nibble carries one bit on D0 and holds D1..D3 low. It costs four bytes per
command byte, which is nothing, and it means the state machine is enabled once and never
touched again. My version had to disable a state machine that could still be shifting,
which desynchronises its output shift register and corrupts every subsequent byte.

**Fault 2: the wrong colour-format byte.** I had taken the init sequence from Waveshare's
1.75" board, the closest published reference. That panel wants `0x3A = 0x05` for 16-bit
colour. **This one wants `0x55`.** One byte, and the difference between a picture and a
blank screen.

**Fault 3: the bus was too fast.** I ran the CPU at 200 MHz with a PIO divider of 1,
giving a 100 MHz QSPI clock. Waveshare clock their board at 150 MHz with the same
divider — a 75 MHz bus. I had unknowingly overclocked the panel by a third. The fix keeps
the CPU at 200 MHz for the solver and sets the divider to 2, for a 50 MHz bus.

**Fault 4** was the transfer-drain ordering described above.

The lesson from faults 2 and 3 is the same one: **a sibling product is not the same
product.** The fix was to stop extrapolating and fetch the board-specific demo pack
(`files.waveshare.com/wiki/RP2350-Touch-AMOLED-1.8/RP2350-Touch-AMOLED-1.8.zip`), which
contains `AMOLED_1in8.c` — the authoritative sequence for this exact panel. It also
confirmed the pin map I had been given by research, which was correct.

**Verifying the fix with nobody available to look at the screen.** Two probes did it:

- The byte-lane probe from 2.5b had already proven the correct bytes reach the wire.
- Register `0x35` asks the panel to emit a tearing-effect pulse once per refresh.
  Waveshare do not document a TE pin, so `tools/hwprobe-rp2350-display/` sends the init
  sequence and then scans every free GPIO for activity. **GPIO 16 toggles at ~60 Hz.**
  That pin only pulses if the panel is powered, has accepted the initialisation and is
  actively refreshing, so together with the byte-lane result it is strong evidence the
  display path is sound.

The probe also alternates full white and full black frames while reading the PMIC, on
the theory that an AMOLED draws far more current on white. That check was inconclusive
here — the system rail is regulated and the board was on USB power — but it costs
nothing and would be decisive on battery.

**One ordering bug worth recording**, because it is easy to write and hard to see: the
window-setting commands for band *N+1* go out on the *command* state machine, which
means disabling the *data* state machine. DMA completing only means the bytes reached
the FIFO — not that they reached the wire. Switching state machines at that moment
truncates the tail of band *N*, a few pixels wide, at the bottom of every band. The fix
is a `finish_transfer()` that waits for DMA, then drains the state machine, then raises
chip select, called before *both* buffer reuse and the next window write.

### 2.6 Touch

Touching the glass pushes the fluid away from your finger, hard enough to open a clear
hole in it. Getting that to feel strong took two corrections, both of which are worth
understanding because the obvious approaches quietly fail.

**A force field barely works.** The first version applied a repulsive acceleration and
the effect was disappointingly weak. The reason is structural: the density solver exists
*precisely* to cancel out anything that squashes particles together. Push on a blob of
incompressible fluid and it presses straight back, so you get a shallow dent no matter
how large the constant. Turning the strength up just fights the solver.

The fix is to stop applying a force and start applying a **positional constraint**,
exactly like the walls. Particles inside the finger are projected onto its surface, so
the solver has no choice but to respect it. And because PBF derives velocity from how far
each particle actually moved, the projection generates the outward jet for free — the
same trick the whole method is built on.

**The shape mattered too.** Modelling the finger as a hemisphere resting on the glass is
the physically honest choice, and it was wrong in practice. The case is only 96 px deep
and the fluid piles against whichever wall gravity points at, so with the board flat on a
desk the fluid sits at the *back* and a shallow dome at the glass misses it entirely. A
column spanning the full depth always connects, and reads as a rod pushed through the
fluid.

Measured with a synthetic touch at the centre of the screen (the solver was instrumented
to count particles inside the finger volume):

| | particles in column | mean speed | peak speed |
|---|---|---|---|
| idle | 35–36 | 0.022 | 0.026 |
| finger down, 0.5 s | 7 | 0.059 | 0.171 |
| finger down, 1.5 s+ | **0** | — | — |

The column empties completely and stays empty. A softer force field with a wider radius
is layered on top for the surrounding wake, along with a push along +Z away from the
glass — your finger is physically at z = 0, so a purely in-plane push would feel like
sliding a magnet under the screen.

The controller is polled every 20 ms rather than every 5 ms — the panel only refreshes
every ~19 ms, so reading faster would only load the I²C bus.

One defensive detail: the panel is taller than it is wide, so a coordinate pair that only
fits when transposed proves the controller reports its axes the other way round. That is
detected once and latched, instead of silently dropping every touch below y = 368 the way
a wrong fixed guess would.

### 2.7 The buttons

Both boards have two buttons and they are completely different animals:

- **BOOT / BOOTSEL** is a normal GPIO used by the ROM bootloader. The firmware does not
  touch it, which is what guarantees you can always recover the board.
- **The custom button is not a GPIO on either board.** It goes to the AXP2101 PMIC's
  PWRKEY pin. To see it you enable the PWRKEY interrupts in `INTEN2` (0x41) and then read
  `INTSTS2` (0x49), where bits 0–3 are positive edge, negative edge, long press and short
  press. The status registers are write-1-to-clear.

The input poll reads that register every 5 ms and calls `fluid_reset()` on a **release**
edge (negative edge or short press, mask `0x0A`) — reacting to the press edge instead
would also fire on the way into a long-press power-off.

Because both boards landed on the same PMIC, this ended up being one of the few drivers
that is line-for-line identical between the two ports rather than divergent.

As noted above, polling rather than taking the interrupt line deliberately leaves the
PMIC's own long-press power-off intact, and neither firmware touches the bootloader
button — so **reboot and reflash keep working on both boards no matter what the
simulation is doing.**

---

## Part 3 — Tuning

Everything portable lives in `src/config.h`; anything board-specific (pins, I²C
addresses, clock, IMU axis mapping) lives in `platform/<target>/board_config.h`, which
`config.h` includes at the bottom. So the table below applies to both boards.

| knob | default | effect |
|---|---|---|
| `PARTICLE_COUNT` | 600 | Dominates cost. Raise until `sim` drops below 30 Hz. |
| `PBF_ITERATIONS` | 2 | 1 = springy and fast, 3 = stiffer, +8 ms. |
| `SPH_SPACING` | 0.55 | Cost scales with the **cube** of this. Raise it to go faster. |
| `SIM_HZ` | 30 | Must satisfy `SPEED_LIMIT_PX < SPH_H_PX × SIM_HZ`. |
| `MAX_NEIGHBOURS` | 28 | Worst-case cap. When it overflows the list keeps the *nearest* neighbours — see "Why particles appeared to vanish". |
| `BOX_D_PX` | 96 | Box depth. Deeper = more 3D, more spread out. |
| `PARTICLE_RADIUS_PX` | 6 | Purely visual. Costs almost nothing — rendering is DMA-bound, not fill-bound. |
| `MAX_CORRECTION` | 0.15 | Cap on positional correction per iteration, in H. Lower = calmer, too low = squishy. |
| `COLOUR_SMOOTHING` | 0.18 | Temporal smoothing of the speed that drives hue. 1.0 = instantaneous, and strobes. |
| `COLOUR_HOT_SPEED` | 520 | Speed (px/s) that reaches full orange-red. |
| `TOUCH_SOLID_PX` | 62 | Radius of the solid column the fluid cannot enter. This is the knob that controls how big a hole your finger makes. |
| `TOUCH_RADIUS_PX` | 165 | Reach of the softer surrounding force field. |
| `TOUCH_STRENGTH` | 26000 | Push acceleration at the centre of the touch, px/s². |
| `TOUCH_PUSH_BACK` | 0.45 | Share of the push directed away from the glass. |
| `XSPH_VISCOSITY` | — | Higher = thicker, more honey-like. |
| `WALL_RESTITUTION` | — | Bounciness of the walls. |
| `PERSPECTIVE_F` | 300 | Lower = wider angle, more dramatic depth. |

Both boards print the truth every 3 seconds, over USB serial either way
(`idf.py monitor` on the ESP32-S3, any terminal on `/dev/cu.usbmodem*` on the RP2350):

```
ESP32-S3   sim 30.3 Hz (30.3 ms)  render 52.7 fps (18.4 ms)  free 182459 B
             phases ms: ext 0.5 grid 0.4 neigh 4.7 solve 21.3 final 2.6

RP2350     sim 30.0 Hz (30.7 ms)  render 52.7 fps (19.0 ms)  free 316520 B
             phases ms: ext 0.3 grid 0.6 neigh 6.9 solve 19.3 final 3.5
```

`solve` should dominate; if `neigh` starts to rival it, your spacing is too small
relative to H.

Comparing the two columns is instructive. The RP2350 is 17 % slower on clock yet
competitive in `solve`, because `solve` is wall-to-wall `sqrt` and reciprocal work and
the Cortex-M33 does both in hardware. It is *slower* on `neigh` and `final`, which are
integer and memory bound — that is partly the 200 MHz showing and partly DMA contending
for SRAM while a band is streaming out.

Note that `PARTICLE_COUNT` is shared, and deliberately so: the RP2350 has headroom for
more, but keeping both boards on the same number makes the comparison meaningful. Raise
it in `src/config.h` if you only care about one board.

### If something looks wrong

| symptom | fix |
|---|---|
| Image shifted sideways | On the ESP32-S3 the V1/V2 revision offset is auto-detected in `display.c` (`esp_lcd_panel_set_gap`). On the RP2350 set `LCD_X_GAP` in `board_config.h` — the 1.75" panel needs 6, the 1.8" needs 0. |
| RP2350 display completely dead, no errors | `PWR_EN` (GPIO 17) was not driven high before panel init. |
| RP2350 shows plausible but wrong colours | Byte lane or byte swap. Re-run the slow-clock PIO probe described in 2.5b rather than guessing. |
| Bottom few pixel rows of each band missing | The data state machine was disabled mid-shift; `finish_transfer()` must drain it before the next window write. |
| Fluid falls the wrong way when tilted | Flip `IMU_SIGN_X` / `IMU_SIGN_Y` in that board's `board_config.h`. |
| Fluid falls along the wrong axis | Toggle `IMU_SWAP_XY` in `board_config.h`. |
| Touch push too weak or too strong | Change `TOUCH_SOLID_PX` first — the solid column does most of the work, not `TOUCH_STRENGTH`. |
| Touches do nothing | Check the log for `touch: chip id`. On the ESP32-S3 a missing controller means the IO expander reset release failed (`touch_release_reset()`); on the RP2350 it means the GPIO 5 reset pulse did not land. |
| Fast particles skitter instead of flowing | Lower `MAX_CORRECTION`, or raise `XSPH_VISCOSITY`. |
| Colour flickers rather than flowing | Lower `COLOUR_SMOOTHING`. |
| Fluid drifts on its own | The board was moving during the boot gyro calibration. Power-cycle and hold it still for the first second. |
| Sim rate below 30 Hz | Lower `PARTICLE_COUNT`, or raise `SPH_SPACING`. |
| Particles seem to disappear after violent shaking | Fixed. See below — the neighbour list must keep the nearest neighbours, not the first ones found. |

### Why particles appeared to vanish

After heavy shaking the fluid looked like it had lost most of its particles, and
the simulation stayed permanently slower (28 ms/step to 35 ms/step) even once the
board was still again. Both symptoms had one cause.

Nothing was actually lost. A diagnostic run that dumped the bounding box of all
600 particles showed `z[85,85]` after shaking, against `z[11,85]` at rest: every
particle had been crushed into a single plane at the back wall. Flattened into
one layer they overlap each other on screen, so a full 600 particles read as a
handful. Packed that tightly each particle also has far more neighbours, which is
where the extra milliseconds went.

The cause was in `build_neighbours()`. The scan walked the 3x3x3 block of grid
cells and stopped the moment it had collected `MAX_NEIGHBOURS` entries. Cells are
visited in grid order — z-slab by z-slab starting from the lowest — so the 28
neighbours it kept were the ones with the lowest z, *not* the nearest ones. At
rest density a particle has about 25 neighbours within H, so the cap is almost
never reached and this never showed. Under compression it is reached constantly,
and then the density estimate is both truncated and biased low. The solver
concluded the fluid was at rest density when it was really three times
over-compressed, so it stopped pushing back. That made the collapse permanent:
once packed, the solver was blind to the packing and could never undo it.

The fix is to never abort the scan early, and to evict the farthest entry when
the list is full, so it always holds the nearest neighbours. Those dominate the
density kernel (poly6 falls off steeply), so truncation error becomes small and
compression stays visible to the solver. In the normal regime the list never
fills, so this costs nothing.

Raising `MAX_NEIGHBOURS` instead does *not* work — it was tried at 40 and made
things worse (90 ms/step and watchdog trips), because every solver pass then
walks a longer list.

One robustness bug surfaced alongside it, and it is FreeRTOS-specific — the RP2350
superloop resynchronises explicitly and was written with this already in mind.
`sim_task` paced itself with
`vTaskDelayUntil`, which returns *immediately* when the previous step overran its
budget. During the transient the sim task therefore never yielded, starved
`IDLE1` and tripped the task watchdog. It now detects the overrun, resynchronises
and yields one tick explicitly, so an overload degrades to a lower frame rate
instead of a watchdog reset.

Verified with a synthetic stress task driving randomised +-40 m/s^2 accelerations
and +-10 rad/s rotations on every axis, re-rolled every 40 ms, for 12 seconds —
far past anything a hand can produce. Afterwards: 600/600 particles, no NaNs,
none out of bounds, `z[11,85]`, and the step time back to 30.3 Hz.

---

## Part 4 — What porting to a second chip actually taught me

The port took a fraction of the time the original build did, and the reasons are worth
writing down.

**The abstraction boundary was chosen by looking at the code, not by guessing.** Before
writing a line of `hal.h` I grepped `fluid.c` and `render.c` for every platform symbol
they touched. The answer was: a clock, a log, a random source, three allocators and one
mutex. That is the whole HAL — about 75 lines. Had I designed it up front I would have
built something far larger and mostly wrong.

**`hal_err_t` is `typedef int` with `HAL_OK == 0`.** That is binary-compatible with
`esp_err_t`/`ESP_OK`, so the existing ESP32 drivers only needed their *signatures*
changed, not their bodies. A richer error enum would have meant touching every return
statement in the project for no benefit.

**Resist abstracting the scheduler.** This is the decision I am most confident about.
Every instinct says to unify `xTaskCreatePinnedToCore` and `multicore_launch_core1`
behind one API, and it would have been a mistake — the RP2350 would have paid for a
FreeRTOS imitation it does not want, and the ESP32-S3's watchdog-yield fix has no
meaning on a chip with no watchdog task. Pushing scheduling *down* into the platform and
keeping policy *up* in `app.c` left both `main.c` files short and honest.

**Verify the hardware, do not read about it.** Every genuinely surprising fact in this
project — the IO expander holding touch in reset, the IMU's 90° rotation, the button
being on the PMIC, the PIO byte lane, the TE pin on GPIO 16 — came from a probe
firmware, and most of them contradicted or were absent from the documentation.
`tools/hwprobe-rp2350/` took twenty minutes to write and found that the "user button" GPIO in
the schematic is a power-latch output that is never anything but low.

**A working log tells you nothing about the screen.** The first RP2350 build reported a
30 Hz solver, 80 fps rendering and every driver initialised, and displayed nothing
whatsoever. Everything the firmware could measure about itself was healthy, because the
one thing that was broken — bytes leaving the chip — is on the far side of every
instrument the firmware has. The way out was to find a signal that *comes back*: the
panel's TE pulse, which only exists if the panel accepted its initialisation. When a
subsystem is write-only, look hard for anything at all it feeds back, because that is
the only thing standing between you and guessing.

**A sibling product is not the same product.** Three of the four blank-screen faults came
from extrapolating off Waveshare's 1.75" board, which shares an architecture with this
one but not its panel timings, its colour-format byte or its pin numbering. Fetching the
board-specific demo pack took two minutes and would have prevented all three.

**The result.** ~1,600 lines of portable simulation and rendering shared by both boards,
against ~820 lines of ESP32-S3 drivers and ~920 of RP2350 drivers. Adding a third board would mean writing one
`board_config.h` and six small files, with no changes at all above the driver line.
