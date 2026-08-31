# Repository Guidelines

## Project Structure & Module Organization

This repository contains an STM32F401RE quadcopter controller and its Altium PCB design. Firmware is under `Project/`:

- `APP/` contains `main.c` and configuration headers that select enabled modules.
- `APP_FUNC/` contains application drivers and control logic (IMU, PID, PWM, GPIO, I²C, UART, and LEDs).
- `BSP_SELF/` contains board-specific startup, clock, interrupt, and BSP code; `BSP_SYS/` and `STLIB/` provide CMSIS and STM32 Standard Peripheral Library sources.
- `UCOS_PORT/` and `UCOS_SOURCE/` contain the µC/OS-II port and kernel.
- `CLIB/` holds the project’s C library headers.
- `PCB/` contains Altium schematics, PCB layouts, libraries, and design-rule reports; `docs/` contains wiring, voltage, and sensor notes.

There is no test tree. Add host-side tests in a top-level `tests/` directory when introducing testable logic, and keep generated binaries out of source directories.

## Build, Test, and Development Commands

The primary workflow uses Keil µVision 5 on Windows:

1. Open `Project/exp4.uvprojx`.
2. Select the desired target and choose **Build**.
3. Use **Download** to flash a connected STM32F401RE board.

The README mentions a Linux `make`/`make burn` flow, but no Makefile is checked in. Validate firmware on hardware after building; verify sensors, motor direction, failsafe behavior, and PID stability with propellers removed.

## Coding Style & Naming Conventions

Use C with four-space indentation, same-line braces, and one statement per line. Match existing naming: uppercase subsystem files (`IMU.c`), lowercase drivers (`gpio.c`, `pwm.c`), and paired `.c`/`.h` files. Use uppercase header guards (for example, `#ifndef PID_H`). Include `includes.h` for shared dependencies and enable modules through `Project/APP/app_cfg.h`. Keep interrupt handlers short and non-blocking.

## Testing Guidelines

No automated framework or coverage threshold is configured. For control or hardware changes, build with warnings enabled and smoke-test startup, UART output, IMU calibration, PWM limits, and µC/OS-II scheduling. Record board, compiler, and test conditions in the pull request.

## Commit & Pull Request Guidelines

Write concise, imperative commit subjects (for example, `Fix IMU calibration timeout`) and keep unrelated changes separate. Pull requests should explain the hardware/firmware impact, identify modified modules, include build and bench-test results, and link relevant issues. Include schematics, oscilloscope captures, or flight-test data when electrical or tuning behavior changes. Do not commit private keys, personal configuration, or large generated build artifacts.

## Safety & Configuration Notes

Review pin assignments in `docs/引脚配置.txt` and voltage limits in `docs/电压分配.txt` before wiring. Treat motor and LiPo-battery tests as hazardous: remove propellers for debugging and use an appropriate test area and failsafe configuration.
