# Contributing to URC Software

This document captures the house style for `urc_software` so the codebase stays
readable and consistent. See [README.md](README.md) for architecture and build
instructions.

## Development workflow

When contributing to the repository:

1. Create a branch for your changes instead of working directly on `main`.
2. Keep each change focused on one feature, fix, documentation update, or cleanup task.
3. Build and test your changes using the project's Docker build before submitting a pull request when possible.
4. Commit your changes with a clear Conventional Commit message.
5. Push your branch and open a pull request into `main`.
6. Make sure the GitHub Actions build passes before merging.

### Commit messages

Use Conventional Commits to keep the repository history clear and consistent.

Common commit types:

- `feat:` — new feature
- `fix:` — bug fix
- `docs:` — documentation changes
- `refactor:` — code restructuring without changing behavior
- `test:` — test changes
- `build:` — build system or dependency changes
- `ci:` — CI or GitHub Actions changes
- `chore:` — general maintenance

Examples:

```text
feat: add autonomous waypoint follower
fix: correct motor command handling
docs: document navigation package
refactor: simplify gps processing
ci: update doxygen deployment workflow
```

### Pull requests and CI

Keep pull requests focused and avoid combining unrelated changes.

Pull requests to `main` automatically run the repository's Docker build through
GitHub Actions. Check that the build succeeds before merging.

When opening a pull request, briefly explain what changed, why it was changed,
and how it was tested.

## Code formatting

Formatting is defined by [`.clang-format`](.clang-format) (C++) and
[`.editorconfig`](.editorconfig) (whitespace for all file types). The style
matches what the team already writes: **4-space indent, 100-column lines, braces
on a new line for function/class bodies and K&R for control flow.**

Apply it inside the build container (or any environment with `clang-format`
installed), from the repo root:

```bash
clang-format -i $(git ls-files '*.cpp' '*.h' '*.hpp' \
  | grep -vE 'libs/|cs_libguarded|shared_code/pid')
```

Most editors (VS Code with the C/C++ extension, CLion, etc.) will pick up
`.clang-format` and `.editorconfig` automatically and format on save.

### Do not reformat vendored code

The following are third-party and should be left exactly as upstream ships them:

- `src/shared_code/pid.{h,cpp}` — MIT-licensed PID implementation (Bradley J. Snyder).
- Everything under `libs/` — git submodules (ImGui, pigpio, sockpp, doxygen-awesome-css).
- Every `include/cs_libguarded/` tree — vendored [CopperSpice libguarded](https://github.com/copperspice/cs_libguarded).

## Comments

- Public classes and functions use **Doxygen** comments (`/** ... */` or `///`),
  since API docs are generated with Doxygen (see [`Doxyfile`](Doxyfile)).
  `CANDriver.h` is a good reference for the expected level of documentation.
- Keep comments describing *why*, not *what the next line obviously does*.
- Delete dead/commented-out code rather than leaving it in place — git history is
  the archive. If a block is intentionally disabled, leave a one-line note saying
  so and why, not the whole commented body.
- Keep log tags accurate. Several existing log calls in shared code use mislabeled
  tags (e.g. `rclcpp::get_logger("Arm")` inside code shared with the driveline);
  prefer the module's real logger.

## File & directory structure

Each ROS 2 package lives under `src/<name>/` with the conventional layout:

```
src/<package>/
├── CMakeLists.txt
├── package.xml
├── launch/          # ROS 2 launch files (+ launchScript.sh docker wrapper)
├── src/             # C++ sources, grouped by node in a subfolder each
├── include/         # public headers (cs_libguarded lives here, vendored)
├── config/          # yaml params (where applicable)
└── description/     # URDF/xacro/rviz (where applicable)
```

Code shared between packages lives in [`src/shared_code`](src/shared_code) and is
pulled into the rover-side packages via `file(GLOB ...)`.

### Known inconsistencies worth cleaning up (not yet done — need a verified build)

These were found during review but not changed, because they touch generated
headers or many build files and can't be compile-verified on a non-Linux host:

1. **Package vs. folder name.** The folder `src/cross_pkg_messages_urc/` builds a
   package named `cross_pkg_messages` (no `_urc`), unlike every other package
   where folder and package name match and carry the `_urc` suffix. Renaming the
   *package* would touch every `find_package(cross_pkg_messages)` and every
   `#include "cross_pkg_messages/msg/..."`; do it in one focused, build-verified
   change if desired.
2. **Header guards.** Most first-party headers use `#pragma once`; a few
   (`CANDriver.h`, the vendored `pid.h`) use `#ifndef` guards. Standardize the
   first-party ones on `#pragma once`.
3. **Stale include paths.** Several `target_include_directories(... PRIVATE
   src/base_station_urc/include/cs_libguarded)` lines point at a doubled path that
   does not exist (the real path is `include/cs_libguarded`). They are harmless
   (a non-existent include dir is ignored) but should be corrected.
4. **`file(GLOB)` for sources.** CMake globbing does not re-run when files are
   added/removed unless CMake reconfigures. Prefer listing sources explicitly, or
   run a clean rebuild after adding/removing files.

### Not currently wired in

See the "Housekeeping notes" section of [README.md](README.md) for executables
and GUI panels that are built but not launched (`ArmCommandEncoder_node`,
`StatusLED_node`, `SoftwareDebugPanel`). Decide whether to wire them up or remove
them.
