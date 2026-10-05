# Contributing to URC Software

This document captures the house style for `urc_software` so the codebase stays
readable and consistent. See [README.md](README.md) for architecture and build
instructions.

## Development workflow

New to the project? Start with [QUICKSTART.md](QUICKSTART.md): it gets the software
running without rover hardware and shows the fast edit-build-run loop.

When contributing to the repository:

1. Create a branch for your changes instead of working directly on `main`.
2. Keep each change focused on one feature, fix, documentation update, or cleanup task.
3. Build and test your changes using the project's Docker build before submitting a pull request when possible.
4. Commit your changes with a clear Conventional Commit message.
5. Push your branch and open a pull request into `main`.
6. Make sure the GitHub Actions build passes before merging.

### Branching, step by step

```bash
git switch main
git pull                          # get everyone else's merged work
git switch -c fix/short-description
# ...edit, build, test, commit...
git push -u origin fix/short-description   # then open the pull request on GitHub
```

- **Never push to `main` directly.** Changes reach `main` only through a merged pull request.
- **Start every new piece of work from a freshly pulled `main`**, not from an old branch.
- **After your PR is merged, don't keep committing on that branch.** PRs can be
  *squash-merged* (PR #27 was), which gives `main` one new commit whose ID doesn't
  match your branch's commits. If you keep using the old branch, git sees the merged
  work as "different" and you get confusing conflicts or rejected pushes.
  Switch to `main`, `git pull`, and start a new branch.
- If `git push` is rejected with "non-fast-forward", someone else pushed to that
  branch first: run `git pull --rebase`, then push again. (If it says that about
  `main`, you were trying to push `main`; push your branch instead.)

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

The goal: someone with a little programming experience can open any file and follow
what it does, why, and how it connects to the rest of the rover. The
[C++ and ROS 2 primer](docs/CPP_ROS2_PRIMER.md) explains the language features the
comments refer to.

### What every file has

- **A file header** (`/** @file ... */` in C++, a module docstring in Python) saying what
  the file is, which computer runs it, which launch file starts it, the topics it
  subscribes to and publishes, and a "How it connects to the system" list.
  [`src/main_computer_urc/src/DriveTrainManager/main.cpp`](src/main_computer_urc/src/DriveTrainManager/main.cpp)
  is a good model.
- **A header block on every non-trivial function**, in this shape:

  ```cpp
  /**
   * One-line summary of what the function does (Doxygen uses this line as the brief)
   *
   * Parameters (inputs):
   *   name - what it is, with units
   *
   * Return value:
   *   what comes back
   *
   * Steps:
   *   1. First thing it does
   *   2. Next thing
   */
  ```

  In a class, put `Parameters`/`Return value` on the declaration in the header and
  `Steps` on the definition in the `.cpp`. Python uses the same idea with
  Google-style `Args:` / `Returns:` docstrings.

### Inline comments

- **`// Syntax: ...`** explains a C++/Python/ROS feature the first time it appears in a file
  (not every time). These are for newer members; skip them once you know the feature.
- **`// System: ...`** marks where a topic or parameter connects to another node or computer.
- Otherwise, explain *why* a line exists, or what a non-obvious value, unit, or formula means.
  A magic number gets a comment even when the logic around it is clear.
- Don't comment trivial lines (`return 0;`), and don't tag every statement with `// Step 1`.

### Style rules for comment text

- **One complete thought per line.** Never wrap a sentence across two lines; if it's too
  long, split it into two sentences or a bullet list. (`clang-format` is set to
  `ReflowComments: false` so it won't re-wrap them.)
- No trailing period on comment lines.
- Describe actions with a present participle: `// Draining pending CAN frames`, not `// Drain pending CAN frames`.
- Put supporting detail in parentheses: `(the textbook formula uses half the width)`.
- **State only what the code guarantees.** If you're inferring intent or haven't checked
  something, say so (`presumably`, `VERIFY`) instead of writing it as fact.

### Housekeeping

- Delete dead/commented-out code rather than leaving it in place — git history is
  the archive. If a block is intentionally disabled, leave a one-line note saying
  so and why, not the whole commented body.
- Keep log tags accurate (the logger name should match the module) and keep per-tick
  logs at `DEBUG` level, so a fast control loop doesn't flood the terminal.

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

### Known inconsistencies worth cleaning up

1. **Package vs. folder name.** The folder `src/cross_pkg_messages_urc/` builds a
   package named `cross_pkg_messages` (no `_urc`), unlike every other package
   where folder and package name match and carry the `_urc` suffix. Renaming the
   *package* would touch every `find_package(cross_pkg_messages)` and every
   `#include "cross_pkg_messages/msg/..."`; do it in one focused, build-verified
   change if desired.
2. **Stale include paths.** Several `target_include_directories(... PRIVATE
   src/base_station_urc/include/cs_libguarded)` lines point at a doubled path that
   does not exist (the real path is `include/cs_libguarded`). They are harmless
   (a non-existent include dir is ignored) but should be corrected.
3. **`file(GLOB)` for sources.** CMake globbing does not re-run when files are
   added/removed unless CMake reconfigures. Prefer listing sources explicitly, or
   run a clean rebuild after adding/removing files.

Already resolved: first-party headers now all use `#pragma once`, and the shared
motor code's per-tick logs use an accurately named `MotorManager` logger at `DEBUG` level.

### Not currently wired in

See the "Housekeeping notes" section of [README.md](README.md) for executables
and GUI panels that are built but not launched (`ArmCommandEncoder_node`,
`StatusLED_node`, `SoftwareDebugPanel`). Decide whether to wire them up or remove
them.
