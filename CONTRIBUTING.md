# Contributing

Thank you for helping out. This guide covers how to build and test a change, the conventions for commits and
pull requests, and the style for code and documentation.
For how the packages fit together, read [doc/Design.md](synapticon_ros2_control/doc/Design.md) first.

## Build

Build in a colcon workspace, never from inside the repository.
The repository ships two dependency manifests:

| Manifest | Contains | Needed for |
|---|---|---|
| `synapticon_ros2_control.repos` | `synapticon_control_msgs` | every build |
| `ethercat_icube.repos` | `ethercat_driver_ros2`, the Synapticon fork, pinned to one commit | the EtherCAT ICube HAL only |

A machine without the IgH EtherLab master builds without the EtherCAT ICube HAL.
The SOEM HAL and the mocks need neither the master nor the driver stack:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/synapticon/synapticon_ros2_control.git
cd ~/ws
vcs import src < src/synapticon_ros2_control/synapticon_ros2_control.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`synapticon_ethercat_icube` then builds nothing.

With the IgH master installed, set `SYNAPTICON_HAL_ETHERCAT_ICUBE=ON` and import the driver stack as well.
The variable must be set in the shell that runs rosdep and colcon.
It switches both the conditional dependencies of `synapticon_ethercat_icube` and its CMake option of the same name.

```bash
export SYNAPTICON_HAL_ETHERCAT_ICUBE=ON
vcs import src < src/synapticon_ros2_control/ethercat_icube.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

CMake keeps the value of the first build, and colcon does not reconfigure a package when only the variable changes.
`--cmake-clean-cache` makes CMake read the variable again. It is needed once, in the first build after a switch.

### Always build optimised

colcon sets no build type. Without one, the compiler gets no optimisation flag at all, which slows down the
real-time loop considerably. The packages default to `Release` when the build type is empty, but pass it
explicitly anyway, or `RelWithDebInfo` when you need symbols.

Check the flags before you measure anything:

```bash
grep CXX_FLAGS build/synapticon_hardware/CMakeFiles/synapticon_hardware.dir/flags.make
grep CXX_FLAGS build/synapticon_ethercat_soem/CMakeFiles/synapticon_ethercat_soem.dir/flags.make
grep CXX_FLAGS build/synapticon_ethercat_icube/CMakeFiles/synapticon_ethercat_icube.dir/flags.make
```

Expected for `Release`:

```text
CXX_FLAGS = -O3 -DNDEBUG -std=gnu++20 -fPIC -Wall -Wextra -Wpedantic
```

Changing the build type of an existing build directory may need a clean rebuild of that package.

## Test

Unit tests and the launch tests on the mock HAL need no hardware:

```bash
colcon test --packages-select synapticon_hardware synapticon_ethercat_icube synapticon_ethercat_soem synapticon_controllers synapticon_bringup synapticon_description
colcon test-result --verbose
```

Expected: `0 errors, 0 failures`.

The hardware-in-the-loop suite, the ros2_control path test, the drive check and the manual tests need a drive.
[doc/Testing.md](synapticon_ros2_control/doc/Testing.md) walks through them step by step.
The suite and the launch hardware tests also run on simulated drives (`mock_cia402`) in `colcon test`.

Shared test code, to build on rather than copy:

| Where | What |
|---|---|
| `synapticon_hardware/include/synapticon_hardware/testing/` | The drive action suite and its fixtures, for any HAL that derives from `Cia402Hal` |
| `synapticon_hardware/test/description_builders.hpp` | ros2_control descriptions: interfaces, joints, transmissions, the bus gpio |
| `synapticon_controllers/test/controller_test_utils.hpp` | Bringing a controller up and down over handles the test owns |
| `synapticon_bringup/synapticon_bringup/testing.py` | The example launch for launch tests, and a probe node that follows the running stack |

A test that holds for every HAL goes into the drive action suite. A test that only makes sense for one HAL
goes into that HAL's package.

## Formatting

pre-commit checks the formatting:

- clang-format for C++, with the repository's `.clang-format` (Google based, 120 columns).
- black for Python.
- cmake-lint for CMake files.
- The standard hooks: trailing whitespace, end of file, YAML and JSON syntax, merge conflicts, large files.

The hooks leave out the vendored SOEM sources and license files of `soem_source`, which stay as they came.

```bash
pip install pre-commit
pre-commit install        # runs the hooks on every commit
pre-commit run -a         # or run them on all files by hand
```

pre-commit only checks files that git knows about. Add new files before you run it.

## Commits and pull requests

- One change per commit, with its tests in the same commit. A rename, a refactor and a behaviour change are
  three commits.
- The build and the tests pass at every commit.
- A change to code that talks to a drive (the EtherCAT HAL, `SynapticonSystemInterface`, the controllers, the launch
  files) also runs the automated hardware tests (`run_hil_tests.py`, see [doc/Testing.md](synapticon_ros2_control/doc/Testing.md)).
  Say in the pull request what you ran and on which drive and firmware.
- Commit messages: a short subject in the imperative ("Validate the bus parameters at setup"), then a body
  that says what changes and why.
- The pull request description gives a short overview and a test plan with copy and paste commands for
  everything a reviewer should run, including what the automated tests do not cover.
- Update the documentation in the same pull request as the behaviour it describes.

## Documentation style

Markdown is written for humans first.

- Start with the overview: what is there, how the parts fit together, where to look next.
  Leave the details to the code.
- Explain with an example or a comparison rather than with abstract text.
- Short sections, short sentences, plain words. Tables for options and comparisons, a diagram where a
  picture explains faster.
- A document should be readable in one sitting. Split by topic instead of growing one document.
- Give copy and paste commands for every example: build, launch, command a joint, and check that it worked,
  including what the expected output looks like.
- Break lines after a full stop, aiming at 100 to 120 characters.

Testing guides are test plans to run, step by step:

1. What is needed: hardware, a joint that is free to move, the build type.
2. Each step as a numbered instruction with the exact command.
3. What to expect after each step, and what a failure looks like.
4. How to leave the drive afterwards.

## Code style

The naming follows ros2_control:

| Kind | Style | Example |
|---|---|---|
| Classes and structs | CamelCase | `SynapticonSystemInterface`, `DriveState` |
| Functions and methods | snake_case | `read()`, `connect()` |
| Member variables | snake_case with a trailing underscore | `joints_`, `streaming_` |
| Constants and enum values | UPPER_SNAKE | `MINIMUM_FIRMWARE`, `DriveAction::QUICK_STOP` |

Code lines are at most 120 characters.

Comments are short and to the point:

- Every source file starts with the license header: the SPDX identifier `MIT` and the copyright lines,
  as in the existing files. New files name Synapticon GmbH and b-robotized GmbH.
  `soem_source` is the exception: it is under SOEM's license, and its files are not changed.
- Every file, class and function gets a header comment. One line where one line is enough.
- Say what a class or function is for. Do not repeat how, when the code already shows it.
- Add an inline comment only where the reason is not clear from the code, for example a hardware quirk,
  a real-time constraint or a lifecycle order that must not change.
- No history, no debugging stories, no references to issues, to other files or to the projects that use
  the package.
- Give units for physical values where the type does not show them (rad, rad/s, Nm, mNm).
- Log messages say what happened or what failed, in one line.

## Writing rules for all text

These apply to code comments, log messages, commit messages and documentation alike:

- No em dashes or en dashes. Use a comma, a colon, parentheses or a new sentence.
- No semicolons in comments or markdown.
- No characters that are hard to type, such as the ellipsis character.
- No machine specific absolute paths.
