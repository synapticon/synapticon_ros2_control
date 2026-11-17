// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Test fixtures that drive any Cia402Hal at BENCH_RATE_HZ, and the adapter a HAL provides for them.
///
/// For test executables only (it needs gtest). The drive action suite in cia402_hal_tests.hpp is
/// written against these fixtures, so the same tests run on simulated drives and on real ones.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/cia402/cia402_hal.hpp"
#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware::testing
{

using Clock = std::chrono::steady_clock;

/// The bench's cycle rate [Hz]: the default control_frequency of the HALs. A HAL under test is configured with it.
constexpr double BENCH_RATE_HZ = 500.0;
constexpr double DT = 1.0 / BENCH_RATE_HZ;

/// The number of bench cycles in `seconds`, at least one.
inline int cycles(double seconds)
{
  return std::max(1, static_cast<int>(std::lround(seconds * BENCH_RATE_HZ)));
}
constexpr double QUIET_NAN = std::numeric_limits<double>::quiet_NaN();
constexpr int CSP_MODE = 8;
constexpr int CSV_MODE = 9;
constexpr int IMPEDANCE_MODE = -6;

/// Set by SIGINT and SIGTERM once install_stop_handlers() ran. The running test then fails fatally, which cuts the
/// voltage, and every later test is skipped.
inline std::atomic<bool> stop_requested{ false };
static_assert(std::atomic<bool>::is_always_lock_free, "stop_requested is set in a signal handler");

/// The signal handler of install_stop_handlers().
inline void request_stop(int /*signal*/)
{
  stop_requested = true;
}

/// Route SIGINT and SIGTERM to stop_requested. Initialise rclcpp without its signal handlers, so it keeps out.
inline void install_stop_handlers()
{
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
}

/// Fail the running test, fatally, once a stop was requested. A bench cycle then cuts the voltage, as after a
/// watchdog trip.
inline void fail_on_stop_request()
{
  if (stop_requested && !::testing::Test::HasFatalFailure())
  {
    FAIL() << "STOPPED: SIGINT or SIGTERM came in, voltage cut";
  }
}

/// Skip the current test once a stop was requested. For a fixture's SetUp().
#define SKIP_AFTER_STOP_REQUEST()                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    if (synapticon_hardware::testing::stop_requested)                                                                  \
    {                                                                                                                  \
      GTEST_SKIP() << "a stop was requested by SIGINT or SIGTERM";                                                     \
    }                                                                                                                  \
  } while (0)

/// The value of environment variable `name`, or `fallback` when it is unset or empty.
inline std::string env_or(const char* name, const std::string& fallback)
{
  const char* value = std::getenv(name);
  return (value != nullptr && *value != '\0') ? value : fallback;
}

/// Stiffness and damping the impedance tests move a joint with: stiff enough that the joint tracks a slow motion
/// instead of sticking on friction, soft enough that the drive stays quiet.
struct TestGains
{
  double stiffness{ 15.0 };  ///< Nm/rad
  double damping{ 0.5 };     ///< Nm s/rad
};

/// Overwrite `gains` with the `stiffness` and `damping` that `node` gives.
inline void read_test_gains(const YAML::Node& node, TestGains& gains)
{
  if (node && node["stiffness"])
  {
    gains.stiffness = node["stiffness"].as<double>();
  }
  if (node && node["damping"])
  {
    gains.damping = node["damping"].as<double>();
  }
}

/// @brief The test gains of joint `joint` (its index in the robot definition) from the `hil_tests` section of the
/// robot definition `robot`: its entry under `joints`, else the section's own values.
///
/// The defaults suit a geared joint. A joint without a gear has far less inertia and needs much smaller values.
/// @throws std::runtime_error when a value cannot be read, or `joints` names a joint the robot does not have.
inline TestGains test_gains(const YAML::Node& robot, std::size_t joint)
{
  TestGains gains;
  const YAML::Node section = robot["hil_tests"];
  read_test_gains(section, gains);
  if (!section || !section["joints"])
  {
    return gains;
  }
  const YAML::Node joints = robot["joints"];
  for (const auto& entry : section["joints"])
  {
    const auto name = entry.first.as<std::string>();
    const bool known = std::any_of(joints.begin(), joints.end(),
                                   [&](const YAML::Node& j) { return j["name"].as<std::string>() == name; });
    if (!known)
    {
      throw std::runtime_error("hil_tests names joint '" + name + "', which the robot does not have");
    }
  }
  if (joint < joints.size())
  {
    read_test_gains(section["joints"][joints[joint]["name"].as<std::string>()], gains);
  }
  return gains;
}

/// @brief The test gains of joint `joint` of the robot definition that SYNAPTICON_HIL_ROBOT names, the defaults
/// without one.
/// @throws std::runtime_error when the file or a value cannot be read.
inline TestGains test_gains(std::size_t joint)
{
  const char* path = std::getenv("SYNAPTICON_HIL_ROBOT");
  if (path == nullptr || *path == '\0')
  {
    return {};
  }
  try
  {
    return test_gains(YAML::LoadFile(path), joint);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(std::string("hil_tests of '") + path + "': " + e.what());
  }
}

/// True when `gains` differ from the default test gains.
inline bool own_test_gains(const TestGains& gains)
{
  const TestGains defaults;
  return gains.stiffness != defaults.stiffness || gains.damping != defaults.damping;
}

/// Strict mode (SYNAPTICON_HIL_STRICT=1): a test that cannot run on this bench fails instead of skipping.
inline bool strict_mode()
{
  return env_or("SYNAPTICON_HIL_STRICT", "0") == "1";
}

/// Skip the current test with `message`, or fail it in strict mode.
#define SKIP_OR_FAIL(message)                                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    if (synapticon_hardware::testing::strict_mode())                                                                   \
    {                                                                                                                  \
      FAIL() << message;                                                                                               \
    }                                                                                                                  \
    GTEST_SKIP() << message;                                                                                           \
  } while (0)

/// How a drive treats a following error, for the tests that provoke one.
enum class FollowingError
{
  DEFAULT,  ///< The drive's default: reported only, with the window off.
  REPORT,   ///< A tight window (a small jump trips it within a cycle), reported only.
  FAULT,    ///< The same window, and the drive faults on it.
};

/// The drive settings a test asks for. A HAL's bench applies them in its own way, for example as
/// SDO entries of a slave_config or as simulation parameters.
struct DriveSettings
{
  int mode{ CSP_MODE };
  bool energize{ true };
  int quick_stop_option_code{ 6 };              ///< CiA 402 0x605A
  double quick_stop_deceleration{ QUIET_NAN };  ///< [rad/s^2], NaN keeps the drive's own
  FollowingError following_error{ FollowingError::DEFAULT };
  bool quick_stop_hold{ true };  ///< the HAL's impedance quick stop hold
};

/// @brief What a HAL provides to run the suite: drives under test, set up as asked.
class HalBench
{
public:
  virtual ~HalBench() = default;

  /// A started HAL with one drive per entry of `drives` (joint_1, joint_2, ...), or nullptr when
  /// the bring-up failed.
  virtual std::shared_ptr<Cia402Hal> open_drives(const std::vector<DriveSettings>& drives) = 0;
  /// Called after a HAL was stopped and torn down, before the next open_drives().
  virtual void released()
  {
  }
  /// Called once before the first test, and once after the last.
  virtual bool prepare()
  {
    return true;
  }
  virtual void finish()
  {
  }
  /// Whether the two drive tests can run.
  virtual bool two_drives() const = 0;
  /// How long drive `drive` (0 is joint_1) keeps control after it was told to cut the voltage, for example while its
  /// brake engages [s].
  virtual double disable_delay(std::size_t /*drive*/) const
  {
    return 0.0;
  }
  /// The smallest velocity step drive `drive` takes [rad/s], 0 when it takes any value.
  virtual double velocity_step(std::size_t /*drive*/) const
  {
    return 0.0;
  }
};

/// The bench of this test executable, set in main() before RUN_ALL_TESTS().
inline std::shared_ptr<HalBench>& bench_instance()
{
  static std::shared_ptr<HalBench> instance;
  return instance;
}
inline HalBench& bench()
{
  return *bench_instance();
}

/// @brief One drive under test, cycled at BENCH_RATE_HZ, with the helpers the tests share.
class SingleDrive : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ASSERT_TRUE(bench().prepare()) << "the bench could not be prepared: is the drive connected and the master free?";
  }
  static void TearDownTestSuite()
  {
    bench().finish();
  }

  void SetUp() override
  {
    SKIP_AFTER_STOP_REQUEST();
    sign_ = (tests_run_++ % 2 == 0) ? 1.0 : -1.0;
  }
  void TearDown() override
  {
    release_drives();
  }

  /// Bring the drive up with `settings` and seed the command with its actual position. False on failure.
  bool start(const DriveSettings& settings)
  {
    drive_ = bench().open_drives({ settings });
    if (!drive_)
    {
      return false;
    }
    setpoint_ = DriveCommand{};
    setpoint_.stiffness = QUIET_NAN;
    setpoint_.damping = QUIET_NAN;
    setpoint_.max_joint_torque = QUIET_NAN;
    next_tick_ = Clock::now();
    sample();
    setpoint_.position = reading_.position;
    return true;
  }

  /// De-energise and release the drive.
  void release_drives()
  {
    if (!drive_)
    {
      return;
    }
    drive_->disable();
    recover_from_fault();
    drive_->stop();
    drive_->disconnect();
    drive_.reset();
    bench().released();
  }

  /// Reset a drive left in Fault, since no stop action does, so the next test can enable it.
  void recover_from_fault()
  {
    sample();
    if (drive_state() != cia402::DeviceState::FAULT)
    {
      return;
    }
    hold_here();
    tick(cycles(0.02));
    // A reset leaves the drive without power, so it also runs after a fatal failure.
    setpoint_.drive_action = DriveAction::RESET_FAULT;
    for (int i = 0; i < cycles(1.0) && drive_state() != cia402::DeviceState::SWITCH_ON_DISABLED; ++i)
    {
      cycle();
    }
    setpoint_.drive_action = DriveAction::NONE;
  }

  void sample()
  {
    std::vector<DriveState> states;
    ASSERT_TRUE(drive_->read(states));
    reading_ = states.at(0);
    read_time_ = Clock::now();
  }

  /// One control cycle: read, check the watchdog, write the command, wait for the period.
  ///
  /// Once the test has a fatal failure, a watchdog trip or a stop request among them, every cycle commands
  /// disable_voltage and tick() returns after one cycle. The helpers that loop stop as well, so a test body that goes
  /// on moves nothing.
  void tick(int count = 1)
  {
    for (int i = 0; i < count; ++i)
    {
      cycle();
      if (HasFatalFailure())
      {
        return;
      }
    }
  }

  /// The one cycle of tick(). After a fatal failure it commands disable_voltage, unless the command is a reset_fault.
  void cycle()
  {
    sample();
    check_travel();
    fail_on_stop_request();
    if (HasFatalFailure() && setpoint_.drive_action != DriveAction::RESET_FAULT)
    {
      setpoint_.drive_action = DriveAction::DISABLE_VOLTAGE;
      setpoint_.velocity = 0.0;
    }
    const bool sent = drive_->write({ setpoint_ });
    next_tick_ += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(DT));
    std::this_thread::sleep_until(next_tick_);
    ASSERT_TRUE(sent);
  }

  /// Cycle until `done`, at most `seconds`. Returns the time it took, or a negative value, also after a fatal failure.
  double tick_until(const std::function<bool()>& done, double seconds)
  {
    const int max = static_cast<int>(seconds / DT);
    for (int i = 0; i < max; ++i)
    {
      if (HasFatalFailure())
      {
        return -1.0;
      }
      if (done())
      {
        return i * DT;
      }
      tick();
    }
    return !HasFatalFailure() && done() ? seconds : -1.0;
  }

  /// A drive_action request: the action for one cycle, then NONE, with the streamed motion carried on through it.
  void act(DriveAction action)
  {
    setpoint_.drive_action = action;
    setpoint_.position += setpoint_.velocity * DT;
    tick();
    setpoint_.drive_action = DriveAction::NONE;
  }

  /// Command the actual position, so no stale setpoint applies on the next enable.
  void hold_here()
  {
    setpoint_.position = reading_.position;
    setpoint_.velocity = 0.0;
  }

  /// Request ENABLE and wait for Operation Enabled.
  void energize()
  {
    hold_here();
    act(DriveAction::ENABLE);
    ASSERT_GE(tick_until([this] { return drive_state() == cia402::DeviceState::OPERATION_ENABLED; }, 2.0), 0.0)
        << "drive did not reach Operation Enabled";
    tick(cycles(0.05));
  }

  /// Accelerate over 0.2 s, then stream a constant velocity for `seconds`, alternating direction between tests.
  void run_at(double speed, double seconds)
  {
    const double v = speed * sign_;
    const int ramp = cycles(0.2);
    for (int i = 0; i < ramp && !HasFatalFailure(); ++i)
    {
      setpoint_.velocity = v * (i + 1) / ramp;
      setpoint_.position += setpoint_.velocity * DT;
      tick();
    }
    const int steps = static_cast<int>(seconds / DT);
    for (int i = 0; i < steps && !HasFatalFailure(); ++i)
    {
      setpoint_.position += v * DT;
      tick();
    }
  }

  /// Keep streaming the motion run_at() started, for `seconds`.
  void carry_on(double seconds)
  {
    const int steps = static_cast<int>(seconds / DT);
    for (int i = 0; i < steps && !HasFatalFailure(); ++i)
    {
      setpoint_.position += setpoint_.velocity * DT;
      tick();
    }
  }

  /// Arm the watchdog: from now on the joint may travel at most `max_travel` rad. A trip stays in force.
  void limit_travel(double max_travel)
  {
    travel_origin_ = reading_.position;
    travel_limit_ = max_travel;
  }

  /// Fail the test, fatally, once the joint has travelled more than `travel_limit_` rad. tick() then cuts the voltage
  /// in this cycle and in every later one.
  void check_travel()
  {
    if (std::isnan(travel_limit_) || limit_hit_)
    {
      return;
    }
    if (std::abs(reading_.position - travel_origin_) > travel_limit_)
    {
      limit_hit_ = true;
      FAIL() << "WATCHDOG: travelled more than " << travel_limit_ << " rad (from " << travel_origin_ << " to "
             << reading_.position << ", statusword 0x" << std::hex << reading_.status_word << std::dec
             << "), voltage cut";
    }
  }

  cia402::DeviceState drive_state() const
  {
    return cia402::device_state(reading_.status_word);
  }
  double joint_speed() const
  {
    return std::abs(reading_.velocity);
  }

  /// One control cycle's reading. `t` is when it came in, in seconds since the recording started, measured by the
  /// clock: after a late cycle the bench catches up with cycles in quick succession, so cycle counts would misplace them.
  struct Snapshot
  {
    double t;
    double speed;
    double position;
    cia402::DeviceState state;
  };
  using Trace = std::vector<Snapshot>;

  /// Cycle for up to `seconds`, recording every reading, until `done` holds.
  Trace trace(double seconds, const std::function<bool(const Trace&)>& done)
  {
    Trace samples;
    const auto started = Clock::now();
    const int max = static_cast<int>(seconds / DT);
    for (int i = 0; i < max && !HasFatalFailure(); ++i)
    {
      tick();
      const double t = std::chrono::duration<double>(read_time_ - started).count();
      samples.push_back({ t, joint_speed(), reading_.position, drive_state() });
      if (done(samples))
      {
        break;
      }
    }
    return samples;
  }

  /// Whether every sample of the `window` seconds up to index `end` was slower than `limit`.
  static bool stayed_below(const Trace& samples, double window, double limit, std::size_t end)
  {
    if (end >= samples.size() || samples[end].t < window)
    {
      return false;
    }
    for (std::size_t i = end + 1; i-- > 0 && samples[end].t - samples[i].t < window;)
    {
      if (samples[i].speed >= limit)
      {
        return false;
      }
    }
    return true;
  }

  static bool at_rest(const Trace& samples)
  {
    return stayed_below(samples, 0.05, 0.05, samples.size() - 1);
  }

  /// Quick stop deceleration [rad/s^2]: least squares slope of the speed over the samples on the ramp, NaN without one.
  static double fitted_deceleration(const Trace& samples)
  {
    double n = 0.0, st = 0.0, sv = 0.0, stt = 0.0, stv = 0.0;
    double first = QUIET_NAN, last = QUIET_NAN;
    for (const auto& sample : samples)
    {
      if (sample.t < 0.05 || sample.speed < 0.15)
      {
        continue;
      }
      first = std::isnan(first) ? sample.t : first;
      last = sample.t;
      n += 1.0;
      st += sample.t;
      sv += sample.speed;
      stt += sample.t * sample.t;
      stv += sample.t * sample.speed;
    }
    if (n < 3.0 || last - first < 0.05)  // at least 50 ms of the ramp
    {
      return QUIET_NAN;
    }
    return -(n * stv - st * sv) / (n * stt - st * st);
  }

  /// Quick stop with the target held at the request point: a joint that ramped past it must not return to it.
  void expect_no_return_after_quick_stop()
  {
    // Only meaningful when the ramp carries the joint past the held target, so retry until it does.
    ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
    bool tested = false;
    for (int attempt = 0; attempt < 3 && !tested; ++attempt)
    {
      ASSERT_NO_FATAL_FAILURE(energize());
      ASSERT_NO_FATAL_FAILURE(run_at(0.8, 0.3));
      limit_travel(1.2);
      ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
      const double held_target = setpoint_.position;
      setpoint_.velocity = 0.0;  // the controller holds its target at the request point
      const Trace samples = trace(3.0, at_rest);
      ASSERT_FALSE(HasFatalFailure());
      ASSERT_TRUE(at_rest(samples)) << "did not stop";
      const double p_stopped = reading_.position;
      if (std::abs(p_stopped - held_target) < 0.05)
      {
        continue;
      }  // no ramp: nothing to snap back
      ASSERT_NO_FATAL_FAILURE(tick(cycles(2.0)));
      EXPECT_NEAR(reading_.position, p_stopped, 0.03) << "snapped back toward the held target";
      tested = true;
    }
    EXPECT_TRUE(tested) << "the drive stopped without a ramp in three attempts";
  }

  /// Arm following error detection, enable, and step the target by `step` rad until the drive reports a fault.
  void provoke_step_fault(double deceleration, double step)
  {
    ASSERT_TRUE(start({ .quick_stop_deceleration = deceleration, .following_error = FollowingError::FAULT }));
    ASSERT_NO_FATAL_FAILURE(energize());
    limit_travel(1.2);  // wide enough for the drive's reaction and for its turn after a Fault Reset
    setpoint_.position += step * sign_;
    ASSERT_GE(tick_until(
                  [this] {
                    return drive_state() == cia402::DeviceState::FAULT_REACTION_ACTIVE ||
                           drive_state() == cia402::DeviceState::FAULT;
                  },
                  0.5),
              0.0)
        << "the step did not fault the drive";
  }

  std::shared_ptr<Cia402Hal> drive_;
  DriveCommand setpoint_;
  DriveState reading_;
  Clock::time_point read_time_;  ///< when reading_ came in
  Clock::time_point next_tick_;
  double travel_origin_{ 0.0 };
  double travel_limit_{ QUIET_NAN };
  bool limit_hit_{ false };
  double sign_{ 1.0 };

  static inline int tests_run_{ 0 };
};

/// @brief Two drives under test, `joint_1` and `joint_2`, cycled at BENCH_RATE_HZ.
///
/// Every cycle advances each joint's target by its commanded velocity. The watchdog cuts both
/// drives if either strays more than `max_lag_` from its target. Skipped when the bench has no
/// second drive.
class DrivePair : public ::testing::Test
{
protected:
  static constexpr std::size_t JOINTS = 2;

  static void SetUpTestSuite()
  {
    if (bench().two_drives())
    {
      ASSERT_TRUE(bench().prepare())
          << "the bench could not be prepared: are both drives connected and the master free?";
    }
  }

  void SetUp() override
  {
    SKIP_AFTER_STOP_REQUEST();
    if (!bench().two_drives())
    {
      GTEST_SKIP() << "needs a second drive on the bus: run with --two-drives";
    }
  }
  void TearDown() override
  {
    release_drives();
  }

  /// Bring both drives up, each with its own settings, and seed each command with the actual position.
  bool start(const DriveSettings& joint_1 = {}, const DriveSettings& joint_2 = {})
  {
    drive_ = bench().open_drives({ joint_1, joint_2 });
    if (!drive_)
    {
      return false;
    }
    setpoints_.assign(JOINTS, DriveCommand{});
    for (auto& command : setpoints_)
    {
      command.stiffness = QUIET_NAN;
      command.damping = QUIET_NAN;
      command.max_joint_torque = QUIET_NAN;
    }
    next_tick_ = Clock::now();
    sample();
    for (std::size_t j = 0; j < JOINTS; ++j)
    {
      setpoints_[j].position = readings_[j].position;
    }
    return true;
  }

  /// De-energise and release both drives.
  void release_drives()
  {
    if (!drive_)
    {
      return;
    }
    drive_->disable();
    recover_from_faults();
    drive_->stop();
    drive_->disconnect();
    drive_.reset();
    bench().released();
  }

  /// Reset any drive left in Fault, explicitly, since no stop action does, before releasing the bus.
  void recover_from_faults()
  {
    sample();
    std::vector<std::size_t> faulted;
    for (std::size_t j = 0; j < JOINTS; ++j)
    {
      if (drive_state(j) == cia402::DeviceState::FAULT)
      {
        faulted.push_back(j);
      }
    }
    if (faulted.empty())
    {
      return;
    }
    hold_here();
    tick(cycles(0.02));
    // A reset leaves the drives without power, so it also runs after a fatal failure.
    for (const std::size_t j : faulted)
    {
      setpoints_[j].drive_action = DriveAction::RESET_FAULT;
    }
    const auto all_reset = [this, &faulted] {
      return std::all_of(faulted.begin(), faulted.end(),
                         [this](std::size_t j) { return drive_state(j) == cia402::DeviceState::SWITCH_ON_DISABLED; });
    };
    for (int i = 0; i < cycles(1.0) && !all_reset(); ++i)
    {
      cycle();
    }
    for (const std::size_t j : faulted)
    {
      setpoints_[j].drive_action = DriveAction::NONE;
    }
  }

  void sample()
  {
    ASSERT_TRUE(drive_->read(readings_));
    ASSERT_EQ(readings_.size(), JOINTS);
  }

  /// One cycle for both joints: advance the targets, read, check the watchdog, write, wait.
  ///
  /// Once the test has a fatal failure, a watchdog trip or a stop request among them, every cycle commands
  /// disable_voltage on both joints and tick() returns after one cycle, like SingleDrive::tick().
  void tick(int count = 1)
  {
    for (int i = 0; i < count; ++i)
    {
      cycle();
      if (HasFatalFailure())
      {
        return;
      }
    }
  }

  /// The one cycle of tick(). After a fatal failure it commands disable_voltage on every joint that is not asked for
  /// a reset_fault.
  void cycle()
  {
    for (auto& command : setpoints_)
    {
      command.position += command.velocity * DT;
    }
    sample();
    check_travel();
    fail_on_stop_request();
    for (auto& command : setpoints_)
    {
      if (HasFatalFailure() && command.drive_action != DriveAction::RESET_FAULT)
      {
        command.drive_action = DriveAction::DISABLE_VOLTAGE;
        command.velocity = 0.0;
      }
    }
    const bool sent = drive_->write(setpoints_);
    next_tick_ += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(DT));
    std::this_thread::sleep_until(next_tick_);
    ASSERT_TRUE(sent);
  }

  /// Cycle until `done`, at most `seconds`. Returns the time it took, or a negative value, also after a fatal failure.
  double tick_until(const std::function<bool()>& done, double seconds)
  {
    const int max = static_cast<int>(seconds / DT);
    for (int i = 0; i < max; ++i)
    {
      if (HasFatalFailure())
      {
        return -1.0;
      }
      if (done())
      {
        return i * DT;
      }
      tick();
    }
    return !HasFatalFailure() && done() ? seconds : -1.0;
  }

  /// A drive_action request for the joints in `joints`, all in the same cycle.
  void act(const std::vector<std::size_t>& joints, DriveAction action)
  {
    for (const std::size_t j : joints)
    {
      setpoints_[j].drive_action = action;
    }
    tick();
    for (const std::size_t j : joints)
    {
      setpoints_[j].drive_action = DriveAction::NONE;
    }
  }

  /// Command each joint's actual position and stop its motion.
  void hold_here()
  {
    for (std::size_t j = 0; j < JOINTS; ++j)
    {
      setpoints_[j].position = readings_[j].position;
      setpoints_[j].velocity = 0.0;
    }
  }

  /// Enable both drives and wait for Operation Enabled.
  void energize_both()
  {
    hold_here();
    act({ 0, 1 }, DriveAction::ENABLE);
    ASSERT_GE(tick_until(
                  [this] {
                    return drive_state(0) == cia402::DeviceState::OPERATION_ENABLED &&
                           drive_state(1) == cia402::DeviceState::OPERATION_ENABLED;
                  },
                  2.0),
              0.0)
        << "the drives did not reach Operation Enabled";
    tick(cycles(0.05));
  }

  /// Start `joint` moving at `speed`, ramping up over 0.2 s.
  void set_off(std::size_t joint, double speed)
  {
    const int ramp = cycles(0.2);
    for (int i = 0; i < ramp && !HasFatalFailure(); ++i)
    {
      setpoints_[joint].velocity = speed * (i + 1) / ramp;
      tick();
    }
  }

  /// Fail the test, fatally, once a joint is more than `max_lag_` rad off its target. tick() then cuts the voltage of
  /// both drives in this cycle and in every later one.
  void check_travel()
  {
    if (limit_hit_ || readings_.size() != JOINTS)
    {
      return;
    }
    for (std::size_t j = 0; j < JOINTS; ++j)
    {
      if (std::abs(readings_[j].position - setpoints_[j].position) > max_lag_)
      {
        limit_hit_ = true;
        FAIL() << "WATCHDOG: joint_" << j + 1 << " is " << readings_[j].position - setpoints_[j].position
               << " rad off its target, voltage cut";
      }
    }
  }

  cia402::DeviceState drive_state(std::size_t j) const
  {
    return cia402::device_state(readings_[j].status_word);
  }

  std::shared_ptr<Cia402Hal> drive_;
  std::vector<DriveCommand> setpoints_;
  std::vector<DriveState> readings_;
  Clock::time_point next_tick_;
  double max_lag_{ 0.5 };
  bool limit_hit_{ false };
};

}  // namespace synapticon_hardware::testing
