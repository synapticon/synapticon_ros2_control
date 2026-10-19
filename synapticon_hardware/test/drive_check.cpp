// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Basic checks of one SOMANET drive through a HAL, without ros2_control.
///
/// The HAL is loaded by its plugin name (--hal) and set up with the parameters a URDF would give it.
/// - read: bring the bus up and print the drive state. Never energises.
/// - timing: time every read() and write() call. Never energises.
/// - hold: ENERGISES. Enable, hold the present position, disable.
/// - move: ENERGISES AND MOVES. Enable, ramp the target back and forth between the start position and
///   --travel-deg away, disable. Stops when the joint leaves that range.
///
/// hold and move run in CSP, or in impedance mode with --impedance. Ctrl-C stops any check cleanly.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/synapticon_hal.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::SynapticonHalI;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double TRAVEL_MARGIN = 0.3;  // [rad] beyond the travel range before the move check gives up
constexpr double TARGET_SPEED = 1.0;   // [rad/s] of the move check's target ramp
constexpr auto PRINT_PERIOD = std::chrono::milliseconds(500);
constexpr double CYCLE_HZ = 500.0;                     // the control rate of the examples
constexpr auto TURN_PERIOD = std::chrono::seconds(2);  // between the direction changes of the move check

constexpr const char* USAGE = R"(
Usage: drive_check <read|timing|hold|move> [--config <slave_config.yaml>] [options]

  read     print the drive state twice a second, turn the joint by hand to watch it change
  timing   measure how long read() and write() take
  hold     ENERGISES: enable and hold the present position
  move     ENERGISES AND MOVES: move back and forth by --travel-deg at 1 rad/s

  --hal NAME         HAL plugin: ethercat_soem (default) or mock_cia402
  --config PATH      the drive's slave_config (default: the bundled example, which fits one drive model only)
  --network-interface IF   network interface of the drive, required by ethercat_soem
  --ring-position N  place of the drive on the ring, counted from 0 (default 0)
  --duration S       run time, 0 until Ctrl-C (default 10)
  --travel-deg D     travel of the move check (default 30)
  --max-torque NM    MaxJointTorque for the run (default: the drive's value)
  --impedance        hold and move in impedance mode instead of CSP
  --stiffness K      impedance stiffness [Nm/rad] (default: the drive's value)
  --damping D        impedance damping [Nm s/rad] (default: the drive's value)
  --rt               real-time scheduling, locked memory, no deep CPU idle states (needs root)
  --rt-priority N    like --rt, at SCHED_FIFO priority N from 1 to 99. --rt alone uses 60, the priority of the
                     example controller manager
)";

std::atomic<bool> g_stop{ false };

/// Command line settings.
struct Options
{
  std::string check;
  std::string hal{ "ethercat_soem" };
  std::string config;
  std::string network_interface;
  uint16_t ring_position{ 0 };
  double duration{ 10.0 };
  double travel_deg{ 30.0 };
  double max_torque{ NaN };
  double stiffness{ NaN };
  double damping{ NaN };
  bool impedance{ false };
  bool rt{ false };
  int rt_priority{ 60 };
};

/// Parse the command line. Prints the usage and returns nothing on --help or when it is incomplete or wrong.
std::optional<Options> parse_options(int argc, char** argv)
{
  Options o;
  const auto u16 = [](const std::string& v) { return static_cast<uint16_t>(std::stoul(v, nullptr, 0)); };
  const std::map<std::string, std::function<void(const std::string&)>> valued = {
    { "--hal", [&](const std::string& v) { o.hal = v; } },
    { "--config", [&](const std::string& v) { o.config = v; } },
    { "--network-interface", [&](const std::string& v) { o.network_interface = v; } },
    { "--ring-position", [&](const std::string& v) { o.ring_position = u16(v); } },
    { "--duration", [&](const std::string& v) { o.duration = std::stod(v); } },
    { "--travel-deg", [&](const std::string& v) { o.travel_deg = std::stod(v); } },
    { "--max-torque", [&](const std::string& v) { o.max_torque = std::stod(v); } },
    { "--stiffness", [&](const std::string& v) { o.stiffness = std::stod(v); } },
    { "--damping", [&](const std::string& v) { o.damping = std::stod(v); } },
    { "--rt-priority",
      [&](const std::string& v) {
        o.rt = true;
        o.rt_priority = std::stoi(v);
      } },
  };
  const std::map<std::string, std::function<void()>> flags = {
    { "--impedance", [&] { o.impedance = true; } },
    { "--rt", [&] { o.rt = true; } },
  };
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help")
    {
      std::cout << USAGE;
      return std::nullopt;
    }
    if (const auto it = valued.find(arg); it != valued.end() && i + 1 < argc)
    {
      it->second(argv[++i]);
    }
    else if (const auto flag = flags.find(arg); flag != flags.end())
    {
      flag->second();
    }
    else if (o.check.empty() && arg.rfind("--", 0) != 0)
    {
      o.check = arg;
    }
    else
    {
      std::cerr << USAGE;
      return std::nullopt;
    }
  }
  const std::vector<std::string> checks = { "read", "timing", "hold", "move" };
  if (std::find(checks.begin(), checks.end(), o.check) == checks.end())
  {
    std::cerr << USAGE;
    return std::nullopt;
  }
  if (o.hal == "ethercat_soem" && o.network_interface.empty())
  {
    std::cerr << "The HAL ethercat_soem needs the network interface of the drive: --network-interface <interface>\n";
    return std::nullopt;
  }
  if (o.rt_priority < 1 || o.rt_priority > 99)
  {
    std::cerr << "--rt-priority takes a SCHED_FIFO priority from 1 to 99.\n";
    return std::nullopt;
  }
  if (o.config.empty())
  {
    o.config = ament_index_cpp::get_package_share_directory("synapticon_hardware") +
               "/config/example_somanet_circulo_safe_motion.yaml";
    std::cout << "No --config: using the bundled example slave_config " << o.config << "\n";
  }
  return o;
}

/// Best effort real-time setup of this process for --rt, at SCHED_FIFO `priority`. Each step reports when it does not
/// work. The CPU idle state limit lasts while its file stays open, so the descriptor is kept until the process ends.
void request_real_time(int priority)
{
  const auto attempt = [](const char* what, bool done) {
    if (!done)
    {
      std::cerr << "--rt: " << what << " not possible: " << std::strerror(errno) << "\n";
    }
  };
  const sched_param scheduling{ .sched_priority = priority };
  attempt("real-time scheduling (SCHED_FIFO)", sched_setscheduler(0, SCHED_FIFO, &scheduling) == 0);
  attempt("locking the memory", mlockall(MCL_CURRENT | MCL_FUTURE) == 0);
  static const int latency_file = ::open("/dev/cpu_dma_latency", O_WRONLY);
  const int32_t no_wakeup_delay = 0;
  attempt("limiting the CPU idle states",
          latency_file >= 0 && ::write(latency_file, &no_wakeup_delay, sizeof(no_wakeup_delay)) ==
                                   static_cast<ssize_t>(sizeof(no_wakeup_delay)));
}

/// The HAL description of one drive from the command line: the parameters a URDF would give the HAL. Optional
/// settings are left out when not given, so that the HAL's defaults apply.
synapticon_hardware::HalConfig hal_config(const Options& o, int mode_of_operation)
{
  synapticon_hardware::HalConfig config;
  auto& hardware = config.hardware_parameters;
  hardware["control_frequency"] = std::to_string(CYCLE_HZ);
  if (!o.network_interface.empty())
  {
    hardware["network_interface"] = o.network_interface;
  }
  synapticon_hardware::JointSpec joint;
  joint.name = "drive";
  joint.parameters = { { "position", std::to_string(o.ring_position) },
                       { "mode_of_operation", std::to_string(mode_of_operation) },
                       { "slave_config", o.config } };
  if (std::isfinite(o.max_torque))
  {
    joint.parameters["max_joint_torque"] = std::to_string(o.max_torque);
  }
  config.joints.push_back(joint);
  return config;
}

/// The HAL with one drive, its bus brought up and started. Stops and releases the bus on destruction.
class Bus
{
public:
  Bus() : loader_("synapticon_hardware", "synapticon_hardware::SynapticonHalI")
  {
  }

  bool open(const Options& o, int mode_of_operation)
  {
    try
    {
      hal_ = loader_.createSharedInstance(o.hal);
    }
    catch (const pluginlib::PluginlibException& e)
    {
      std::cerr << "Cannot load the HAL '" << o.hal << "': " << e.what() << "\n";
      return false;
    }
    if (!hal_->setup(hal_config(o, mode_of_operation)))
    {
      std::cerr << "setup() failed\n";
      return false;
    }
    brought_up_ = hal_->connect();
    started_ = brought_up_ && hal_->start();
    if (!started_)
    {
      std::cerr << "The bus did not come up\n";
    }
    return started_;
  }

  ~Bus()
  {
    if (started_)
    {
      hal_->stop();
    }
    if (brought_up_)
    {
      hal_->disconnect();
    }
  }

  SynapticonHalI& hal()
  {
    return *hal_;
  }

private:
  pluginlib::ClassLoader<SynapticonHalI> loader_;  // outlives the HAL it loaded
  std::shared_ptr<SynapticonHalI> hal_;
  bool brought_up_{ false };
  bool started_{ false };
};

/// A command that keeps the drive where it is: NaN position holds, NaN gains and torque limit change nothing.
DriveCommand neutral_command()
{
  DriveCommand command;
  command.position = NaN;
  command.stiffness = NaN;
  command.damping = NaN;
  command.max_joint_torque = NaN;
  return command;
}

void print_state(const DriveState& s)
{
  std::cout << "q " << s.position << " rad | dq " << s.velocity << " rad/s | tau " << s.effort << " Nm | sw 0x"
            << std::hex << std::setw(4) << std::setfill('0') << s.status_word << " | fault 0x" << std::setw(4)
            << s.error_code << std::dec << std::setfill(' ') << "\n";
}

/// Call `cycle` once per bus period until it returns false, the duration ends or Ctrl-C.
/// Returns the number of periods the loop fell behind, or nothing when `cycle` failed.
std::optional<std::size_t> run_cycles(const Options& o, const std::function<bool()>& cycle)
{
  const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / CYCLE_HZ));
  const auto end =
      o.duration > 0.0 ?
          Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(o.duration)) :
          Clock::time_point::max();
  std::size_t behind = 0;
  for (auto next = Clock::now() + period; !g_stop && Clock::now() < end; next += period)
  {
    if (!cycle())
    {
      return std::nullopt;
    }
    if (Clock::now() < next)
    {
      std::this_thread::sleep_until(next);
    }
    else
    {
      ++behind;
    }
  }
  return behind;
}

/// True once every PRINT_PERIOD.
class PrintTimer
{
public:
  bool due()
  {
    if (Clock::now() < next_)
    {
      return false;
    }
    next_ = Clock::now() + PRINT_PERIOD;
    return true;
  }

private:
  Clock::time_point next_{ Clock::now() };
};

int check_read(SynapticonHalI& hal, const Options& o)
{
  std::vector<DriveState> states;
  const std::vector<DriveCommand> neutral(1, neutral_command());
  PrintTimer timer;
  const auto behind = run_cycles(o, [&] {
    if (!hal.read(states) || !hal.write(neutral))
    {
      std::cerr << "Exchange with the drive failed\n";
      return false;
    }
    if (timer.due())
    {
      print_state(states[0]);
    }
    return true;
  });
  return behind ? 0 : 1;
}

/// Summary of call durations in microseconds.
struct Durations
{
  double median{ 0.0 };
  double p99{ 0.0 };
  double p999{ 0.0 };
  double worst{ 0.0 };
  double mean{ 0.0 };
  double spread{ 0.0 };  ///< standard deviation
};

/// The summary of `samples`, which it sorts.
Durations summarize(std::vector<double>& samples)
{
  Durations d;
  if (samples.empty())
  {
    return d;
  }
  std::sort(samples.begin(), samples.end());
  const auto rank = [&samples](double fraction) {
    return samples[static_cast<std::size_t>(std::llround(fraction * static_cast<double>(samples.size() - 1)))];
  };
  const double count = static_cast<double>(samples.size());
  d.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / count;
  const double square_sum = std::accumulate(samples.begin(), samples.end(), 0.0,
                                            [&d](double sum, double x) { return sum + (x - d.mean) * (x - d.mean); });
  d.spread = std::sqrt(square_sum / count);
  d.median = rank(0.5);
  d.p99 = rank(0.99);
  d.p999 = rank(0.999);
  d.worst = samples.back();
  return d;
}

void print_durations(const std::string& call, std::vector<double> samples)
{
  const Durations d = summarize(samples);
  std::cout << std::fixed << std::setprecision(1) << call << ": median " << d.median << " us, 99 % below " << d.p99
            << " us, 99.9 % below " << d.p999 << " us, worst " << d.worst << " us (mean " << d.mean << " +/- "
            << d.spread << " us, " << samples.size() << " calls)\n";
}

int check_timing(SynapticonHalI& hal, const Options& o)
{
  const double seconds = o.duration > 0.0 ? o.duration : 600.0;
  std::vector<double> read_us, write_us;
  read_us.reserve(static_cast<std::size_t>(seconds * CYCLE_HZ) + 1);
  write_us.reserve(read_us.capacity());
  std::vector<DriveState> states;
  const std::vector<DriveCommand> neutral(1, neutral_command());
  std::size_t failures = 0;
  const auto micros = [](Clock::duration d) { return std::chrono::duration<double, std::micro>(d).count(); };

  const auto behind = run_cycles(o, [&] {
    const auto t0 = Clock::now();
    failures += hal.read(states) ? 0 : 1;
    const auto t1 = Clock::now();
    failures += hal.write(neutral) ? 0 : 1;
    const auto t2 = Clock::now();
    if (read_us.size() < read_us.capacity())  // never reallocate inside the loop
    {
      read_us.push_back(micros(t1 - t0));
      write_us.push_back(micros(t2 - t1));
    }
    return true;
  });
  print_durations("read", read_us);
  print_durations("write", write_us);
  std::cout << read_us.size() << " cycles, " << failures << " failed calls, " << behind.value_or(0)
            << " cycles behind schedule\n";
  return failures == 0 ? 0 : 1;
}

/// hold and move: enable, follow the target, disable. move ramps the target between the start position and the
/// travel away, and stops when the joint leaves that range.
int check_motion(SynapticonHalI& hal, const Options& o)
{
  std::vector<DriveState> states;
  if (!hal.read(states))
  {
    return 1;
  }
  const double start = states[0].position;
  const double travel = o.check == "move" ? o.travel_deg * M_PI / 180.0 : 0.0;
  const double lowest = std::min(start, start + travel) - TRAVEL_MARGIN;
  const double highest = std::max(start, start + travel) + TRAVEL_MARGIN;

  std::cout
      << "Powering the drive now. Keep clear of the joint and the e-stop within reach, Ctrl-C takes the power off.\n";
  if (!hal.enable())
  {
    std::cerr << "enable() failed\n";
    return 1;
  }
  DriveCommand command = neutral_command();
  command.position = start;
  command.stiffness = o.stiffness;
  command.damping = o.damping;
  double goal = start;
  const double max_step = TARGET_SPEED / CYCLE_HZ;
  auto next_turn = Clock::now();
  PrintTimer timer;

  const auto behind = run_cycles(o, [&] {
    if (!hal.read(states))
    {
      std::cerr << "read() failed\n";
      return false;
    }
    const DriveState& s = states[0];
    if (s.status_word & 0x08)
    {
      std::cerr << "Drive fault, error code 0x" << std::hex << s.error_code << std::dec << "\n";
      return false;
    }
    if (s.position < lowest || s.position > highest)
    {
      std::cerr << "The joint left the range [" << lowest << ", " << highest << "] rad\n";
      return false;
    }
    if (travel != 0.0 && Clock::now() >= next_turn)
    {
      goal = goal == start ? start + travel : start;
      next_turn += TURN_PERIOD;
    }
    command.position += std::clamp(goal - command.position, -max_step, max_step);
    if (timer.due())
    {
      std::cout << "setpoint " << command.position << " rad | ";
      print_state(s);
    }
    return hal.write({ command });
  });
  const bool disabled = hal.disable();
  std::cout << (disabled ? "Disabled\n" : "disable() failed\n");
  return behind && disabled ? 0 : 1;
}
}  // namespace

int main(int argc, char** argv)
{
  const auto options = parse_options(argc, argv);
  if (!options)
  {
    return 2;
  }
  rclcpp::init(argc, argv);
  std::signal(SIGINT, [](int) { g_stop = true; });
  if (options->rt)
  {
    request_real_time(options->rt_priority);  // threads started later by the HAL inherit it
  }

  const bool moves = options->check == "hold" || options->check == "move";
  int result = 1;
  {
    Bus bus;
    if (bus.open(*options, moves && options->impedance ? -6 : 8))
    {
      std::cout << "The drive streams its data and has no power yet\n";
      if (options->check == "read")
      {
        result = check_read(bus.hal(), *options);
      }
      else if (options->check == "timing")
      {
        result = check_timing(bus.hal(), *options);
      }
      else
      {
        result = check_motion(bus.hal(), *options);
      }
    }
  }
  rclcpp::shutdown();
  return result;
}
