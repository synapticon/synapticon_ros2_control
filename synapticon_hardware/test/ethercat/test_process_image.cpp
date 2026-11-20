// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief One drive's process image: its layout, and the bytes it reads and writes, as the EtherCAT ICube HAL's
/// generic CiA 402 drive does.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "synapticon_hardware/ethercat/process_image.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"

namespace
{
using namespace synapticon_hardware::ethercat;
namespace ec = synapticon_hardware::ethercat;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr int CSP = 8;
constexpr int CSV = 9;
constexpr int IMPEDANCE = -6;
constexpr uint8_t UNTOUCHED = 0xAA;

PdoChannel command(uint16_t index, const char* type, const char* interface, double default_value = NaN)
{
  PdoChannel c;
  c.index = index;
  c.type = *value_type(type);
  c.command_interface = interface;
  c.default_value = default_value;
  return c;
}

PdoChannel state(uint16_t index, const char* type, const char* interface, double factor = 1.0, double offset = 0.0)
{
  PdoChannel c;
  c.index = index;
  c.type = *value_type(type);
  c.state_interface = interface;
  c.factor = factor;
  c.offset = offset;
  return c;
}

/// A small drive: controlword, mode, target position, target velocity and the digital outputs without a command;
/// statusword, mode display, position (scaled by 2, offset 1) and the 8-byte error report.
SlaveConfig small_drive()
{
  SlaveConfig config;
  PdoChannel unused;
  unused.index = 0x60FE;
  unused.sub_index = 1;
  unused.type = *value_type("uint32");
  config.rpdo = { { 0x1600,
                    { command(0x6040, "uint16", "control_word", 0), command(0x6060, "int8", "mode_of_operation", -6),
                      command(0x607A, "int32", "position"), command(0x60FF, "int32", "velocity", 0), unused } } };
  config.tpdo = { { 0x1A00,
                    { state(0x6041, "uint16", "status_word"), state(0x6061, "int8", "mode_of_operation"),
                      state(0x6064, "int32", "position", 2.0, 1.0), state(0x203F, "real64", "error_report") } } };
  return config;
}

/// Byte offsets of the small drive's entries.
constexpr std::size_t OUT_CW = 0, OUT_MODE = 2, OUT_TARGET = 3, OUT_VELOCITY = 7, OUT_UNUSED = 11;
constexpr std::size_t IN_SW = 0, IN_DISPLAY = 2, IN_POSITION = 3, IN_REPORT = 7;

template <typename T>
T at(const std::vector<uint8_t>& bytes, std::size_t offset)
{
  T value;
  std::memcpy(&value, bytes.data() + offset, sizeof(T));
  return value;
}

template <typename T>
void put(std::vector<uint8_t>& bytes, std::size_t offset, T value)
{
  std::memcpy(bytes.data() + offset, &value, sizeof(T));
}

/// The small drive in `mode`, with its buffers and bytes.
class SmallDrive : public ::testing::Test
{
protected:
  explicit SmallDrive(int mode = CSP) : image_(small_drive(), mode)
  {
  }

  /// The drive reports `display` as its mode and `position` as its raw actual position.
  void report(int8_t display, int32_t position)
  {
    put<uint16_t>(input_bytes_, IN_SW, 0x0237);
    put<int8_t>(input_bytes_, IN_DISPLAY, display);
    put<int32_t>(input_bytes_, IN_POSITION, position);
    image_.read_inputs(input_bytes_.data(), inputs_);
  }

  void write()
  {
    image_.write_outputs(outputs_, output_bytes_.data());
  }

  ProcessImage image_;
  std::vector<double> outputs_ = std::vector<double>(ec::layout::OUTPUT_SIZE, NaN);
  std::vector<double> inputs_ = std::vector<double>(ec::layout::INPUT_SIZE, 0.0);
  std::vector<uint8_t> output_bytes_ = std::vector<uint8_t>(15, UNTOUCHED);
  std::vector<uint8_t> input_bytes_ = std::vector<uint8_t>(15, 0);
};

class ImpedanceDrive : public SmallDrive
{
protected:
  ImpedanceDrive() : SmallDrive(IMPEDANCE)
  {
  }
};

// --- Layout ---------------------------------------------------------------------------------

TEST_F(SmallDrive, PacksTheEntriesInMappingOrder)
{
  EXPECT_EQ(image_.output_bytes(), 15u);
  EXPECT_EQ(image_.input_bytes(), 15u);
}

TEST(ExampleDrive, HasTheSizesOfItsPdos)
{
  std::string error;
  const auto config = load_slave_config(ament_index_cpp::get_package_share_directory("synapticon_hardware") +
                                            "/config/example_somanet_circulo_safe_motion.yaml",
                                        error);
  ASSERT_TRUE(config) << error;
  const ProcessImage image(*config, CSP);
  EXPECT_EQ(image.output_bytes(), 27u);  // controlword, mode, six 32-bit entries
  EXPECT_EQ(image.input_bytes(), 43u);   // 0x1A00, the diagnostics with the 8-byte error report
}

// --- Relocation -----------------------------------------------------------------------------

/// A locator that puts each channel of `offsets` (object index to byte offset) at that byte, and has no place for others.
ChannelLocator at_bytes(std::vector<std::pair<uint16_t, std::size_t>> offsets)
{
  return [offsets](const PdoChannel& channel) -> std::optional<std::size_t> {
    for (const auto& [index, byte] : offsets)
    {
      if (index == channel.index)
      {
        return byte * 8;
      }
    }
    return std::nullopt;
  };
}

/// The small drive's entries scattered in a 64-byte image, in another order than in the slave_config.
const auto SCATTERED_OUTPUTS =
    at_bytes({ { 0x6040, 40 }, { 0x6060, 10 }, { 0x607A, 20 }, { 0x60FF, 30 }, { 0x60FE, 50 } });
const auto SCATTERED_INPUTS = at_bytes({ { 0x6041, 33 }, { 0x6061, 3 }, { 0x6064, 12 }, { 0x203F, 50 } });

TEST(Relocation, ReadsAndWritesTheChannelsWhereTheMasterPutThem)
{
  ProcessImage image(small_drive(), CSP);
  std::string error;
  ASSERT_TRUE(image.relocate(SCATTERED_OUTPUTS, SCATTERED_INPUTS, error)) << error;
  EXPECT_EQ(image.output_bytes(), 15u);  // the sizes of the packed layout
  EXPECT_EQ(image.input_bytes(), 15u);

  std::vector<uint8_t> in(64, 0);
  put<uint16_t>(in, 33, 0x0237);
  put<int8_t>(in, 3, CSP);
  put<int32_t>(in, 12, -100);
  std::vector<double> inputs(ec::layout::INPUT_SIZE, 0.0);
  image.read_inputs(in.data(), inputs);
  EXPECT_EQ(inputs[ec::slot::STATUS_WORD], 0x0237);
  EXPECT_EQ(inputs[ec::slot::MODE_DISPLAY], CSP);
  EXPECT_EQ(inputs[ec::slot::ACTUAL_POSITION], -199.0);

  std::vector<uint8_t> out(64, UNTOUCHED);
  std::vector<double> outputs(ec::layout::OUTPUT_SIZE, NaN);
  outputs[ec::slot::CONTROL_WORD] = 0x000F;
  outputs[ec::slot::TARGET_POSITION] = 1234;
  image.write_outputs(outputs, out.data());
  EXPECT_EQ(at<uint16_t>(out, 40), 0x000F);
  EXPECT_EQ(at<int8_t>(out, 10), CSP);
  EXPECT_EQ(at<int32_t>(out, 20), 1234);
  EXPECT_EQ(out[0], UNTOUCHED);   // nothing at the packed offsets
  EXPECT_EQ(out[50], UNTOUCHED);  // the channel without a command
}

TEST(Relocation, IsRefusedForAChannelWithoutAPlace)
{
  ProcessImage image(small_drive(), CSP);
  std::string error;
  EXPECT_FALSE(image.relocate(at_bytes({ { 0x6040, 0 } }), SCATTERED_INPUTS, error));
  EXPECT_NE(error.find("0x6060:0"), std::string::npos) << error;
}

TEST(Relocation, IsRefusedForAChannelThatDoesNotStartOnAByte)
{
  ProcessImage image(small_drive(), CSP);
  const ChannelLocator off_by_a_bit = [](const PdoChannel& channel) -> std::optional<std::size_t> {
    return channel.index == 0x6041 ? 3 : 64;
  };
  std::string error;
  EXPECT_FALSE(image.relocate(SCATTERED_OUTPUTS, off_by_a_bit, error));
  EXPECT_NE(error.find("0x6041:0"), std::string::npos) << error;

  // Unchanged after the refusal: still the packed layout.
  std::vector<uint8_t> in(15, 0);
  put<uint16_t>(in, IN_SW, 0x0237);
  std::vector<double> inputs(ec::layout::INPUT_SIZE, 0.0);
  image.read_inputs(in.data(), inputs);
  EXPECT_EQ(inputs[ec::slot::STATUS_WORD], 0x0237);
}

// --- Inputs ---------------------------------------------------------------------------------

TEST_F(SmallDrive, ReadsScaledValuesIntoTheirSlots)
{
  report(CSP, -100);
  EXPECT_EQ(inputs_[ec::slot::STATUS_WORD], 0x0237);
  EXPECT_EQ(inputs_[ec::slot::MODE_DISPLAY], CSP);
  EXPECT_EQ(inputs_[ec::slot::ACTUAL_POSITION], -199.0);  // 2 * -100 + 1
}

TEST_F(SmallDrive, CopiesTheErrorReportBitForBit)
{
  const char text[8] = { 'E', '1', '2', '3', 'x', 0, 0, 0 };
  std::memcpy(input_bytes_.data() + IN_REPORT, text, sizeof(text));
  report(CSP, 0);
  double expected;
  std::memcpy(&expected, text, sizeof(text));
  const std::size_t slot = ec::slot::ERROR_REPORT;  // the error_report channel
  EXPECT_EQ(std::memcmp(&inputs_[slot], &expected, sizeof(double)), 0);
}

// --- Outputs ---------------------------------------------------------------------------------

TEST_F(SmallDrive, WritesTheControlwordAndTheJointsMode)
{
  outputs_[ec::slot::CONTROL_WORD] = 0x000F;
  write();
  EXPECT_EQ(at<uint16_t>(output_bytes_, OUT_CW), 0x000F);
  EXPECT_EQ(at<int8_t>(output_bytes_, OUT_MODE), CSP);
}

TEST_F(ImpedanceDrive, TakesAVendorModeFromTheSlaveConfig)
{
  write();
  EXPECT_EQ(at<int8_t>(output_bytes_, OUT_MODE), IMPEDANCE);
}

TEST_F(SmallDrive, WritesTheDefaultOfACommandThatIsNaN)
{
  write();
  EXPECT_EQ(at<int32_t>(output_bytes_, OUT_VELOCITY), 0);
  outputs_[ec::slot::TARGET_VELOCITY] = 3.7;
  write();
  EXPECT_EQ(at<int32_t>(output_bytes_, OUT_VELOCITY), 3);  // truncated
}

TEST_F(SmallDrive, LeavesAnEntryWithoutCommandUntouched)
{
  write();
  EXPECT_EQ(at<uint32_t>(output_bytes_, OUT_UNUSED), 0xAAAAAAAAu);
}

/// Before the drive reports a mode, the target keeps its bytes, whatever the command.
TEST_F(SmallDrive, WritesNoTargetBeforeTheDriveReportsAMode)
{
  outputs_[ec::slot::TARGET_POSITION] = 500.0;
  write();
  EXPECT_EQ(at<uint32_t>(output_bytes_, OUT_TARGET), 0xAAAAAAAAu);
}

TEST_F(SmallDrive, WritesTheActualPositionAsTargetWhileTheCommandIsNaN)
{
  report(CSP, -100);
  write();
  EXPECT_EQ(at<int32_t>(output_bytes_, OUT_TARGET), -199);
}

TEST_F(SmallDrive, WritesTheCommandedTargetInCsp)
{
  report(CSP, -100);
  outputs_[ec::slot::TARGET_POSITION] = 500.0;
  write();
  EXPECT_EQ(at<int32_t>(output_bytes_, OUT_TARGET), 500);
}

TEST_F(ImpedanceDrive, WritesTheCommandedTargetInImpedanceMode)
{
  report(IMPEDANCE, -100);
  outputs_[ec::slot::TARGET_POSITION] = 500.0;
  write();
  EXPECT_EQ(at<int32_t>(output_bytes_, OUT_TARGET), 500);
}

/// In CSV the drive does not follow a position target, so it always gets the actual position.
TEST_F(SmallDrive, WritesTheActualPositionAsTargetInCsv)
{
  report(CSV, -100);
  outputs_[ec::slot::TARGET_POSITION] = 500.0;
  write();
  EXPECT_EQ(at<int32_t>(output_bytes_, OUT_TARGET), -199);
}

// --- Values -------------------------------------------------------------------------------------

TEST(Values, ANegativeValueWrapsIntoAnUnsignedEntry)
{
  std::vector<uint8_t> bytes(4, 0);
  write_value(bytes.data(), 0, *value_type("uint32"), 255, -1.0);
  EXPECT_EQ(at<uint32_t>(bytes, 0), 0xFFFFFFFFu);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("uint32"), 255), 4294967295.0);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("int32"), 255), -1.0);
}

TEST(Values, SignedEntriesKeepTheirSign)
{
  std::vector<uint8_t> bytes(8, 0);
  write_value(bytes.data(), 8, *value_type("int16"), 255, -1234.0);
  EXPECT_EQ(read_value(bytes.data(), 8, *value_type("int16"), 255), -1234.0);
  write_value(bytes.data(), 0, *value_type("int64"), 255, -5.0);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("int64"), 255), -5.0);
}

TEST(Values, IntegerEntriesSaturateAtTheirRange)
{
  std::vector<uint8_t> bytes(4, 0);
  write_value(bytes.data(), 0, *value_type("int32"), 255, std::numeric_limits<double>::infinity());
  EXPECT_EQ(at<int32_t>(bytes, 0), std::numeric_limits<int32_t>::max());
  write_value(bytes.data(), 0, *value_type("int16"), 255, -std::numeric_limits<double>::infinity());
  EXPECT_EQ(at<int16_t>(bytes, 0), std::numeric_limits<int16_t>::min());
  write_value(bytes.data(), 0, *value_type("uint16"), 255, 1e12);
  EXPECT_EQ(at<uint16_t>(bytes, 0), 0xFFFF);
  std::vector<uint8_t> wide(8, 0);
  write_value(wide.data(), 0, *value_type("uint64"), 255, std::numeric_limits<double>::infinity());
  EXPECT_EQ(at<uint64_t>(wide, 0), std::numeric_limits<uint64_t>::max());
  write_value(wide.data(), 0, *value_type("int64"), 255, -1e300);
  EXPECT_EQ(at<int64_t>(wide, 0), std::numeric_limits<int64_t>::min());
}

TEST(Values, RealsRoundTrip)
{
  std::vector<uint8_t> bytes(8, 0);
  write_value(bytes.data(), 0, *value_type("real32"), 255, 1.5);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("real32"), 255), 1.5);
  write_value(bytes.data(), 0, *value_type("real64"), 255, -2.25);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("real64"), 255), -2.25);
}

TEST(Values, BitEntriesWorkUnderTheirMask)
{
  std::vector<uint8_t> bytes{ 0x81 };
  write_value(bytes.data(), 0, *value_type("bool"), 0x04, 1.0);
  EXPECT_EQ(bytes[0], 0x85);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("bool"), 0x04), 1.0);
  write_value(bytes.data(), 0, *value_type("bool"), 0x04, 0.0);
  EXPECT_EQ(bytes[0], 0x81);
  write_value(bytes.data(), 0, *value_type("bit4"), 0x0F, 0x0A);
  EXPECT_EQ(bytes[0], 0x8A);
  EXPECT_EQ(read_value(bytes.data(), 0, *value_type("bit4"), 0x0F), 10.0);
}

}  // namespace
