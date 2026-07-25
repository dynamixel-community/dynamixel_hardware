// Copyright 2026 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <cstdint>

#include "dynamixel_hardware/workbench_driver.hpp"

namespace
{

using dynamixel_hardware::WorkbenchDriver;

ControlItem make_item(uint16_t address, uint8_t data_length)
{
  ControlItem item;
  item.address = address;
  item.data_length = data_length;
  return item;
}

TEST(TestWorkbenchDriver, read_window_starts_at_min_of_position_and_current_addresses)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  // Current/load block below the position block (Protocol 2.0 layout).
  WorkbenchDriver::compute_read_window(
    make_item(132, 4), make_item(128, 4), make_item(126, 2), start_address, read_length);
  EXPECT_EQ(126u, start_address);
  // Position block below the current/load block (Protocol 1.0 layout).
  WorkbenchDriver::compute_read_window(
    make_item(36, 2), make_item(38, 2), make_item(40, 2), start_address, read_length);
  EXPECT_EQ(36u, start_address);
}

TEST(TestWorkbenchDriver, read_window_length_includes_the_historical_two_byte_gap)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  // MX (Protocol 1.0): Present_Position 36 (2 bytes), Present_Speed 38
  // (2 bytes), Present_Load 40 (2 bytes) -> 2 + 2 + 2 + 2 gap = 8.
  WorkbenchDriver::compute_read_window(
    make_item(36, 2), make_item(38, 2), make_item(40, 2), start_address, read_length);
  EXPECT_EQ(8u, read_length);
}

// Representative X-series present block (e.g. XM430): Present_Current 126
// (2 bytes), Present_Velocity 128 (4 bytes), Present_Position 132 (4 bytes)
// -> one window starting at 126 of length 4 + 4 + 2 + 2 = 12.
TEST(TestWorkbenchDriver, read_window_covers_the_x_series_present_block)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  WorkbenchDriver::compute_read_window(
    make_item(132, 4), make_item(128, 4), make_item(126, 2), start_address, read_length);
  EXPECT_EQ(126u, start_address);
  EXPECT_EQ(12u, read_length);
}

TEST(TestWorkbenchDriver, read_window_degenerates_when_all_items_share_one_address)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  WorkbenchDriver::compute_read_window(
    make_item(100, 2), make_item(100, 2), make_item(100, 2), start_address, read_length);
  EXPECT_EQ(100u, start_address);
  EXPECT_EQ(8u, read_length);
}

}  // namespace
