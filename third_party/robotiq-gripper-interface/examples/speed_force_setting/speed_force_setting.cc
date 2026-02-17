// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License").
// You may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robotiq/robotiq_gripper_interface.h"
#include <iostream>

// Define the communication parameters
std::string port = robotiq::DEFAULT_PORT;
std::size_t baud = robotiq::DEFAULT_BAUD;

// Define the scale factors.  Here we map the position such that 100 is
// fully open and 0 is fully closed.
const double alpha = 100.0;
const double beta = 0.0;

bool parse_args(int argc, char* argv[]) {
  for (int i = 1; i < argc; i += 2) {
    if (std::string(argv[i]) == "--help") {
      std::cout << "  --port <value> Optional serial port ID\n";
      std::cout << "  --baud <value> Optional baud rate\n";
      return false;
    } else if (std::string(argv[i]) == "--port") {
      port = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--baud") {
      baud = std::atoi(argv[i + 1]);
    }
  }
  return true;
}

void print_feedback(const robotiq::GripperFeedback& feedback) {
  std::cout << "\n\n";
  std::cout << "===============================\n";
  std::cout << "Gripper feedback:\n";
  std::cout << "  commanded_position:     " << feedback.commanded_position << "\n";
  std::cout << "  position:               " << feedback.position << "\n";
  std::cout << "  current:                " << feedback.current << "\n";
  std::cout << "===============================\n";
}

int main(int argc, char* argv[]) {
  // Load the args
  if (not parse_args(argc, argv)) {
    return 0;
  }

  // Open the serial port
  robotiq::RobotiqGripperInterface gripper;
  std::cout << "Connected: " << gripper.connect(port, baud, alpha, beta) << "\n";

  // Check if gripper is activated and activate otherwise
  if (not gripper.is_activated()) {
    std::cout << "Gripper is not activated... Activating...";
    gripper.activate();
  }
  std::cout << "Gripper is activated!\n";

  // Close gripper with no force
  std::cout << "Move to 60% with no force: " << gripper.set_gripper_position(60.0, 50.0, 100) << "\n";
  print_feedback(gripper.get_feedback());
  std::cout << "\n\nPress Enter to continue...\n\n" << std::endl;
  std::cin.get();

  // Hold the position but change the force. Grip at 50%

  std::cout << "Grip 50% tighter: " << gripper.set_gripper_position(61.0, 50.0, 50.0) << "\n";
  print_feedback(gripper.get_feedback());
  std::cout << "\n\nPress Enter to continue...\n\n" << std::endl;
  std::cin.get();

  // Hold the position but change the force. Grip tighter at 100%
  std::cout << "Grip tightly " << gripper.set_gripper_position(62.0, 50.0, 0.0) << "\n";
  print_feedback(gripper.get_feedback());
  std::cout << "\n\nPress Enter to continue...\n\n" << std::endl;
  std::cin.get();

  // Hold the position but change the force. Relax it to 0%
  std::cout << "Relax grip" << gripper.set_gripper_position(61.0, 50.0, 100.0) << "\n";
  print_feedback(gripper.get_feedback());
  std::cout << "\n\nPress Enter to continue...\n\n" << std::endl;
  std::cin.get();

  std::cout << "Open gripper" << gripper.set_gripper_position(0.0, 50.0, 100.0) << "\n";
  print_feedback(gripper.get_feedback());

  return 0;
}
