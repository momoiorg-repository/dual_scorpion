#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Modifications Copyright 2025 S.Satoya
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass

from ..config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("dual_scropion_leader")
@dataclass
class DualScropionLeaderConfig(TeleoperatorConfig):
    """
    Configuration for the SO-101 Dual Leader Arm Teleoperator
    """
    # Port to connect to the arm
    right_arm_port: str  # Port for the right arm (e.g. "/dev/ttyACM1")
    left_arm_port: str  # Port for the left arm (e.g. "/dev/ttyACM3")

    use_degrees: bool = False