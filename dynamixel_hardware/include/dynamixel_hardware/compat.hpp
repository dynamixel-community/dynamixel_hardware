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

#pragma once

#include <hardware_interface/version.h>

#define DXL_HAS_PARAMS_ON_INIT HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
// ConstSharedPtr on_export_* signatures exist since 4.19.0 (4.18.x used SharedPtr)
#define DXL_HAS_ON_EXPORT HARDWARE_INTERFACE_VERSION_GTE(4, 19, 0)
#define DXL_HAS_COMPONENT_LOGGER HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
// Test-side ResourceManager constructor gate.
#define DXL_HAS_RM_PARAMS_CTOR HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
