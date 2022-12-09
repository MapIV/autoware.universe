// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_CONTROL_TOOLBOX_HPP_
#define AUTOWARE_CONTROL_TOOLBOX_HPP_

#include <boost/optional.hpp>

#include <array>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

// Library headers
#include "autoware_control_toolbox/control/act_definitions.hpp"
#include "autoware_control_toolbox/control/act_signal_builder.hpp"
#include "autoware_control_toolbox/utils_act/act_utils_eigen.hpp"
#include "autoware_control_toolbox/control/balance.hpp"
#include "autoware_control_toolbox/control/state_space.hpp"
#include "autoware_control_toolbox/control/tf_algebra.hpp"
#include "autoware_control_toolbox/control/transfer_functions.hpp"
#include "autoware_control_toolbox/splines/bsplines_smoother.hpp"
#include "autoware_control_toolbox/splines/bspline_interpolator_templated.hpp"
#include "autoware_control_toolbox/splines/bsplines_interpolator.hpp"
#include "autoware_control_toolbox/splines/interpolating_spline_pcg.hpp"

#endif  // AUTOWARE_CONTROL_TOOLBOX_HPP_