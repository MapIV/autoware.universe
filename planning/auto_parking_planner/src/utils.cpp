// Copyright 2022 Tier IV, Inc. All rights reserved.
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

#include "auto_parking_planner.hpp"

namespace auto_parking_planner
{

bool containLanelet(const lanelet::ConstPolygon3d & polygon, const lanelet::ConstLanelet & llt)
{
  // if one of the vertexes of the lanlet is contained by the polygon
  // this function returns true
  const lanelet::CompoundPolygon3d llt_poly = llt.polygon3d();
  for (const auto & pt : llt_poly) {
    if (lanelet::geometry::within(pt, polygon.basicPolygon())) {
      return true;
    }
  }
  return false;
}

}  // namespace auto_parking_planner
