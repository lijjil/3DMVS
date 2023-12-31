// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_IMAGE_COLLECTION_MATCHER_HPP
#define OPENMVG_MATCHING_IMAGE_COLLECTION_MATCHER_HPP

#include <string>
#include <vector>
#include <memory>

#include "openMVG/matching/indMatch.hpp"

class C_Progress;

namespace openMVG {

namespace sfm {
  struct Regions_Provider;
} // namespace sfm


namespace matching_image_collection {


// 匹配函数是一个纯虚函数，必须继承重写才能使用。
// 有两个类继承了 Matcher：Cascade_Hashing_Matcher_Regions  、 Matcher_Regions 。
// Matcher_Regions 使用的频次更多
/// Implementation of an Image Collection Matcher
/// Compute putative matches between a collection of pictures
class Matcher
{
  public:
  Matcher() = default;

  virtual ~Matcher() = default;

  /// Find corresponding points between some pair of view Ids
  virtual void Match(
    const std::shared_ptr<sfm::Regions_Provider> & regions_provider,
    const Pair_Set & pairs, // list of pair to consider for matching
    matching::PairWiseMatchesContainer & map_putatives_matches, // the output pairwise photometric corresponding points
    C_Progress * progress = nullptr
    )const = 0;
};

} // namespace matching_image_collection
} // namespace openMVG

#endif // OPENMVG_MATCHING_IMAGE_COLLECTION_MATCHER_HPP
