// openMVG几何滤波的结构

// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_IMAGE_COLLECTION_GEOMETRIC_FILTER_HPP
#define OPENMVG_MATCHING_IMAGE_COLLECTION_GEOMETRIC_FILTER_HPP

#include <algorithm>
#include <map>
#include <vector>

#include "openMVG/features/feature.hpp"
#include "openMVG/matching/indMatch.hpp"

#include "third_party/progress/progress_display.hpp"

namespace openMVG { namespace sfm { struct Regions_Provider; } }
namespace openMVG { namespace sfm { struct SfM_Data; } }

namespace openMVG {

namespace matching_image_collection {

using namespace openMVG::matching;


// 几何滤波的对象为 ImageCollectionGeometricFilter ，是一个结构体，而且没法定义为纯虚的。
/// Allow to keep only geometrically coherent matches
/// -> It discards pairs that do not lead to a valid robust model estimation
struct ImageCollectionGeometricFilter
{
  ImageCollectionGeometricFilter
  (
    const sfm::SfM_Data * sfm_data,
    const std::shared_ptr<sfm::Regions_Provider> & regions_provider
  ):sfm_data_(sfm_data),
    regions_provider_(regions_provider)
  {}


  // 在ImageCollectionGeometricFilter结构体中定义了一个模板函数，模板参数为一个几何滤波的类
  /// Perform robust model estimation (with optional guided_matching) for all
  /// the pairs and regions correspondences contained in the putative_matches
  /// set.
  template<typename GeometryFunctor>
  void Robust_model_estimation
  (
    const GeometryFunctor & functor,
    const PairWiseMatches & putative_matches,
    const bool b_guided_matching = false,
    const double d_distance_ratio = 0.6,
    C_Progress *progress_bar = nullptr
  );

  const PairWiseMatches & Get_geometric_matches() const
  {
    return _map_GeometricMatches;
  }

  // Data
  const sfm::SfM_Data * sfm_data_;
  const std::shared_ptr<sfm::Regions_Provider> & regions_provider_;
  PairWiseMatches _map_GeometricMatches;
};



// 模板函数的实现 。 调用 Robust_model_estimation 函数 进行几何滤波 。而这个函数会调用模板中的几何滤波方法：
// 模板参数为一个几何滤波的类，在openMVG中有多种几何滤波的方法可选 。
template<typename GeometryFunctor>
void ImageCollectionGeometricFilter::Robust_model_estimation
(
  //functor 中使用的是 GeometricFilter_HMatrix_AC 等:
  // 1. GeometricFilter_HMatrix_AC      H 矩阵
  // 2. GeometricFilter_FMatrix_AC      F 矩阵
  // 3. GeometricFilter_EMatrix_AC      E 矩阵
  // 4. GeometricFilter_ESphericalMatrix_AC_Angular
  // 5. GeometricFilter_ESphericalMatrix_AC_Angular
  // 6. GeometricFilter_EOMatrix_RA
  const GeometryFunctor & functor,

  // PairWiseMatches 继承于 std::map<Pair, IndMatches>
  // IndMatches 是 std::vector<matching::IndMatch>
  // matching::IndMatch 是结构以保存成对索引的引用，存在排序运算符以删除 IndMatch 序列的重复项。
  // 也就是放入匹配点的数据结构
  const PairWiseMatches & putative_matches,
  const bool b_guided_matching,
  const double d_distance_ratio,
  C_Progress * my_progress_bar      // 进度条
)
{
  if (!my_progress_bar)
    my_progress_bar = &C_Progress::dummy();
  my_progress_bar->restart( putative_matches.size(), "\n- Geometric filtering -\n" );

#ifdef OPENMVG_USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i = 0; i < (int)putative_matches.size(); ++i)
  {
    if (my_progress_bar->hasBeenCanceled())
      continue;
    auto iter = putative_matches.begin();
    advance(iter,i);

    Pair current_pair = iter->first;
    const std::vector<IndMatch> & vec_PutativeMatches = iter->second;

    //-- Apply the geometric filter (robust model estimation)
    {
      IndMatches putative_inliers;

      // 调用模板中的几何滤波方法
      GeometryFunctor geometricFilter = functor; // use a copy since we are in a multi-thread context

      // 并调用 geometricFilter.Robust_estimation ,这个函数内部会调用指定功能的几何滤波方法。
      if (geometricFilter.Robust_estimation(
        sfm_data_,
        regions_provider_,
        iter->first,
        vec_PutativeMatches,
        putative_inliers))
      {
        if (b_guided_matching)
        {
          IndMatches guided_geometric_inliers;
          geometricFilter.Geometry_guided_matching(
            sfm_data_,
            regions_provider_,
            iter->first,
            d_distance_ratio,
            guided_geometric_inliers);
          //std::cout
          // << "#before/#after: " << putative_inliers.size()
          // << "/" << guided_geometric_inliers.size() << std::endl;
          std::swap(putative_inliers, guided_geometric_inliers);
        }

#ifdef OPENMVG_USE_OPENMP
#pragma omp critical
#endif
        {
          _map_GeometricMatches.insert( {current_pair, std::move(putative_inliers)});
        }
      }
    }
    ++(*my_progress_bar);
  }
}

} // namespace matching_image_collection
} // namespace openMVG

#endif // OPENMVG_MATCHING_IMAGE_COLLECTION_GEOMETRIC_FILTER_HPP
