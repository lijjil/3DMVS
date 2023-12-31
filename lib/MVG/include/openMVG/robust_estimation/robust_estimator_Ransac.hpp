// Ransac: 随机采样一致。可以动态地决定采样次数 。
// Ransac需要设置：阈值 。

// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_ROBUST_ESTIMATION_SIMPLE_RANSAC_HPP
#define OPENMVG_ROBUST_ESTIMATION_SIMPLE_RANSAC_HPP

#include <cassert>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include "openMVG/robust_estimation/rand_sampling.hpp"
#include "openMVG/robust_estimation/robust_ransac_tools.hpp"

namespace openMVG {
namespace robust{

/// \brief The famous Random Sample Consensus algorithm (Fischler&Bolles 1981).
/// \details The number of tests is reevaluated down as soon as more inliers are
/// found. Return the found model.
// 1. The model.
// 2. The minimum number of samples needed to fit.
// 3. A way to convert samples to a model.
// 4. A way to convert samples and a model to an error.
//
// 1. Kernel::Model
// 2. Kernel::MINIMUM_SAMPLES
// 3. Kernel::Fit(vector<int>, vector<Kernel::Model> *)
// 4. Kernel::Error(Model, int) -> error
template<typename Kernel, typename Scorer>
typename Kernel::Model RANSAC(
  const Kernel &kernel,
  const Scorer &scorer,
  std::vector<uint32_t> *best_inliers = nullptr ,
  size_t *best_score = nullptr , // Found number of inliers
  double outliers_probability = 1e-2)
{
  assert(outliers_probability < 1.0);
  assert(outliers_probability > 0.0);
  uint32_t iteration = 0;
  const uint32_t min_samples = Kernel::MINIMUM_SAMPLES;
  const uint32_t total_samples = kernel.NumSamples();

  uint32_t max_iterations = 100;
  const uint32_t really_max_iterations = 4096;

  uint32_t best_num_inliers = 0;
  double best_inlier_ratio = 0.0;
  typename Kernel::Model best_model;

  // Test if we have sufficient points for the kernel.
  if (total_samples < min_samples) {
    if (best_inliers) {
      best_inliers->resize(0);
    }
    return best_model;
  }

  // In this robust estimator, the scorer always works on all the data points
  // at once. So precompute the list ahead of time [0,..,total_samples].
  std::vector<uint32_t> all_samples(total_samples);
  std::iota(all_samples.begin(), all_samples.end(), 0);

  //--
  // Random number generation
  std::mt19937 random_generator(std::mt19937::default_seed);

  std::vector<uint32_t> sample;
  for (iteration = 0;
    iteration < max_iterations &&
    iteration < really_max_iterations; ++iteration) {
      UniformSample(min_samples, random_generator, &all_samples, &sample);

      std::vector<typename Kernel::Model> models;
      kernel.Fit(sample, &models);

      // Compute the inlier list for each fit.
      for (const auto& model_it : models) {
        std::vector<uint32_t> inliers;
        scorer.Score(kernel, model_it, all_samples, &inliers);

        if (best_num_inliers < inliers.size()) {
          best_num_inliers = inliers.size();
          best_inlier_ratio = inliers.size() / double(total_samples);
          best_model = model_it;
          if (best_inliers) {
            best_inliers->swap(inliers);
          }
        }
        if (best_inlier_ratio) {
          max_iterations = IterationsRequired(min_samples,
            outliers_probability,
            best_inlier_ratio);
        }
      }
  }
  if (best_score)
    *best_score = best_num_inliers;
  return best_model;
}


} // namespace robust
} // namespace openMVG

#endif // OPENMVG_ROBUST_ESTIMATION_SIMPLE_RANSAC_HPP
