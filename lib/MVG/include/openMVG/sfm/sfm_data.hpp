// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_SFM_DATA_HPP
#define OPENMVG_SFM_SFM_DATA_HPP

#include <string>

#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/sfm/sfm_landmark.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/types.hpp"

namespace openMVG {
namespace sfm {

// 相机的内参属性
/// Define a collection of IntrinsicParameter (indexed by View::id_intrinsic)
using Intrinsics = Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase>>;

// 定义姿势集合（按View::id_Pose索引）。方向矩阵和旋转中心 Mat3 rotation_;   Vec3 center_;			Eigen类型
/// Define a collection of Pose (indexed by View::id_pose)
using Poses = Hash_Map<IndexT, geometry::Pose3>;

// 存储View类型。View视图通过一个字符串和视图、相机和姿势的唯一索引来定义图像
/// Define a collection of View (indexed by View::id_view)
using Views = Hash_Map<IndexT, std::shared_ptr<View>>;


// SfM_Data类 包含了SfM的相关数据
/// Generic SfM data container
/// Store structure and camera properties:
struct SfM_Data
{
  /// Considered views    // Views结构体包含: 1 图像路径以及图像文件名， 2 图像ID id_view，3 内参数和位姿的id id_pose，id_intrinsic，4 图像大小 image size。
  Views views;

  /// Considered poses (indexed by view.id_pose)
  /// 影像 外参，包括 旋转矩阵、平移矩阵
  // using Poses = Hash_Map<IndexT, geometry::Pose3>;
  // Pose3就是相机3x3旋转矩阵变量和一个3x1的相机位置变量
  Poses poses;

  // 相机内参，支持多组不同内参数
  /// Considered camera intrinsics (indexed by view.id_intrinsic)
  // using Intrinsics = Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase>>;
  // IntrinsicBase的成员变量：实际上 IntrinsicBase结构 中只有width和height变量
  Intrinsics intrinsics;

  // 二维视图特征关联的 3D点云。 Landmark包含两个成员：3d点、及其所对应于图像上的坐标的哈系表。
  // 因为一个世界中的坐标可以被多张相机所观测到。Landmarks点位又分为三角测量获得的点（用于BA）和地面控制点（用于GCP）
  /// Structure (3D points with their 2D observations)
  // using Landmarks = Hash_Map<IndexT, Landmark>;
    // 其中Landmark的成员变量：
    //   (1)3d points: Vec3 X;
    //   (2)3D点对应的观测图像特征点：Observations obs;//using Observations = Hash_Map<IndexT, Observation>;
    //其中IndexT为ImageID, Observation的成员变量：
    //   (1)2d image points:  Vec2 x;
    //   (2)特征点的id：  IndexT id_feat;
  Landmarks structure;

  /// Controls points (stored as Landmarks (id_feat has no meaning here))
  Landmarks control_points;

  /// Root Views path     // 图片的根目录路径
  std::string s_root_path;

  //--
  // Accessors
  //--
  const Views & GetViews() const {return views;}
  const Poses & GetPoses() const {return poses;}
  const Intrinsics & GetIntrinsics() const {return intrinsics;}
  const Landmarks & GetLandmarks() const {return structure;}
  const Landmarks & GetControl_Points() const {return control_points;}

  /// Check if the View have defined intrinsic and pose
  bool IsPoseAndIntrinsicDefined(const View * view) const
  {
    if (!view) return false;
    return (
      view->id_intrinsic != UndefinedIndexT &&
      view->id_pose != UndefinedIndexT &&
      intrinsics.find(view->id_intrinsic) != intrinsics.end() &&
      poses.find(view->id_pose) != poses.end());
  }

  /// Get the pose associated to a view
  const geometry::Pose3 GetPoseOrDie(const View * view) const
  {
    return poses.at(view->id_pose);
  }
};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_SFM_DATA_HPP
