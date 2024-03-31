#include <gtsam/geometry/Pose3.h>

#include <iostream>
int main(int argc, char** argv) {
  // B在姿态上绕x轴旋转180度
  gtsam::Pose3 B =
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 2, 0));
  // A在姿态上绕z轴旋转180度
  gtsam::Pose3 A =
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 1, 0));
  // 动坐标系为基准：
  // 从零点平移 gtsam::Point3(2, 1, 0)再旋转gtsam::Rot3(0, 0, 0, 1)
  // 再平移 gtsam::Point3(1, 2, 0)再旋转gtsam::Rot3(0, 1, 0, 0)
  auto it = A.compose(B);
  std::cout << it << std::endl;
  return 0;
}
