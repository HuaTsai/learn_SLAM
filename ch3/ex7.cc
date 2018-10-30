#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char *argv[]) {
  Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
  q1.normalize();
  Eigen::Isometry3d T1(q1);
  Eigen::Vector3d t1(0.3, 0.1, 0.1);
  T1.pretranslate(t1);
  std::cout << "T1w:" << std::endl << T1.matrix() << std::endl << std::endl;

  Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
  q2.normalize();
  Eigen::Isometry3d T2(q2);
  Eigen::Vector3d t2(-0.1, 0.5, 0.3);
  T2.pretranslate(t2);
  std::cout << "T2w:" << std::endl << T2.matrix() << std::endl << std::endl;

  Eigen::Vector3d p1(0.5, 0, 0.2);
  Eigen::Vector3d p2;
  p2 = T2 * T1.inverse() * p1;
  std::cout << "p2: " << p2.transpose() << std::endl;
}