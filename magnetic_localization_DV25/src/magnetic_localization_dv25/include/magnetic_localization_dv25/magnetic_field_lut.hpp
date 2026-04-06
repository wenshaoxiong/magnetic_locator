#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace magnetic_localization_dv25
{

class MagneticFieldLUT
{
public:
  bool loadNpzStored(const std::string & npz_path);
  Eigen::Vector3d interpolateFieldT(const Eigen::Vector3d & p_map) const;
  bool isInside(const Eigen::Vector3d & p_map) const;
  bool valid() const;

private:
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> z_;
  std::vector<double> B_;
  int nx_{0};
  int ny_{0};
  int nz_{0};

  int clampIndex(int idx, int max) const;
  double atB(int ix, int iy, int iz, int c) const;
};

}  // namespace magnetic_localization_dv25
