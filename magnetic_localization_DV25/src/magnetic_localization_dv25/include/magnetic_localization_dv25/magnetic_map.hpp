#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace magnetic_localization_dv25
{

struct MagneticSource
{
  std::string id;
  std::string frame_id; // 磁源在 TF 中的坐标系 ID
  Eigen::Vector3d position_m{0.0, 0.0, 0.0};
  Eigen::Vector3d moment{0.0, 0.0, 1.0};
  double radius_m{0.01};  // 磁体半径 (m)
  double length_m{0.02};  // 磁体长度 (m)
  std::string type{"cylinder"}; // "cylinder" or "rectangular"
  
  // 球谐系数 (Spherical Harmonics Coefficients)
  // n, m, g, h
  struct SHCoeff {
    int n, m;
    double g, h;
  };
  std::vector<SHCoeff> sh_coeffs;
  std::string spherical_harmonics_path;
};

class MagneticMap
{
public:
  bool loadFromYaml(const std::string & path);
  const std::vector<MagneticSource> & sources() const;
  void updateSourcePose(const std::string & id, const Eigen::Vector3d & pos, const Eigen::Vector3d & moment);

private:
  std::vector<MagneticSource> sources_;
};

}  // namespace magnetic_localization_dv25
