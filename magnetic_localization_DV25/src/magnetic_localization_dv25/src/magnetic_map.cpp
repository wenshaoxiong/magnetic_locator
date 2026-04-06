#include "magnetic_localization_dv25/magnetic_map.hpp"

#include <yaml-cpp/yaml.h>

namespace magnetic_localization_dv25
{

bool MagneticMap::loadFromYaml(const std::string & path)
{
  sources_.clear();
  YAML::Node root = YAML::LoadFile(path);
  if (!root || !root["sources"]) {
    return false;
  }

  for (const auto & node : root["sources"]) {
    MagneticSource s;
    if (node["id"]) {
      s.id = node["id"].as<std::string>();
    }
    if (node["frame_id"]) {
      s.frame_id = node["frame_id"].as<std::string>();
    }
    if (node["position_m"] && node["position_m"].IsSequence() && node["position_m"].size() == 3) {
      s.position_m = Eigen::Vector3d(
        node["position_m"][0].as<double>(),
        node["position_m"][1].as<double>(),
        node["position_m"][2].as<double>());
    }
    if (node["moment"] && node["moment"].IsSequence() && node["moment"].size() == 3) {
      s.moment = Eigen::Vector3d(
        node["moment"][0].as<double>(),
        node["moment"][1].as<double>(),
        node["moment"][2].as<double>());
    }
    if (node["radius_m"]) {
      s.radius_m = node["radius_m"].as<double>();
    }
    if (node["length_m"]) {
      s.length_m = node["length_m"].as<double>();
    }
    if (node["type"]) {
      s.type = node["type"].as<std::string>();
    }
    if (node["spherical_harmonics_path"]) {
      s.spherical_harmonics_path = node["spherical_harmonics_path"].as<std::string>();
      if (!s.spherical_harmonics_path.empty()) {
        try {
          YAML::Node sh_node = YAML::LoadFile(s.spherical_harmonics_path);
          if (sh_node["coefficients"]) {
            for (const auto & c : sh_node["coefficients"]) {
              MagneticSource::SHCoeff coeff;
              coeff.n = c["n"].as<int>();
              coeff.m = c["m"].as<int>();
              coeff.g = c["g"].as<double>();
              coeff.h = c["h"].as<double>();
              s.sh_coeffs.push_back(coeff);
            }
          }
        } catch (...) {
          // 忽略路径错误或解析失败
        }
      }
    }
    sources_.push_back(s);
  }

  return !sources_.empty();
}

const std::vector<MagneticSource> & MagneticMap::sources() const
{
  return sources_;
}

void MagneticMap::updateSourcePose(const std::string & id, const Eigen::Vector3d & pos, const Eigen::Vector3d & moment)
{
  for (auto & s : sources_) {
    if (s.id == id) {
      s.position_m = pos;
      s.moment = moment;
      return;
    }
  }
}

}  // namespace magnetic_localization_dv25
