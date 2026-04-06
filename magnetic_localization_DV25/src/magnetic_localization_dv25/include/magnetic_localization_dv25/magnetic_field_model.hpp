#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>

#include "magnetic_localization_dv25/magnetic_field_lut.hpp"
#include "magnetic_localization_dv25/magnetic_map.hpp"

namespace magnetic_localization_dv25
{

class MagneticFieldModel
{
public:
  explicit MagneticFieldModel(const MagneticMap * map);

  void setSensorsBase(const std::vector<Eigen::Vector3d> & sensors_base);
  void setLut(std::shared_ptr<const MagneticFieldLUT> lut);
  Eigen::VectorXd predictStackedFieldT(const Eigen::Vector3d & p_map, const Eigen::Quaterniond & q_map_base) const;
  Eigen::Vector3d predictFieldTAt(const Eigen::Vector3d & p_sensor_map) const;

private:
  const MagneticMap * map_;
  std::shared_ptr<const MagneticFieldLUT> lut_;
  std::vector<Eigen::Vector3d> sensors_base_;
};

}  // namespace magnetic_localization_dv25
