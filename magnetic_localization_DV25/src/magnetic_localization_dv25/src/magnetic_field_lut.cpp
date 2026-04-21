#include "magnetic_localization_dv25/magnetic_field_lut.hpp"

#include <fstream>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <cstring>

namespace magnetic_localization_dv25
{

namespace
{

uint16_t readU16(std::istream & is)
{
  uint8_t b0 = 0, b1 = 0;
  is.read(reinterpret_cast<char *>(&b0), 1);
  is.read(reinterpret_cast<char *>(&b1), 1);
  return static_cast<uint16_t>(b0 | (static_cast<uint16_t>(b1) << 8));
}

uint32_t readU32(std::istream & is)
{
  uint8_t b[4]{0, 0, 0, 0};
  is.read(reinterpret_cast<char *>(b), 4);
  return static_cast<uint32_t>(b[0]) |
         (static_cast<uint32_t>(b[1]) << 8) |
         (static_cast<uint32_t>(b[2]) << 16) |
         (static_cast<uint32_t>(b[3]) << 24);
}

std::unordered_map<std::string, std::vector<uint8_t>> readZipStored(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("npz open failed");
  }

  std::unordered_map<std::string, std::vector<uint8_t>> files;
  while (true) {
    const auto pos = f.tellg();
    if (!f) {
      break;
    }
    uint32_t sig = 0;
    try {
      sig = readU32(f);
    } catch (...) {
      break;
    }
    if (!f || sig != 0x04034b50) {
      f.clear();
      f.seekg(pos);
      break;
    }

    (void)readU16(f);
    const uint16_t flags = readU16(f);
    const uint16_t method = readU16(f);
    (void)readU16(f);
    (void)readU16(f);
    (void)readU32(f);
    uint32_t comp_size = readU32(f);
    uint32_t uncomp_size = readU32(f);
    const uint16_t name_len = readU16(f);
    const uint16_t extra_len = readU16(f);

    std::string name(name_len, '\0');
    f.read(name.data(), name_len);
    if (extra_len > 0) {
      f.seekg(extra_len, std::ios::cur);
    }

    if (method != 0) {
      throw std::runtime_error("npz compression not supported; use np.savez (not compressed)");
    }

    std::vector<uint8_t> data;
    if ((flags & 0x0008) != 0) {
      data.reserve(uncomp_size);
      std::vector<uint8_t> tmp;
      tmp.resize(comp_size);
      f.read(reinterpret_cast<char *>(tmp.data()), comp_size);
      data = std::move(tmp);

      const uint32_t dd_sig = readU32(f);
      if (dd_sig == 0x08074b50) {
        (void)readU32(f);
        comp_size = readU32(f);
        uncomp_size = readU32(f);
      } else {
        (void)dd_sig;
        (void)readU32(f);
        (void)readU32(f);
      }
    } else {
      data.resize(comp_size);
      f.read(reinterpret_cast<char *>(data.data()), comp_size);
    }

    if (uncomp_size != 0 && data.size() != uncomp_size) {
      data.resize(uncomp_size);
    }

    files[name] = std::move(data);
  }

  return files;
}

struct NpyArray
{
  std::vector<double> data;
  std::vector<size_t> shape;
};

NpyArray parseNpyF8(const std::vector<uint8_t> & bytes)
{
  if (bytes.size() < 16) {
    throw std::runtime_error("npy too small");
  }
  if (!(bytes[0] == 0x93 && bytes[1] == 'N' && bytes[2] == 'U' && bytes[3] == 'M' && bytes[4] == 'P' &&
        bytes[5] == 'Y'))
  {
    throw std::runtime_error("npy magic mismatch");
  }

  const uint8_t major = bytes[6];
  const uint8_t minor = bytes[7];
  size_t header_len = 0;
  size_t offset = 8;
  if (major == 1) {
    header_len = static_cast<size_t>(bytes[offset]) | (static_cast<size_t>(bytes[offset + 1]) << 8);
    offset += 2;
  } else if (major == 2) {
    header_len = static_cast<size_t>(bytes[offset]) |
                 (static_cast<size_t>(bytes[offset + 1]) << 8) |
                 (static_cast<size_t>(bytes[offset + 2]) << 16) |
                 (static_cast<size_t>(bytes[offset + 3]) << 24);
    offset += 4;
  } else {
    throw std::runtime_error("npy version not supported");
  }

  if (offset + header_len > bytes.size()) {
    throw std::runtime_error("npy header overflow");
  }
  std::string header(reinterpret_cast<const char *>(&bytes[offset]), header_len);
  offset += header_len;

  if (header.find("'descr': '<f8'") == std::string::npos && header.find("\"descr\": \"<f8\"") == std::string::npos) {
    throw std::runtime_error("npy dtype not supported");
  }
  if (header.find("fortran_order") != std::string::npos && header.find("True") != std::string::npos) {
    throw std::runtime_error("npy fortran order not supported");
  }

  const auto shape_pos = header.find("shape");
  if (shape_pos == std::string::npos) {
    throw std::runtime_error("npy shape missing");
  }
  const auto lpar = header.find('(', shape_pos);
  const auto rpar = header.find(')', shape_pos);
  if (lpar == std::string::npos || rpar == std::string::npos || rpar <= lpar) {
    throw std::runtime_error("npy shape parse failed");
  }
  const std::string shape_str = header.substr(lpar + 1, rpar - lpar - 1);

  std::vector<size_t> shape;
  size_t start = 0;
  while (start < shape_str.size()) {
    const auto comma = shape_str.find(',', start);
    const auto token = shape_str.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    const std::string trimmed = [&]() {
      size_t a = token.find_first_not_of(" \t");
      size_t b = token.find_last_not_of(" \t");
      if (a == std::string::npos) {
        return std::string();
      }
      return token.substr(a, b - a + 1);
    }();
    if (!trimmed.empty()) {
      shape.push_back(static_cast<size_t>(std::stoul(trimmed)));
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  if (shape.empty()) {
    throw std::runtime_error("npy shape empty");
  }

  size_t count = 1;
  for (auto s : shape) {
    count *= s;
  }
  const size_t need = count * sizeof(double);
  if (offset + need > bytes.size()) {
    throw std::runtime_error("npy data overflow");
  }

  NpyArray arr;
  arr.shape = shape;
  arr.data.resize(count);
  std::memcpy(arr.data.data(), &bytes[offset], need);
  return arr;
}

}  // namespace

bool MagneticFieldLUT::loadNpzStored(const std::string & npz_path)
{
  x_.clear();
  y_.clear();
  z_.clear();
  B_.clear();
  nx_ = ny_ = nz_ = 0;

  const auto files = readZipStored(npz_path);
  const auto itx = files.find("x.npy");
  const auto ity = files.find("y.npy");
  const auto itz = files.find("z.npy");
  const auto itB = files.find("B.npy");
  if (itx == files.end() || ity == files.end() || itz == files.end() || itB == files.end()) {
    return false;
  }

  const auto ax = parseNpyF8(itx->second);
  const auto ay = parseNpyF8(ity->second);
  const auto az = parseNpyF8(itz->second);
  const auto aB = parseNpyF8(itB->second);

  if (ax.shape.size() != 1 || ay.shape.size() != 1 || az.shape.size() != 1) {
    return false;
  }
  if (aB.shape.size() != 4 || aB.shape[3] != 3) {
    return false;
  }

  nx_ = static_cast<int>(ax.shape[0]);
  ny_ = static_cast<int>(ay.shape[0]);
  nz_ = static_cast<int>(az.shape[0]);
  if (aB.shape[0] != static_cast<size_t>(nx_) || aB.shape[1] != static_cast<size_t>(ny_) || aB.shape[2] != static_cast<size_t>(nz_)) {
    return false;
  }

  x_ = ax.data;
  y_ = ay.data;
  z_ = az.data;
  B_ = aB.data;
  return valid();
}

bool MagneticFieldLUT::valid() const
{
  return nx_ > 1 && ny_ > 1 && nz_ > 1 && static_cast<int>(B_.size()) == nx_ * ny_ * nz_ * 3;
}

bool MagneticFieldLUT::isInside(const Eigen::Vector3d & p_map) const
{
  if (!valid()) return false;
  return p_map.x() >= x_.front() && p_map.x() <= x_.back() &&
         p_map.y() >= y_.front() && p_map.y() <= y_.back() &&
         p_map.z() >= z_.front() && p_map.z() <= z_.back();
}

int MagneticFieldLUT::clampIndex(int idx, int max) const
{
  if (idx < 0) {
    return 0;
  }
  if (idx > max) {
    return max;
  }
  return idx;
}

double MagneticFieldLUT::atB(int ix, int iy, int iz, int c) const
{
  // 优化索引计算：确保三线性插值时的内存访问具有良好的局部性
  // 布局为 [ix][iy][iz][c]，其中 c 是最内层，iz 是次内层
  const size_t idx = ((static_cast<size_t>(ix) * ny_ + iy) * nz_ + iz) * 3 + c;
  return B_[idx];
}

Eigen::Vector3d MagneticFieldLUT::interpolateFieldT(const Eigen::Vector3d & p_map) const
{
  if (!valid()) {
    return Eigen::Vector3d::Zero();
  }

  // 快速查找索引（假设均匀网格以获得 O(1) 性能，否则回退到二分查找）
  const auto findIdx = [](const std::vector<double> & arr, double v, int n) -> int {
    if (n < 2) return 0;
    const double start = arr.front();
    const double end = arr.back();
    const double step = (end - start) / (n - 1);
    
    // 均匀网格优化
    if (std::abs(step) > 1e-12) {
      int idx = static_cast<int>((v - start) / step);
      if (idx < 0) return 0;
      if (idx >= n - 1) return n - 2;
      return idx;
    }

    // 回退到二分查找
    int lo = 0;
    int hi = n - 2;
    while (lo <= hi) {
      const int mid = (lo + hi) / 2;
      if (arr[static_cast<size_t>(mid)] <= v && v < arr[static_cast<size_t>(mid + 1)]) {
        return mid;
      }
      if (v < arr[static_cast<size_t>(mid)]) {
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }
    return n - 2;
  };

  const int ix0 = findIdx(x_, p_map.x(), nx_);
  const int iy0 = findIdx(y_, p_map.y(), ny_);
  const int iz0 = findIdx(z_, p_map.z(), nz_);

  const int ix1 = ix0 + 1;
  const int iy1 = iy0 + 1;
  const int iz1 = iz0 + 1;

  const double x0 = x_[static_cast<size_t>(ix0)];
  const double x1 = x_[static_cast<size_t>(ix1)];
  const double y0 = y_[static_cast<size_t>(iy0)];
  const double y1 = y_[static_cast<size_t>(iy1)];
  const double z0 = z_[static_cast<size_t>(iz0)];
  const double z1 = z_[static_cast<size_t>(iz1)];

  const double tx = (x1 - x0) > 1e-12 ? (p_map.x() - x0) / (x1 - x0) : 0.0;
  const double ty = (y1 - y0) > 1e-12 ? (p_map.y() - y0) / (y1 - y0) : 0.0;
  const double tz = (z1 - z0) > 1e-12 ? (p_map.z() - z0) / (z1 - z0) : 0.0;

  Eigen::Vector3d out;
  // 展开循环并预计算权重以提高吞吐量
  const double w000 = (1.0 - tx) * (1.0 - ty) * (1.0 - tz);
  const double w100 = tx * (1.0 - ty) * (1.0 - tz);
  const double w010 = (1.0 - tx) * ty * (1.0 - tz);
  const double w110 = tx * ty * (1.0 - tz);
  const double w001 = (1.0 - tx) * (1.0 - ty) * tz;
  const double w101 = tx * (1.0 - ty) * tz;
  const double w011 = (1.0 - tx) * ty * tz;
  const double w111 = tx * ty * tz;

  for (int c = 0; c < 3; ++c) {
    out[c] = w000 * atB(ix0, iy0, iz0, c) +
             w100 * atB(ix1, iy0, iz0, c) +
             w010 * atB(ix0, iy1, iz0, c) +
             w110 * atB(ix1, iy1, iz0, c) +
             w001 * atB(ix0, iy0, iz1, c) +
             w101 * atB(ix1, iy0, iz1, c) +
             w011 * atB(ix0, iy1, iz1, c) +
             w111 * atB(ix1, iy1, iz1, c);
  }
  return out;
}

}  // namespace magnetic_localization_dv25
