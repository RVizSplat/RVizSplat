#include "gsplat_rviz_trials/ply_loader.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gsplat_rviz_trials
{
namespace
{

// ─────────────────────────────────────────────────────────
// PLY property descriptor
// ─────────────────────────────────────────────────────────

struct PropEntry
{
  std::string type;
  size_t byte_offset;   // offset in the binary per-vertex buffer
  size_t value_index;   // column index in the ASCII values array
};

// Returns the byte size of a PLY scalar type token.
static size_t plyTypeBytes(const std::string & t)
{
  if (t == "float"  || t == "float32" ||
      t == "int"    || t == "int32"   ||
      t == "uint"   || t == "uint32") return 4;
  if (t == "double" || t == "float64" ||
      t == "int64"  || t == "uint64") return 8;
  if (t == "short"  || t == "int16"  ||
      t == "ushort" || t == "uint16") return 2;
  if (t == "char"   || t == "int8"   ||
      t == "uchar"  || t == "uint8") return 1;
  return 4;  // safe default
}

// Reads a scalar from raw bytes and returns it as float.
static float readBytesAsFloat(const char * ptr, const std::string & type)
{
  if (type == "float" || type == "float32") {
    float v; std::memcpy(&v, ptr, 4); return v;
  }
  if (type == "double" || type == "float64") {
    double v; std::memcpy(&v, ptr, 8); return static_cast<float>(v);
  }
  // Integer types – cast via float reinterpret is wrong; do a proper cast.
  if (type == "uchar" || type == "uint8") {
    uint8_t v; std::memcpy(&v, ptr, 1); return static_cast<float>(v);
  }
  if (type == "char" || type == "int8") {
    int8_t v; std::memcpy(&v, ptr, 1); return static_cast<float>(v);
  }
  if (type == "ushort" || type == "uint16") {
    uint16_t v; std::memcpy(&v, ptr, 2); return static_cast<float>(v);
  }
  if (type == "short" || type == "int16") {
    int16_t v; std::memcpy(&v, ptr, 2); return static_cast<float>(v);
  }
  if (type == "int" || type == "int32") {
    int32_t v; std::memcpy(&v, ptr, 4); return static_cast<float>(v);
  }
  if (type == "uint" || type == "uint32") {
    uint32_t v; std::memcpy(&v, ptr, 4); return static_cast<float>(v);
  }
  float v; std::memcpy(&v, ptr, 4); return v;
}

// Helper: get a named property from a binary vertex buffer (default if absent).
static float binGet(
  const std::string & name,
  const std::unordered_map<std::string, PropEntry> & props,
  const char * vertex_ptr,
  float default_val = 0.0f)
{
  auto it = props.find(name);
  if (it == props.end()) return default_val;
  return readBytesAsFloat(vertex_ptr + it->second.byte_offset, it->second.type);
}

// Helper: get a named property from an ASCII values array (default if absent).
static float ascGet(
  const std::string & name,
  const std::unordered_map<std::string, PropEntry> & props,
  const std::vector<float> & values,
  float default_val = 0.0f)
{
  auto it = props.find(name);
  if (it == props.end() || it->second.value_index >= values.size()) return default_val;
  return values[it->second.value_index];
}

// ─────────────────────────────────────────────────────────
// 3DGS math helpers
// ─────────────────────────────────────────────────────────

static inline float sigmoid(float x)
{
  return 1.0f / (1.0f + std::exp(-x));
}

static inline float clamp01(float x)
{
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/**
 * @brief Compute GaussianData from raw 3DGS scalar values.
 *
 * @param fdc0/1/2   First-order SH DC coefficients (f_dc_0..2)
 * @param raw_opacity  Stored logit opacity; converted via sigmoid()
 * @param ls0/1/2    Log-scale values (scale_0..2); exponentiated to get world scale
 * @param qw/qx/qy/qz  Rotation quaternion (rot_0=w, rot_1=x, rot_2=y, rot_3=z)
 */
static GaussianData computeGaussian(
  float x, float y, float z,
  float fdc0, float fdc1, float fdc2,
  float raw_opacity,
  float ls0, float ls1, float ls2,
  float qw, float qx, float qy, float qz)
{
  constexpr float SH_C0 = 0.28209479177387814f;

  GaussianData g;

  // Position
  g.position = Ogre::Vector3(x, y, z);

  // Color: SH DC → linear RGB, opacity: logit → [0,1]
  g.color = Ogre::ColourValue(
    clamp01(0.5f + SH_C0 * fdc0),
    clamp01(0.5f + SH_C0 * fdc1),
    clamp01(0.5f + SH_C0 * fdc2),
    sigmoid(raw_opacity));

  // Scale (log-space → world-space)
  const float sx = std::exp(ls0);
  const float sy = std::exp(ls1);
  const float sz = std::exp(ls2);

  // Normalise quaternion
  float qnorm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (qnorm > 1e-6f) { qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm; }

  // Rotation matrix R from quaternion (w,x,y,z)
  const float R[3][3] = {
    {1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)},
    {    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)},
    {    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)}
  };

  // Σ = R · diag(s²) · Rᵀ  →  upper-triangular {v11,v12,v13,v22,v23,v33}
  const float s2[3] = {sx*sx, sy*sy, sz*sz};
  // Mapping (row,col) → covariance array index
  // (0,0)→0  (0,1)→1  (0,2)→2
  //          (1,1)→3  (1,2)→4
  //                   (2,2)→5
  const int cov_idx[3][3] = {{0, 1, 2}, {-1, 3, 4}, {-1, -1, 5}};
  for (int row = 0; row < 3; ++row) {
    for (int col = row; col < 3; ++col) {
      float val = 0.0f;
      for (int k = 0; k < 3; ++k) val += R[row][k] * s2[k] * R[col][k];
      g.covariance[cov_idx[row][col]] = val;
    }
  }

  return g;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────

std::vector<GaussianData> loadPly(const std::string & path, std::string & error_msg)
{
  error_msg.clear();

  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    error_msg = "Cannot open file: " + path;
    return {};
  }

  // ── Parse header ──────────────────────────────────────

  std::string line;
  std::getline(file, line);
  if (line.rfind("ply", 0) != 0) {
    error_msg = "Not a PLY file (missing 'ply' magic)";
    return {};
  }

  enum class Format { Unknown, BinaryLE, ASCII };
  Format format = Format::Unknown;
  int vertex_count = 0;
  std::vector<std::pair<std::string, std::string>> prop_list;  // (name, type) in order
  bool in_vertex_element = false;

  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line == "end_header") break;

    std::istringstream iss(line);
    std::string token;
    iss >> token;

    if (token == "format") {
      std::string fmt;
      iss >> fmt;
      if (fmt == "binary_little_endian") format = Format::BinaryLE;
      else if (fmt == "ascii")           format = Format::ASCII;
      else {
        error_msg = "Unsupported PLY format: " + fmt;
        return {};
      }
    } else if (token == "element") {
      std::string elem;
      iss >> elem;
      in_vertex_element = (elem == "vertex");
      if (in_vertex_element) iss >> vertex_count;
    } else if (token == "property" && in_vertex_element) {
      std::string type, name;
      iss >> type;
      if (type == "list") continue;  // skip list properties
      iss >> name;
      prop_list.push_back({name, type});
    }
  }

  if (format == Format::Unknown) {
    error_msg = "PLY format not specified in header";
    return {};
  }
  if (vertex_count <= 0) {
    error_msg = "PLY vertex count is 0 or missing";
    return {};
  }

  // Build property map with byte offsets and value indices
  std::unordered_map<std::string, PropEntry> props;
  size_t byte_offset = 0;
  for (size_t i = 0; i < prop_list.size(); ++i) {
    const auto & [name, type] = prop_list[i];
    props[name] = PropEntry{type, byte_offset, i};
    byte_offset += plyTypeBytes(type);
  }
  const size_t vertex_stride = byte_offset;

  if (!props.count("x") || !props.count("y") || !props.count("z")) {
    error_msg = "PLY is missing required position properties (x, y, z)";
    return {};
  }

  // Determine SH degree from the number of f_rest_* properties.
  // The PLY stores them channel-major: all R rest first, then G, then B.
  // Total rest count must be divisible by 3 (one group per channel).
  //   num_rest_per_channel:  0 → degree 0
  //                          3 → degree 1  (4 coeffs/channel = (1+1)²)
  //                          8 → degree 2  (9 coeffs/channel = (2+1)²)
  //                         15 → degree 3  (16 coeffs/channel = (3+1)²)
  int num_rest_total = 0;
  for (const auto & [name, type] : prop_list) {
    if (name.rfind("f_rest_", 0) == 0) ++num_rest_total;
  }
  const int num_rest_per_channel = num_rest_total / 3;
  const int total_per_channel    = 1 + num_rest_per_channel;
  int file_sh_degree = 0;
  if (total_per_channel >= 4)  file_sh_degree = 1;
  if (total_per_channel >= 9)  file_sh_degree = 2;
  if (total_per_channel >= 16) file_sh_degree = 3;

  std::vector<GaussianData> gaussians;
  gaussians.reserve(static_cast<size_t>(vertex_count));

  // ── Read vertex data ───────────────────────────────────

  if (format == Format::BinaryLE) {
    // Read the entire data block at once for performance.
    const size_t total_bytes = vertex_stride * static_cast<size_t>(vertex_count);
    std::vector<char> data(total_bytes);
    if (!file.read(data.data(), static_cast<std::streamsize>(total_bytes))) {
      error_msg = "Unexpected end of file while reading vertex data";
      return {};
    }

    for (int i = 0; i < vertex_count; ++i) {
      const char * vptr = data.data() + i * vertex_stride;
      GaussianData g = computeGaussian(
        binGet("x", props, vptr), binGet("y", props, vptr), binGet("z", props, vptr),
        binGet("f_dc_0", props, vptr), binGet("f_dc_1", props, vptr), binGet("f_dc_2", props, vptr),
        binGet("opacity", props, vptr, 10.0f),
        binGet("scale_0", props, vptr), binGet("scale_1", props, vptr), binGet("scale_2", props, vptr),
        binGet("rot_0",   props, vptr, 1.0f),
        binGet("rot_1",   props, vptr),
        binGet("rot_2",   props, vptr),
        binGet("rot_3",   props, vptr));

      // Fill spherical_harmonics array: coefficient-major (R,G,B per basis).
      // DC term (basis 0).
      std::fill(g.sh, g.sh + 48, 0.0f);
      g.sh_degree = file_sh_degree;
      g.sh[0] = binGet("f_dc_0", props, vptr);
      g.sh[1] = binGet("f_dc_1", props, vptr);
      g.sh[2] = binGet("f_dc_2", props, vptr);
      // Higher-order: PLY is channel-major → reorder to coefficient-major.
      for (int ci = 0; ci < num_rest_per_channel; ++ci) {
        g.sh[3*(ci+1) + 0] = binGet("f_rest_" + std::to_string(ci),                          props, vptr);
        g.sh[3*(ci+1) + 1] = binGet("f_rest_" + std::to_string(ci + num_rest_per_channel),   props, vptr);
        g.sh[3*(ci+1) + 2] = binGet("f_rest_" + std::to_string(ci + 2*num_rest_per_channel), props, vptr);
      }

      gaussians.push_back(g);
    }
  } else {
    // ASCII: one vertex per line, space-separated floats
    for (int i = 0; i < vertex_count; ++i) {
      if (!std::getline(file, line)) {
        error_msg = "Unexpected end of file at ASCII vertex " + std::to_string(i);
        return {};
      }
      if (!line.empty() && line.back() == '\r') line.pop_back();

      std::istringstream vss(line);
      std::vector<float> values;
      float v;
      while (vss >> v) values.push_back(v);

      if (values.size() < prop_list.size()) {
        error_msg = "Too few values at ASCII vertex " + std::to_string(i);
        return {};
      }

      GaussianData g = computeGaussian(
        ascGet("x", props, values), ascGet("y", props, values), ascGet("z", props, values),
        ascGet("f_dc_0", props, values), ascGet("f_dc_1", props, values), ascGet("f_dc_2", props, values),
        ascGet("opacity", props, values, 10.0f),
        ascGet("scale_0", props, values), ascGet("scale_1", props, values), ascGet("scale_2", props, values),
        ascGet("rot_0",   props, values, 1.0f),
        ascGet("rot_1",   props, values),
        ascGet("rot_2",   props, values),
        ascGet("rot_3",   props, values));

      std::fill(g.sh, g.sh + 48, 0.0f);
      g.sh_degree = file_sh_degree;
      g.sh[0] = ascGet("f_dc_0", props, values);
      g.sh[1] = ascGet("f_dc_1", props, values);
      g.sh[2] = ascGet("f_dc_2", props, values);
      for (int ci = 0; ci < num_rest_per_channel; ++ci) {
        g.sh[3*(ci+1) + 0] = ascGet("f_rest_" + std::to_string(ci),                          props, values);
        g.sh[3*(ci+1) + 1] = ascGet("f_rest_" + std::to_string(ci + num_rest_per_channel),   props, values);
        g.sh[3*(ci+1) + 2] = ascGet("f_rest_" + std::to_string(ci + 2*num_rest_per_channel), props, values);
      }

      gaussians.push_back(g);
    }
  }

  return gaussians;
}

}  // namespace gsplat_rviz_trials
