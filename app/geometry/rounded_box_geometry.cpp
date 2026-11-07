#include "geometry/rounded_box_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <QByteArray>

namespace robotarm {

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Sample coordinates along one axis of a face, from -half to +half: equal
// angle steps through each rounded band, evenly spaced across the flat
// middle (see the class comment for why).
std::vector<float> axis_samples(float half, float radius, int round_segments) {
  const float flat_half = std::max(0.0f, half - radius);
  std::vector<float> samples;
  for (int i = round_segments; i >= 1; i--) {
    const float angle = (kPi / 4.0f) * static_cast<float>(i) / static_cast<float>(round_segments);
    samples.push_back(-flat_half - radius * std::tan(angle));
  }
  const int flat_segments = flat_half > 1e-6f ? 2 : 0;
  for (int i = 0; i <= flat_segments; i++) {
    if (flat_segments == 0) {
      samples.push_back(0.0f);
      break;
    }
    samples.push_back(-flat_half + 2.0f * flat_half * static_cast<float>(i) / static_cast<float>(flat_segments));
  }
  for (int i = 1; i <= round_segments; i++) {
    const float angle = (kPi / 4.0f) * static_cast<float>(i) / static_cast<float>(round_segments);
    samples.push_back(flat_half + radius * std::tan(angle));
  }
  // The outermost samples are exactly at +-(flat_half + radius) == +-half
  // (tan(45deg) == 1), so faces meet exactly at the cube's edges.
  return samples;
}

struct Vertex {
  float px, py, pz;
  float nx, ny, nz;
  float u, v;
};

} // namespace

RoundedBoxGeometry::RoundedBoxGeometry(QQuick3DObject *parent) : QQuick3DGeometry(parent) { rebuild(); }

void RoundedBoxGeometry::setSize(const QVector3D &size) {
  if (size == _size) {
    return;
  }
  _size = size;
  emit sizeChanged();
  rebuild();
}

void RoundedBoxGeometry::setRadius(float radius) {
  if (qFuzzyCompare(radius, _radius)) {
    return;
  }
  _radius = radius;
  emit radiusChanged();
  rebuild();
}

void RoundedBoxGeometry::setTaper(float taper) {
  if (qFuzzyCompare(taper, _taper)) {
    return;
  }
  _taper = taper;
  emit taperChanged();
  rebuild();
}

void RoundedBoxGeometry::setRoundSegments(int segments) {
  segments = std::clamp(segments, 1, 16);
  if (segments == _roundSegments) {
    return;
  }
  _roundSegments = segments;
  emit roundSegmentsChanged();
  rebuild();
}

void RoundedBoxGeometry::rebuild() {
  const std::array<float, 3> half = {std::max(1e-5f, _size.x() * 0.5f), std::max(1e-5f, _size.y() * 0.5f),
                                     std::max(1e-5f, _size.z() * 0.5f)};
  // Never round more than the thinnest half-extent allows.
  const float radius = std::clamp(_radius, 0.0f, std::min({half[0], half[1], half[2]}));
  const std::array<float, 3> inner = {half[0] - radius, half[1] - radius, half[2] - radius};

  std::array<std::vector<float>, 3> samples = {axis_samples(half[0], radius, _roundSegments),
                                               axis_samples(half[1], radius, _roundSegments),
                                               axis_samples(half[2], radius, _roundSegments)};

  std::vector<Vertex> vertices;
  std::vector<quint32> indices;

  // Six faces: `axis` is the face normal's axis, `sign` its direction;
  // (a, b) are the two in-face axes.
  for (int axis = 0; axis < 3; axis++) {
    for (int sign = -1; sign <= 1; sign += 2) {
      const int a = (axis + 1) % 3;
      const int b = (axis + 2) % 3;
      const std::vector<float> &sa = samples[static_cast<size_t>(a)];
      const std::vector<float> &sb = samples[static_cast<size_t>(b)];
      const quint32 base = static_cast<quint32>(vertices.size());

      for (size_t j = 0; j < sb.size(); j++) {
        for (size_t i = 0; i < sa.size(); i++) {
          std::array<float, 3> q{};
          q[static_cast<size_t>(axis)] = static_cast<float>(sign) * half[static_cast<size_t>(axis)];
          q[static_cast<size_t>(a)] = sa[i];
          q[static_cast<size_t>(b)] = sb[j];

          std::array<float, 3> c{};
          std::array<float, 3> n{};
          float length = 0.0f;
          for (size_t k = 0; k < 3; k++) {
            c[k] = std::clamp(q[k], -inner[k], inner[k]);
            n[k] = q[k] - c[k];
            length += n[k] * n[k];
          }
          length = std::sqrt(length);
          if (length < 1e-9f) {
            n = {0.0f, 0.0f, 0.0f};
            n[static_cast<size_t>(axis)] = static_cast<float>(sign);
          } else {
            for (float &component : n) {
              component /= length;
            }
          }

          Vertex vertex{};
          float p[3];
          for (size_t k = 0; k < 3; k++) {
            p[k] = c[k] + n[k] * radius;
          }

          // Linear taper of the X/Z cross-section along Y. The normal is
          // corrected by the inverse of the local scale so shading follows
          // the narrowed shape.
          const float along = (p[1] + half[1]) / (2.0f * half[1]); // 0 at -Y, 1 at +Y
          const float scale = 1.0f + (_taper - 1.0f) * along;
          p[0] *= scale;
          p[2] *= scale;
          float nx = n[0] / scale;
          float ny = n[1];
          float nz = n[2] / scale;
          const float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
          if (nlen > 1e-9f) {
            nx /= nlen;
            ny /= nlen;
            nz /= nlen;
          }

          vertex.px = p[0];
          vertex.py = p[1];
          vertex.pz = p[2];
          vertex.nx = nx;
          vertex.ny = ny;
          vertex.nz = nz;
          vertex.u = static_cast<float>(i) / static_cast<float>(sa.size() - 1);
          vertex.v = static_cast<float>(j) / static_cast<float>(sb.size() - 1);
          vertices.push_back(vertex);
        }
      }

      const quint32 stride_a = static_cast<quint32>(sa.size());
      for (size_t j = 0; j + 1 < sb.size(); j++) {
        for (size_t i = 0; i + 1 < sa.size(); i++) {
          const quint32 i00 = base + static_cast<quint32>(j) * stride_a + static_cast<quint32>(i);
          const quint32 i10 = i00 + 1;
          const quint32 i01 = i00 + stride_a;
          const quint32 i11 = i01 + 1;
          // a x b == +axis, so (i00, i10, i11) winds counter-clockwise
          // seen from outside on a +axis face; flip it for the -axis face.
          if (sign > 0) {
            indices.insert(indices.end(), {i00, i10, i11, i00, i11, i01});
          } else {
            indices.insert(indices.end(), {i00, i11, i10, i00, i01, i11});
          }
        }
      }
    }
  }

  QByteArray vertex_bytes(reinterpret_cast<const char *>(vertices.data()),
                          static_cast<qsizetype>(vertices.size() * sizeof(Vertex)));
  QByteArray index_bytes(reinterpret_cast<const char *>(indices.data()),
                         static_cast<qsizetype>(indices.size() * sizeof(quint32)));

  clear();
  setStride(sizeof(Vertex));
  setVertexData(vertex_bytes);
  setIndexData(index_bytes);
  setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
  addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);
  addAttribute(QQuick3DGeometry::Attribute::NormalSemantic, 3 * sizeof(float), QQuick3DGeometry::Attribute::F32Type);
  addAttribute(QQuick3DGeometry::Attribute::TexCoord0Semantic, 6 * sizeof(float),
               QQuick3DGeometry::Attribute::F32Type);
  addAttribute(QQuick3DGeometry::Attribute::IndexSemantic, 0, QQuick3DGeometry::Attribute::U32Type);
  setBounds(QVector3D(-half[0], -half[1], -half[2]), QVector3D(half[0], half[1], half[2]));
  update();
}

} // namespace robotarm
