#include "geometry/tube_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QByteArray>
#include <QQuaternion>
#include <QVector3D>

namespace robotarm {

namespace {

constexpr float kPi = 3.14159265358979323846f;

struct Vertex {
  float px, py, pz;
  float nx, ny, nz;
  float u, v;
};

// Uniform Catmull-Rom between p1 and p2 (p0/p3 are the neighbours).
QVector3D catmull_rom(const QVector3D &p0, const QVector3D &p1, const QVector3D &p2, const QVector3D &p3, float t) {
  const float t2 = t * t;
  const float t3 = t2 * t;
  return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                 (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

} // namespace

TubeGeometry::TubeGeometry(QQuick3DObject *parent) : QQuick3DGeometry(parent) { rebuild(); }

void TubeGeometry::setPoints(const QVariantList &points) {
  if (points == _points) {
    return;
  }
  _points = points;
  emit pointsChanged();
  rebuild();
}

void TubeGeometry::setRadius(float radius) {
  if (qFuzzyCompare(radius, _radius)) {
    return;
  }
  _radius = radius;
  emit radiusChanged();
  rebuild();
}

void TubeGeometry::setRadialSegments(int segments) {
  segments = std::clamp(segments, 3, 64);
  if (segments == _radialSegments) {
    return;
  }
  _radialSegments = segments;
  emit radialSegmentsChanged();
  rebuild();
}

void TubeGeometry::setSamplesPerSpan(int samples) {
  samples = std::clamp(samples, 1, 64);
  if (samples == _samplesPerSpan) {
    return;
  }
  _samplesPerSpan = samples;
  emit samplesPerSpanChanged();
  rebuild();
}

void TubeGeometry::setClosed(bool closed) {
  if (closed == _closed) {
    return;
  }
  _closed = closed;
  emit closedChanged();
  rebuild();
}

void TubeGeometry::rebuild() {
  std::vector<QVector3D> control;
  control.reserve(static_cast<size_t>(_points.size()));
  for (const QVariant &point : _points) {
    control.push_back(point.value<QVector3D>());
  }

  clear();
  if (control.size() < 2) {
    update();
    return;
  }

  // 1. Sample the spline.
  const size_t n = control.size();
  const size_t spans = _closed ? n : n - 1;
  auto at = [&](long i) -> QVector3D {
    if (_closed) {
      return control[static_cast<size_t>((i % static_cast<long>(n) + static_cast<long>(n)) % static_cast<long>(n))];
    }
    // Open curve: mirror the end points so the tangent there follows the
    // first/last segment.
    if (i < 0) {
      return 2.0f * control.front() - control[1];
    }
    if (i >= static_cast<long>(n)) {
      return 2.0f * control.back() - control[n - 2];
    }
    return control[static_cast<size_t>(i)];
  };

  std::vector<QVector3D> path;
  for (size_t s = 0; s < spans; s++) {
    const long i = static_cast<long>(s);
    for (int k = 0; k < _samplesPerSpan; k++) {
      const float t = static_cast<float>(k) / static_cast<float>(_samplesPerSpan);
      path.push_back(catmull_rom(at(i - 1), at(i), at(i + 1), at(i + 2), t));
    }
  }
  if (_closed) {
    path.push_back(path.front());
  } else {
    path.push_back(control.back());
  }

  // 2. Tangents and a parallel-transport frame along the path.
  const size_t rings = path.size();
  std::vector<QVector3D> tangents(rings);
  for (size_t i = 0; i < rings; i++) {
    const QVector3D prev = path[i == 0 ? 0 : i - 1];
    const QVector3D next = path[i + 1 < rings ? i + 1 : rings - 1];
    QVector3D t = next - prev;
    if (t.lengthSquared() < 1e-14f) {
      t = QVector3D(1, 0, 0);
    }
    tangents[i] = t.normalized();
  }

  std::vector<QVector3D> normals(rings);
  {
    const QVector3D t0 = tangents[0];
    QVector3D helper = std::abs(t0.y()) < 0.9f ? QVector3D(0, 1, 0) : QVector3D(1, 0, 0);
    normals[0] = QVector3D::crossProduct(t0, QVector3D::crossProduct(helper, t0)).normalized();
    for (size_t i = 1; i < rings; i++) {
      const QQuaternion turn = QQuaternion::rotationTo(tangents[i - 1], tangents[i]);
      QVector3D normal = turn.rotatedVector(normals[i - 1]);
      normal = (normal - QVector3D::dotProduct(normal, tangents[i]) * tangents[i]).normalized();
      normals[i] = normal;
    }
  }

  // 3. Rings of vertices.
  std::vector<Vertex> vertices;
  std::vector<quint32> indices;
  const int radial = _radialSegments;
  float distance = 0.0f;
  for (size_t i = 0; i < rings; i++) {
    if (i > 0) {
      distance += (path[i] - path[i - 1]).length();
    }
    const QVector3D normal = normals[i];
    const QVector3D binormal = QVector3D::crossProduct(tangents[i], normal);
    for (int r = 0; r <= radial; r++) {
      const float angle = 2.0f * kPi * static_cast<float>(r) / static_cast<float>(radial);
      const QVector3D direction = std::cos(angle) * normal + std::sin(angle) * binormal;
      const QVector3D position = path[i] + _radius * direction;
      vertices.push_back({position.x(), position.y(), position.z(), direction.x(), direction.y(), direction.z(),
                          static_cast<float>(r) / static_cast<float>(radial), distance});
    }
  }
  const quint32 ring_size = static_cast<quint32>(radial + 1);
  for (quint32 i = 0; i + 1 < rings; i++) {
    for (quint32 r = 0; r < static_cast<quint32>(radial); r++) {
      const quint32 a = i * ring_size + r;
      const quint32 b = a + 1;
      const quint32 c = a + ring_size;
      const quint32 d = c + 1;
      // direction = cos*N + sin*B with B = T x N, so increasing r runs
      // counter-clockwise around T; (a, b, d) and (a, d, c) then have
      // their cross products pointing out of the tube.
      indices.insert(indices.end(), {a, b, d, a, d, c});
    }
  }

  // 4. Flat end caps for an open tube.
  if (!_closed) {
    for (int end = 0; end < 2; end++) {
      const size_t ring = end == 0 ? 0 : rings - 1;
      const QVector3D outward = end == 0 ? -tangents[ring] : tangents[ring];
      const quint32 center = static_cast<quint32>(vertices.size());
      vertices.push_back({path[ring].x(), path[ring].y(), path[ring].z(), outward.x(), outward.y(), outward.z(), 0.5f,
                          0.5f});
      const quint32 first = static_cast<quint32>(vertices.size());
      for (int r = 0; r <= radial; r++) {
        const Vertex &rim = vertices[ring * ring_size + static_cast<size_t>(r)];
        vertices.push_back({rim.px, rim.py, rim.pz, outward.x(), outward.y(), outward.z(), 0.0f, 0.0f});
      }
      for (quint32 r = 0; r < static_cast<quint32>(radial); r++) {
        if (end == 0) {
          indices.insert(indices.end(), {center, first + r + 1, first + r});
        } else {
          indices.insert(indices.end(), {center, first + r, first + r + 1});
        }
      }
    }
  }

  QVector3D minimum(1e9f, 1e9f, 1e9f);
  QVector3D maximum(-1e9f, -1e9f, -1e9f);
  for (const Vertex &vertex : vertices) {
    minimum = QVector3D(std::min(minimum.x(), vertex.px), std::min(minimum.y(), vertex.py),
                        std::min(minimum.z(), vertex.pz));
    maximum = QVector3D(std::max(maximum.x(), vertex.px), std::max(maximum.y(), vertex.py),
                        std::max(maximum.z(), vertex.pz));
  }

  setStride(sizeof(Vertex));
  setVertexData(QByteArray(reinterpret_cast<const char *>(vertices.data()),
                           static_cast<qsizetype>(vertices.size() * sizeof(Vertex))));
  setIndexData(QByteArray(reinterpret_cast<const char *>(indices.data()),
                          static_cast<qsizetype>(indices.size() * sizeof(quint32))));
  setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
  addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);
  addAttribute(QQuick3DGeometry::Attribute::NormalSemantic, 3 * sizeof(float), QQuick3DGeometry::Attribute::F32Type);
  addAttribute(QQuick3DGeometry::Attribute::TexCoord0Semantic, 6 * sizeof(float),
               QQuick3DGeometry::Attribute::F32Type);
  addAttribute(QQuick3DGeometry::Attribute::IndexSemantic, 0, QQuick3DGeometry::Attribute::U32Type);
  setBounds(minimum, maximum);
  update();
}

} // namespace robotarm
