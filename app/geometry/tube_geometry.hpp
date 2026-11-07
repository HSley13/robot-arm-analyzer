#pragma once

#include <QQuick3DGeometry>
#include <QtQml/qqmlregistration.h>
#include <QVariantList>

namespace robotarm {

// A round tube swept along a smooth curve through a list of points - the
// cables, wire bundles and bent support rods running down the forearm.
//
//   Model { geometry: TubeGeometry { points: [Qt.vector3d(0,0,0), Qt.vector3d(0.1,0.02,0)]; radius: 0.002 } }
//
// The curve is a centripetal-style Catmull-Rom spline through every point
// (so it passes exactly through what you give it, with no overshoot
// tuning needed), sampled `samplesPerSpan` times between neighbours. The
// cross-section circle is carried along with a parallel-transport frame -
// each ring is rotated from the previous one by the smallest rotation
// that follows the curve's turn - so the tube never twists or pinches the
// way a fixed "up" vector does when the curve points straight at it.
//
// `closed: true` joins the last point back to the first (a ring).
class TubeGeometry : public QQuick3DGeometry {
  Q_OBJECT
  QML_ELEMENT // `import Humanoid.Geometry`, then `geometry: TubeGeometry { ... }`
  Q_PROPERTY(QVariantList points READ points WRITE setPoints NOTIFY pointsChanged)
  Q_PROPERTY(float radius READ radius WRITE setRadius NOTIFY radiusChanged)
  Q_PROPERTY(int radialSegments READ radialSegments WRITE setRadialSegments NOTIFY radialSegmentsChanged)
  Q_PROPERTY(int samplesPerSpan READ samplesPerSpan WRITE setSamplesPerSpan NOTIFY samplesPerSpanChanged)
  Q_PROPERTY(bool closed READ closed WRITE setClosed NOTIFY closedChanged)

public:
  explicit TubeGeometry(QQuick3DObject *parent = nullptr);

  QVariantList points() const { return _points; }
  void setPoints(const QVariantList &points);
  float radius() const { return _radius; }
  void setRadius(float radius);
  int radialSegments() const { return _radialSegments; }
  void setRadialSegments(int segments);
  int samplesPerSpan() const { return _samplesPerSpan; }
  void setSamplesPerSpan(int samples);
  bool closed() const { return _closed; }
  void setClosed(bool closed);

signals:
  void pointsChanged();
  void radiusChanged();
  void radialSegmentsChanged();
  void samplesPerSpanChanged();
  void closedChanged();

private:
  void rebuild();

  QVariantList _points;
  float _radius = 0.002f;
  int _radialSegments = 10;
  int _samplesPerSpan = 8;
  bool _closed = false;
};

} // namespace robotarm
