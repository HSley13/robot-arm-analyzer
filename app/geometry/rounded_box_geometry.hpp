#pragma once

#include <QQuick3DGeometry>
#include <QtQml/qqmlregistration.h>
#include <QVector3D>

namespace robotarm {

// A box with every edge and corner rounded off, generated procedurally so
// QML can use it like a built-in mesh:
//
//   Model { geometry: RoundedBoxGeometry { size: Qt.vector3d(0.02, 0.05, 0.018); radius: 0.005 } }
//
// Qt Quick 3D's built-in primitives (#Cube, #Cylinder, #Sphere) have hard
// edges or are perfectly round - neither reads as a molded robot shell.
// A phalanx, a palm cover or a forearm housing is essentially a rounded
// box, so this one class covers most of the humanoid's body.
//
// Units are whatever the scene uses (meters here) - the geometry is built
// at its final size, so the Model needs no scale.
//
// How the rounding works: start from a cube whose faces are subdivided
// into a grid, then push every grid vertex onto the surface of
// "inner box (size - 2*radius) grown by radius": clamp the point to the
// inner box, and move `radius` along the direction from the clamped point
// to the original one. Points over a flat face move straight out; points
// near an edge or corner land on a cylinder or sphere of that radius - and
// that same direction is the exact surface normal, so lighting is smooth.
// The grid is spaced by equal ANGLE steps inside the rounded band
// (offset = radius * tan(angle)), so the curve gets the vertices and the
// flat middle doesn't waste any.
//
// `taper` scales the X/Z cross-section linearly along Y: 1 at -Y, `taper`
// at +Y - a finger segment or a forearm that narrows toward its far end.
class RoundedBoxGeometry : public QQuick3DGeometry {
  Q_OBJECT
  QML_ELEMENT // `import Humanoid.Geometry`, then `geometry: RoundedBoxGeometry { ... }`
  Q_PROPERTY(QVector3D size READ size WRITE setSize NOTIFY sizeChanged)
  Q_PROPERTY(float radius READ radius WRITE setRadius NOTIFY radiusChanged)
  Q_PROPERTY(float taper READ taper WRITE setTaper NOTIFY taperChanged)
  Q_PROPERTY(int roundSegments READ roundSegments WRITE setRoundSegments NOTIFY roundSegmentsChanged)

public:
  explicit RoundedBoxGeometry(QQuick3DObject *parent = nullptr);

  QVector3D size() const { return _size; }
  void setSize(const QVector3D &size);
  float radius() const { return _radius; }
  void setRadius(float radius);
  float taper() const { return _taper; }
  void setTaper(float taper);
  int roundSegments() const { return _roundSegments; }
  void setRoundSegments(int segments);

signals:
  void sizeChanged();
  void radiusChanged();
  void taperChanged();
  void roundSegmentsChanged();

private:
  void rebuild();

  QVector3D _size{1.0f, 1.0f, 1.0f};
  float _radius = 0.1f;
  float _taper = 1.0f;
  int _roundSegments = 4; // per 45 degrees of each rounded edge
};

} // namespace robotarm
