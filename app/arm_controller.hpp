#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QVariantList>
#include <QVector3D>

#include "robotarm/arm_config.hpp"
#include "robotarm/arm_model.hpp"
#include "robotarm/statics.hpp"
#include "robotarm/surrogate.hpp"

namespace robotarm {

// QML-facing backend for the robot arm view: owns the ArmModel/LinkMass
// data for one arm config, holds the current joint angles/payload/mode,
// and recomputes torques, joint positions, and threshold colors whenever
// any of those change. QML only ever sees plain Qt types (QVariantList,
// QVector3D, QString, double, bool, int) - never the core/ml C++ types
// directly, matching the plan's "three pieces only share data" goal.
class ArmController : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString armName READ armName CONSTANT)
  Q_PROPERTY(int jointCount READ jointCount CONSTANT)
  Q_PROPERTY(bool aiModeAvailable READ aiModeAvailable CONSTANT)
  Q_PROPERTY(QVector3D canvasCenter READ canvasCenter CONSTANT)

  Q_PROPERTY(double payloadKg READ payloadKg WRITE setPayloadKg NOTIFY resultsChanged)
  Q_PROPERTY(bool aiMode READ aiMode WRITE setAiMode NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList jointAnglesDegrees READ jointAnglesDegrees NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointTorques READ jointTorques NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList jointTorqueLimits READ jointTorqueLimits CONSTANT)
  Q_PROPERTY(QVariantList jointColors READ jointColors NOTIFY resultsChanged)

  Q_PROPERTY(QVariantList jointPositions READ jointPositions NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkMidpoints READ linkMidpoints NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkLengths READ linkLengths NOTIFY resultsChanged)
  Q_PROPERTY(QVariantList linkRotations READ linkRotations NOTIFY resultsChanged)

public:
  // surrogate_path may point to a file that doesn't exist - AI mode is
  // then simply unavailable (aiModeAvailable() returns false) rather than
  // throwing, since not every arm has a trained surrogate yet.
  explicit ArmController(const std::filesystem::path &config_path, const std::filesystem::path &surrogate_path,
                          QObject *parent = nullptr);

  QString armName() const;
  int jointCount() const { return static_cast<int>(_arm.num_joints()); }
  bool aiModeAvailable() const { return _surrogate != nullptr; }
  QVector3D canvasCenter() const { return _canvasCenter; }

  double payloadKg() const { return _payloadKg; }
  void setPayloadKg(double kg);
  bool aiMode() const { return _aiMode; }
  void setAiMode(bool enabled);

  QVariantList jointAnglesDegrees() const;
  QVariantList jointTorques() const;
  QVariantList jointTorqueLimits() const;
  QVariantList jointColors() const;

  QVariantList jointPositions() const;
  QVariantList linkMidpoints() const;
  QVariantList linkLengths() const;
  QVariantList linkRotations() const;

  Q_INVOKABLE void setJointAngleDegrees(int index, double degrees);

signals:
  void resultsChanged();

private:
  void recompute();

  ArmConfig _config;
  ArmModel _arm;
  std::vector<LinkMass> _links;
  std::unique_ptr<Surrogate> _surrogate;

  std::vector<double> _jointAnglesRadians;
  double _payloadKg = 0.0;
  bool _aiMode = false;

  std::vector<Eigen::Vector3d> _jointPositions; // base origin + one per joint
  std::vector<double> _jointTorques;
  QVector3D _canvasCenter;
};

} // namespace robotarm
