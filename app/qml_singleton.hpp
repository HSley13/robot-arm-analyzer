#pragma once

#include <QJSEngine>
#include <QtGlobal>

namespace robotarm {

// The controllers are QML singletons (QML_ELEMENT + QML_SINGLETON), but
// QML must not construct them itself: they need their config files and
// each other, so main() builds them. A singleton type whose class has a
//     static T *create(QQmlEngine *, QJSEngine *)
// gets its instance from that function instead of a default constructor;
// this hands QML the one main() made, and tells the engine it doesn't own
// it (C++ deletes it, not the garbage collector).
template <typename T> T *existingQmlSingleton(T *instance) {
  Q_ASSERT_X(instance != nullptr, "existingQmlSingleton",
             "set the instance (setQmlInstance) before loading any QML that uses it");
  QJSEngine::setObjectOwnership(instance, QJSEngine::CppOwnership);
  return instance;
}

} // namespace robotarm
