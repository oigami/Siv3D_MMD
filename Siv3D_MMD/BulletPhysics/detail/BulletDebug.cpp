#include "BulletPhysicsDetail.h"
#include "BulletDebug.h"
#include "Siv3DBulletConverter.h"
namespace s3d_bullet {

  namespace {
    template<class F> void SetRasterizerState(RasterizerState state, F f) {
      const RasterizerState rasterizerState = Graphics3D::GetRasterizerState();
      Graphics3D::SetRasterizerState(state);
      f();
      Graphics3D::SetRasterizerState(rasterizerState);
    }

  }
  DebugDraw::DebugDraw()
    : m_debugMode(1) {
  }

  DebugDraw::~DebugDraw() {
  }


  void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& btColor) {
    const Vec3 vec1 = bullet::ConvertVector(from);
    const Vec3 vec2 = bullet::ConvertVector(to);
    ColorF color = bullet::ConvertColor(btColor);
    Line3D(vec1, vec2).drawForward(color);
  }



  void DebugDraw::drawSphere(btScalar radius, const btTransform &transform, const btVector3 & color) {
    const Vec3 pos = bullet::ConvertVector(transform.getOrigin());
    SetRasterizerState(RasterizerState::WireframeCullBack, [&]() {
      Sphere(pos, radius).draw(bullet::ConvertColor(color));
    });
  }

  void DebugDraw::drawBox(const btVector3 & bbMin, const btVector3 & bbMax, const btVector3 & color) {
    const Vec3 from = bullet::ConvertVector(bbMin);
    const Vec3 to = bullet::ConvertVector(bbMax);
    SetRasterizerState(RasterizerState::WireframeCullBack, [&]() {
      Box(from, to).draw(bullet::ConvertColor(color));
    });
  }

  //void DebugDraw::drawBox(const btVector3 & bbMin, const btVector3 & bbMax, const btTransform& trans, const btVector3 & color) {
  //  const Vec3 from = bullet::ConvertVector(trans.getOrigin() - bbMin);
  //  const Vec3 to = bullet::ConvertVector(bbMax - bbMin);
  //  const auto rot = bullet::ConvertQuaternion(trans.getRotation());
  //  SetRasterizerState(RasterizerState::WireframeCullBack, [&]() {
  //    Box(from, to, rot).draw(bullet::ConvertColor(color));
  //  });
  //}

  void DebugDraw::drawCapsule(btScalar radius, btScalar halfHeight, int upAxis,
    const btTransform& transform, const btVector3& color) {

    const Vec3 vec = bullet::ConvertVector(transform.getOrigin());
    const Quaternion rot = bullet::ConvertQuaternion(transform.getRotation()*upAxis);
    SetRasterizerState(RasterizerState::WireframeCullBack, [&]() {

      //TODO: Capsuleがない
      Cylinder(vec, radius, halfHeight * 2, rot).draw(bullet::ConvertColor(color));
    });
  }

  void DebugDraw::drawCylinder(btScalar radius, btScalar halfHeight, int upAxis,
    const btTransform & transform, const btVector3 & color) {

    const Vec3 vec = bullet::ConvertVector(transform.getOrigin());
    const Quaternion rot = bullet::ConvertQuaternion(transform.getRotation()*upAxis);
    SetRasterizerState(RasterizerState::WireframeCullBack, [&]() {
      Cylinder(vec, radius, halfHeight * 2, rot).draw(bullet::ConvertColor(color));
    });
  }

  void DebugDraw::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
    btScalar distance, int, const btVector3& color) {
    btVector3 to = distance * (PointOnB + normalOnB);
    drawLine(PointOnB, to, color);
  }

  void DebugDraw::reportErrorWarning(const char* warningString) {
    Println(Widen(warningString));
  }

  void DebugDraw::draw3dText(const btVector3& location, const char* textString) {
    Println(Widen(textString));
  }

  void DebugDraw::setDebugMode(int debugMode) {
    m_debugMode = debugMode;
  }

  int DebugDraw::getDebugMode() const {
    return m_debugMode;
  }

}
