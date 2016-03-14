#include "../include/BulletPhysics.h"
#include "../include/BulletRigidBody.h"
#include "detail/Siv3DBulletConverter.h"
#include "detail/BulletPhysicsDetail.h"

namespace s3d_bullet {


  class BulletPhysics::Pimpl {
  public:
    Pimpl(btVector3 gravity) :bulletDatail(gravity) {

    }
    detail::BulletPhysicsDetail bulletDatail;
  };

  BulletPhysics::BulletPhysics(const Float3 & gravity) {
    m_pimpl = std::make_shared<Pimpl>(bullet::ConvertVector(gravity));
  }

  /*bullet::Data BulletPhysics::CreateCylinder(float radius, float length, const Mat4x4 & world, float mass, float restitution,
    float friction, float linear_damp, float angular_damp, bool kinematic, unsigned short group, unsigned short mask) {

    auto data = m_pimpl->bulletDatail.CreateCylinder(radius, length, bullet::ConvertMatrix(world), mass, restitution, friction,
      linear_damp, angular_damp, kinematic, group, mask);
    return bullet::Data(data);
  }*/


  void BulletPhysics::Add6DofSpringConstraint(bullet::RigidBody & bodyA, bullet::RigidBody & bodyB,
    const Mat4x4 & frameInA, const Mat4x4 & frameInB,
    const std::array<float, 3>& c_p1, const std::array<float, 3>& c_p2,
    const std::array<float, 3>& c_r1, const std::array<float, 3>& c_r2,
    const Float3 & stiffnessPos, const Float3 & stiffnessRot) {

    auto constraint = m_pimpl->bulletDatail.Add6DofSpringConstraint(*bodyA.m_rigidBody, *bodyB.m_rigidBody,
      bullet::ConvertMatrix(frameInA), bullet::ConvertMatrix(frameInB),
      c_p1, c_p2, c_r1, c_r2,
      bullet::ConvertFloat(stiffnessPos), bullet::ConvertFloat(stiffnessRot));
    bodyA.m_constraint.push_back(constraint);
    bodyA.m_constraint.push_back(constraint);
  }

  void BulletPhysics::StepSimulation() {
    m_pimpl->bulletDatail.StepSimulation();
  }

  void BulletPhysics::DebugDraw() {
    m_pimpl->bulletDatail.DebugDraw();
  }

  void BulletPhysics::addRigidBody(const bullet::RigidBody& body, std::uint_fast16_t group, std::uint_fast16_t mask) {
    m_pimpl->bulletDatail.addRigidBody(body.m_rigidBody.get(), group, mask);
  }

  void BulletPhysics::removeRigidBody() {
  }

}
