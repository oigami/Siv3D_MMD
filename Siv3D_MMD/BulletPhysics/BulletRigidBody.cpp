#include <memory>
#include "detail/BulletPhysicsDetail.h"
#include "detail/Siv3DBulletConverter.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "../include/BulletPhysics.h"
#include "BulletRigidBody.h"
namespace s3d_bullet {
  namespace bullet {
    RigidBody::RigidBody(const Shape & shape) {
      m_shape = shape;
      m_motionState.reset(new btDefaultMotionState(ConvertMatrix(Mat4x4::Identity())));
      float mass = shape.mass();
      btVector3 localInertia(0.f, 0.f, 0.f);
      shape.get()->calculateLocalInertia(mass, localInertia);
      btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,
        m_motionState.get(), shape.get(), localInertia);
      m_rigidBody.reset(new btRigidBody(rbInfo));
    }

    RigidBody & RigidBody::setRestitution(float restitution) {
      m_rigidBody->setRestitution(restitution);
      return *this;
    }

    RigidBody & RigidBody::setFriction(float friction) {
      m_rigidBody->setFriction(friction);
      return *this;
    }

    RigidBody & RigidBody::setDamping(float linear_damp, float angular_damp) {
      m_rigidBody->setDamping(linear_damp, angular_damp);
      return *this;
    }

    RigidBody & RigidBody::setKinematic(bool isKinematic) {
      if (isKinematic) {
        m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        m_rigidBody->setActivationState(DISABLE_DEACTIVATION);
      } else {
        m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
        m_rigidBody->setActivationState(ACTIVE_TAG);
      }
      return *this;
    }

    void RigidBody::addWorld(std::uint_fast16_t group, std::uint_fast16_t mask) {
      auto bulletPhysics = getBulletPhysics();
      bulletPhysics.addRigidBody(*this, group, mask);
    }

    void RigidBody::removeWorld() {

    }
  }
}