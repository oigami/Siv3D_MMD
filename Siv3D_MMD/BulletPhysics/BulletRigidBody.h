#pragma once
#include <memory>
#include "detail/BulletPhysicsDetail.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

namespace s3d_bullet {
  class BulletPhysics;
  namespace bullet {
    class Shape {
      std::shared_ptr<btCollisionShape> m_shape;
      float m_mass;

    public:
      Shape() = default;
      Shape(std::shared_ptr<btCollisionShape> shape, float mass) :m_shape(shape), m_mass(mass) {
      }
      btCollisionShape* get() const { return m_shape.get(); }
      float mass() const { return m_mass; }
    };

    class RigidBody {
      friend BulletPhysics;
      std::unique_ptr<btDefaultMotionState> m_motionState;
      Shape m_shape;

    public:
      std::unique_ptr<btRigidBody> m_rigidBody;
      RigidBody() = default;
      RigidBody(const Shape& shape);

      RigidBody& setRestitution(float restitution);

      RigidBody& setFriction(float friction);

      RigidBody& setDamping(float linear_damp, float angular_damp);

      RigidBody& setKinematic(bool isKinematic);

      void addWorld(std::uint_fast16_t group, std::uint_fast16_t mask);

      void removeWorld();
    };


  }
}
