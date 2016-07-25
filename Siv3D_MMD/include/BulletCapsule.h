#pragma once
#include <memory>
#include "BulletRigidBody.h"

namespace s3d_bullet
{
  namespace bullet
  {
    class Capsule
    {
      class Pimpl;
      std::shared_ptr<Pimpl> m_pimpl;

    public:
      Capsule() = default;
      Capsule(float radius, float height);

      void setMass(float mass);

      operator Shape() const;

    };
  }
}
