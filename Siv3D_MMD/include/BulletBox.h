#pragma once
#include <Siv3D.hpp>
#include "BulletRigidBody.h"
namespace s3d_bullet
{
  namespace bullet
  {
    class RigidBody;

    class Box
    {
      friend RigidBody;
      class Pimpl;
      std::shared_ptr<Pimpl> m_pimpl;
    public:
      Box() = default;
      Box(const Float3& size);
      Box(float width, float height, float depth);

      void setMass(float mass);

      float mass() const;
      operator Shape() const;
    };

  }
}
