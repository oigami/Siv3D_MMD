#pragma once
#include <memory>
#include "BulletRigidBody.h"
namespace s3d_bullet
{
  namespace bullet
  {
    class Sphere
    {
      class Pimpl;
      std::shared_ptr<Pimpl> m_pimpl;

    public:

      Sphere() = default;
      Sphere(float radius);
      void setMass(float mass);
      operator Shape()const;
    };
  }
}
