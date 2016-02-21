#pragma once
#include <Siv3D.hpp>
namespace s3d_bullet {
  namespace bullet {
    class RigidBody;

    class Box {
      friend RigidBody;
      class Pimpl;
      std::shared_ptr<Pimpl> m_pimpl;
    public:
      Box() = default;
      Box(const Float3& pos, const Float3& size);
      Box::Box(float width, float height, float depth);

      void calculateLocalInertia(float mass);

      float mass() const;
      Float3 localInertia() const;
      operator Shape() const;
    };

  }
}
