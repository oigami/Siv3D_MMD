#include "../include/BulletCapsule.h"
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>

namespace s3d_bullet {
  namespace bullet {
    class Capsule::Pimpl {
    public:
      float m_mass;
      std::shared_ptr<btCapsuleShape> shape;

      Pimpl(float radius, float height) {
        shape = std::make_shared<btCapsuleShape>(radius, height);
        m_mass = 0.f;
      }
    };

    Capsule::Capsule(float radius, float height) {
      m_pimpl = std::make_shared<Pimpl>(radius, height);
    }

    void Capsule::setMass(float mass) {
      m_pimpl->m_mass = mass;
    }

    Capsule::operator Shape() const {
      return Shape(m_pimpl->shape, m_pimpl->m_mass);
    }

  }
}
