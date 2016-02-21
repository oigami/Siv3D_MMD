#include "BulletSphere.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

namespace s3d_bullet {
  namespace bullet {
    class Sphere::Pimpl {
    public:
      float m_mass;
      std::shared_ptr<btSphereShape> shape;

      Pimpl(float radius) {
        shape = std::make_shared<btSphereShape>(radius);
        m_mass = 0.f;
      }
    };

    Sphere::Sphere(float radius) {
      m_pimpl = std::make_shared<Pimpl>(radius);
    }

    void Sphere::setMass(float mass) {
      m_pimpl->m_mass = mass;
    }

    Sphere::operator Shape() const {
      return Shape(m_pimpl->shape, m_pimpl->m_mass);
    }

  }
}
