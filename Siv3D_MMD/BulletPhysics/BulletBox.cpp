#include "../include/BulletPhysics.h"
#include "detail/BulletPhysicsDetail.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "detail/Siv3DBulletConverter.h"
#include "BulletBox.h"

namespace s3d_bullet {

  namespace bullet {

    class Box::Pimpl {
    public:
      bullet::Data data;
      std::shared_ptr<btBoxShape> box;
      float mass;
      Float3 localInertia;
    };
    Box::Box(const Float3 & pos, const Float3 & size) {
      auto bullet = getBulletPhysics();

    }

    Box::Box(float width, float height, float depth) {
      m_pimpl = std::make_shared<Pimpl>();
      btVector3 halfExtents(width / 2, height / 2, depth / 2);
      m_pimpl->box = std::make_shared<btBoxShape>(halfExtents);

    }

    void Box::calculateLocalInertia(float mass) {
      m_pimpl->mass = mass;
      m_pimpl->box->calculateLocalInertia(mass, bullet::ConvertVector(m_pimpl->localInertia));
    }

    float Box::mass() const {
      return m_pimpl->mass;
    }
    Float3 Box::localInertia() const {
      return m_pimpl->localInertia;
    }

    Box::operator Shape() const {
      return Shape(m_pimpl->box, mass());
    }


  }
}
