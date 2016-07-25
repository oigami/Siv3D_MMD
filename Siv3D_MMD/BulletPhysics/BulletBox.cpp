#include "../include/BulletBox.h"
#include <BulletCollision/CollisionShapes/btBoxShape.h>

namespace s3d_bullet
{

  namespace bullet
  {

    class Box::Pimpl
    {
    public:
      std::shared_ptr<btBoxShape> box;
      float m_mass;
    };
    Box::Box(const Float3 & size) :Box(size.x, size.y, size.z)
    {}

    Box::Box(float width, float height, float depth)
    {
      m_pimpl = std::make_shared<Pimpl>();
      btVector3 halfExtents(width / 2, height / 2, depth / 2);
      m_pimpl->box = std::make_shared<btBoxShape>(halfExtents);
      m_pimpl->m_mass = 0;

    }

    void Box::setMass(float mass)
    {
      m_pimpl->m_mass = mass;
    }

    float Box::mass() const
    {
      return m_pimpl->m_mass;
    }

    Box::operator Shape() const
    {
      return Shape(m_pimpl->box, mass());
    }


  }
}
