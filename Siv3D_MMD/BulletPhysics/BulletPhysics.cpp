#include "../include/BulletPhysics.h"

#include "../BulletPhysics/detail/Siv3DBulletConverter.h"
#include "../BulletPhysics/detail/BulletPhysicsDetail.h"
namespace s3d_bullet {
  namespace bullet {
    class Data::Pimpl {
    public:
      std::shared_ptr<bullet::detail::Data> m_data;

    };

    Data::Data(std::shared_ptr<detail::Data> data) {
      m_pimpl = std::make_shared<Pimpl>();
      m_pimpl->m_data = data;
    }

    Data::~Data() {
    }

    void Data::RemoveRigidBody() {
      m_pimpl->m_data->RemoveRigidBody();
    }

    void Data::AddRigidBody(std::uint16_t group, std::uint16_t mask) {
      m_pimpl->m_data->AddRigidBody(group, mask);
    }

    void Data::MoveRigidBody(const Mat4x4 & trans) {
      m_pimpl->m_data->MoveRigidBody(ConvertMatrix(trans));
    }

    void Data::SetMatrixRigidBody(const Mat4x4 & trans) {
      m_pimpl->m_data->SetMatrixRigidBody(ConvertMatrix(trans));
    }

    Mat4x4 Data::GetWorld() const {
      btTransform trans;
      m_pimpl->m_data->body->getMotionState()->getWorldTransform(trans);
      return ConvertMatrix(trans);
    }
  }

  class BulletPhysics::Pimpl {
  public:
    Pimpl(btVector3 gravity) :bulletDatail(gravity) {

    }
    detail::BulletPhysicsDetail bulletDatail;
  };

  BulletPhysics::BulletPhysics(const Float3 & gravity) {
    m_pimpl = std::make_shared<Pimpl>(bullet::ConvertVector(gravity));
  }

  bullet::Data BulletPhysics::CreateBox(float width, float height, float depth, const Mat4x4 & world, float mass,
    float restitution, float friction, float linear_damp, float angular_damp, bool kinematic,
    unsigned short group, unsigned short mask, const Float3 & coord) {

    auto data = m_pimpl->bulletDatail.CreateBox(width, height, depth, bullet::ConvertMatrix(world), mass,
      restitution, restitution, linear_damp, angular_damp, kinematic, group, mask, bullet::ConvertVector(coord));
    return bullet::Data(data);
  }

  bullet::Data BulletPhysics::CreateSphere(float radius, const Mat4x4 & world, float mass, float restitution,
    float friction, float linear_damp, float angular_damp, bool kinematic, unsigned short group, unsigned short mask) {

    auto data = m_pimpl->bulletDatail.CreateSphere(radius, bullet::ConvertMatrix(world), mass, restitution, friction,
      linear_damp, angular_damp, kinematic, group, mask);
    return bullet::Data(data);
  }

  bullet::Data BulletPhysics::CreateCylinder(float radius, float length, const Mat4x4 & world, float mass, float restitution,
    float friction, float linear_damp, float angular_damp, bool kinematic, unsigned short group, unsigned short mask) {

    auto data = m_pimpl->bulletDatail.CreateCylinder(radius, length, bullet::ConvertMatrix(world), mass, restitution, friction,
      linear_damp, angular_damp, kinematic, group, mask);
    return bullet::Data(data);
  }

  bullet::Data BulletPhysics::CreateCapsule(float radius, float height, const Mat4x4 & world, float mass, float restitution,
    float friction, float linear_damp, float angular_damp, bool kinematic,
    unsigned short group, unsigned short mask, const Float3 & coord) {

    auto data = m_pimpl->bulletDatail.CreateCapsule(radius, height, bullet::ConvertMatrix(world),
      mass, restitution, friction, linear_damp, angular_damp, kinematic, group, mask, bullet::ConvertVector(coord));
    return bullet::Data(data);
  }


  void BulletPhysics::Add6DofSpringConstraint(bullet::Data bodyA, bullet::Data bodyB,
    const Mat4x4 & frameInA, const Mat4x4 & frameInB,
    const std::array<float, 3> & c_p1, const std::array<float, 3> & c_p2,
    const std::array<float, 3> & c_r1, const std::array<float, 3> & c_r2,
    const Float3 & stiffnessPos, const Float3 & stiffnessRot) {

    m_pimpl->bulletDatail.Add6DofSpringConstraint(bodyA.m_pimpl->m_data, bodyB.m_pimpl->m_data,
      bullet::ConvertMatrix(frameInA), bullet::ConvertMatrix(frameInB),
      c_p1, c_p2, c_r1, c_r2,
      bullet::ConvertVector(stiffnessPos), bullet::ConvertVector(stiffnessRot));
  }

  void BulletPhysics::StepSimulation() {
    m_pimpl->bulletDatail.StepSimulation();
  }

  void BulletPhysics::DebugDraw() {
    m_pimpl->bulletDatail.DebugDraw();
  }

}

