#include <MMD/MMD.h>
#include <MMD/mmd_model.h>
#include "mmd_pimpl.h"

namespace s3d_mmd
{
  namespace
  {
    class EmptyMMDPhysicsWorld : public IMMDPhysicsWorld
    {
    public:
      std::shared_ptr<IMMDPhysics> create() override
      {
        class EmptyPhycics : public IMMDPhysics
        {
        public:
          void create(std::shared_ptr<mmd::Bones>,
                      const std::vector<pmd_struct::RigidBody>&,
                      const std::vector<pmd_struct::Joint>&) override { }

          void boneUpdate(const Mat4x4&) override {}
        };
        return std::make_shared<EmptyPhycics>();
      }

      void update() override {}
    };
  }

  static std::shared_ptr<IMMDPhysicsWorld> defaultPhysicsWorld = std::make_shared<EmptyMMDPhysicsWorld>();

  void MMD::SetDefaultPhysicsWorld(std::shared_ptr<IMMDPhysicsWorld> factory)
  {
    defaultPhysicsWorld = std::move(factory);
  }

  MMD::MMD(const MMDModel& model, std::shared_ptr<IMMDPhysics> physics)
  {
    m_handle = std::make_shared<Pimpl>(model, physics ? physics : defaultPhysicsWorld->create());
  }

  MMD::~MMD() {}

  void MMD::PhysicsUpdate() const
  {
    m_handle->physicsUpdate();
  }

  void MMD::drawForward(double edgeSize, const Mat4x4& worldMat) const
  {
    drawForward(worldMat);
    drawEdge(edgeSize, worldMat);
  }

  void MMD::drawEdge(double edgeSize, const Mat4x4& worldMat) const
  {
    m_handle->drawEdge(edgeSize, worldMat);
  }

  void MMD::drawForward(const Mat4x4& worldMat) const
  {
    m_handle->drawForward(worldMat);
  }

  const Texture& MMD::vertexTexture() const
  {
    return m_handle->m_vertexTexture;
  }

  void MMD::attach(const VMD& vmd) const
  {
    m_handle->attach(vmd);
  }

  void MMD::dettach() const
  {
    m_handle->attach(VMD());
  }

  void MMD::release() { m_handle.reset(); }

  bool MMD::isOpened() const
  {
    return !!m_handle;
  }

  std::shared_ptr<mmd::Bones> MMD::bones() const
  {
    return m_handle->m_bones;
  }

  mmd::FaceMorph& MMD::morphs() const
  {
    return m_handle->m_faceMorph;
  }

  const String& MMD::name() const
  {
    return m_handle->m_name;
  }

  const String& MMD::comment() const
  {
    return m_handle->m_comment;
  }

  const MMD& MMD::update() const
  {
    m_handle->update();

    return *this;
  }
}
