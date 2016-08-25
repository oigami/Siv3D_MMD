#include <MMD/MMD.h>
#include <MMD/MMDModel.h>
#include "detail/mmd_pimpl.h"
#ifdef USE_BULLET_PHYSICS
#include "MMDPhysics.h"
#endif // USE_BULLET_PHYSICS

namespace s3d_mmd
{

  MMD::MMD(const MMDModel &model)
  {
    m_handle = std::make_shared<Pimpl>(model);
  }

  MMD::~MMD() {}
  void MMD::PhysicsUpdate(Array<Mat4x4> &boneWorld) const
  {
    m_handle->PhysicsUpdate(boneWorld);

  }
  void MMD::draw(double edgeSize) const
  {
    draw();
    drawEdge(edgeSize);
  }

  void MMD::drawEdge(double edgeSize) const
  {
    m_handle->drawEdge();
  }

  void MMD::draw(const Mat4x4& worldMat) const
  {
    m_handle->draw(worldMat);
  }

  void MMD::draw(const VMD &vmd, const Mat4x4& worldMat) const
  {
    DrawableVMD(*this, vmd).draw(worldMat);
  }

  const Texture & MMD::vertexTexture() const
  {
    return m_handle->m_vertexTexture;
  }

  bool MMD::isOpen() const
  {
    return !!m_handle;
  }

  std::shared_ptr<mmd::Bones> MMD::bones() const
  {
    return m_handle->m_bones;
  }

  mmd::FaceMorph & MMD::morphs() const
  {
    return m_handle->m_faceMorph;
  }

  const String & MMD::name() const
  {
    return m_handle->m_name;
  }

  const String & MMD::comment() const
  {
    return m_handle->m_comment;
  }

  DrawableVMD::DrawableVMD(const MMD &mmd, const VMD &vmd) :m_mmd(mmd), m_vmd(vmd) {}

  void DrawableVMD::draw(const Mat4x4& worldMat) const
  {
    auto bones = m_mmd.bones();
    m_vmd.UpdateBone(*bones);
    m_vmd.UpdateMorph(m_mmd.morphs());
    {
      m_mmd.draw(worldMat);
    }
    {
      m_mmd.drawEdge(1.0);
    }
  }

}
