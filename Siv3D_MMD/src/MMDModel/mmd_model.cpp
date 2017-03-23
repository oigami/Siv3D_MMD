#include <MMD/mmd_model.h>
#include <MMD/pmd_reader.h>
#include "mmd_model_pimpl.h"

namespace s3d_mmd
{
  MMDModel::MMDModel() {}

  MMDModel::MMDModel(const FilePath& path, std::shared_ptr<ITextureLoader> textureLoader)
  {
    m_handle = std::make_shared<Pimpl>(path, textureLoader);
  }

  MMDModel::~MMDModel()
  {
    m_handle = std::make_shared<Pimpl>();
  }

  void MMDModel::release()
  {
    m_handle->release();
  }

  HandleIDType MMDModel::id() const
  {
    return m_handle->m_handle;
  }

  bool MMDModel::isEmpty() const
  {
    return !m_handle;
  }

  bool MMDModel::operator==(const MMDModel& model) const
  {
    return m_handle == model.m_handle;
  }

  bool MMDModel::operator!=(const MMDModel& model) const
  {
    return !(*this == model);
  }

  Array<mmd::ModelNode>& MMDModel::nodes() const
  {
    return m_handle->m_nodes;
  }

  Array<mmd::MeshVertex>& MMDModel::vertices() const
  {
    return m_handle->m_vertecies;
  }

  Array<uint16>& MMDModel::indices() const
  {
    return m_handle->m_indices;
  }

  std::shared_ptr<mmd::Bones> MMDModel::bones() const
  {
    return m_handle->m_bones;
  }

  const pmd_struct::RigidBodies& MMDModel::rigidBodies() const
  {
    return m_handle->m_rigidBodies;
  }

  const pmd_struct::Joints& MMDModel::joints() const
  {
    return m_handle->m_joints;
  }

  const pmd_struct::SkinData MMDModel::skinData() const
  {
    return m_handle->m_skinData;
  }

  const String& MMDModel::name() const
  {
    return m_handle->m_modelName;
  }

  const String& MMDModel::comment() const
  {
    return m_handle->m_comment;
  }
}
