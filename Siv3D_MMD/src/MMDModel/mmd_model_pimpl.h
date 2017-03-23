#pragma once
#include "MMD/MMD.h"
#include "MMD/pmd_reader.h"

namespace s3d_mmd
{
  class MMDModel::Pimpl
  {
  public:
    std::shared_ptr<ITextureLoader> m_textureLoader;

    Pimpl(const FilePath& path, const std::shared_ptr<ITextureLoader> textureLoader);

    Pimpl(PMDReader& loader, const std::shared_ptr<ITextureLoader> textureLoader);

    void load(const PMDReader& loader, const String& path);

    Pimpl() : m_handle(NullHandleID) {}

    void release();

    Array<mmd::ModelNode> m_nodes;
    std::shared_ptr<mmd::Bones> m_bones;

    pmd_struct::RigidBodies m_rigidBodies;
    pmd_struct::Joints m_joints;
    pmd_struct::SkinData m_skinData;
    String m_modelName;
    String m_comment;
    HandleIDType m_handle;
    std::vector<mmd::MeshVertex> m_vertecies;
    std::vector<uint16> m_indices;
    String m_baseDir;

    mmd::Material createMaterial(const pmd_struct::Material& pmdMaterial, const FilePath& m_filepath) const;
    void createNode(const PMDReader& loader, const FilePath& m_filepath);
  };
}
