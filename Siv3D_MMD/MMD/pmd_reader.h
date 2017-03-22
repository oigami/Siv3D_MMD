#pragma once
#include <Siv3D.hpp>
#include <MMD/pmd_struct.h>
namespace s3d_mmd
{

  class PMDReader
  {

    pmd_struct::Vertices m_vertices;
    pmd_struct::Faces m_faces;
    pmd_struct::Materials m_materials;
    pmd_struct::Bones m_bones;
    pmd_struct::IkData m_ikData;
    pmd_struct::RigidBodies m_rigidBodies;
    pmd_struct::Joints m_joints;
    pmd_struct::SkinData m_skinData;
    pmd_struct::SkinIndices m_skinIndices;
    pmd_struct::BoneDispNames m_boneDispNames;
    pmd_struct::BoneDisps m_boneDisps;
    Optional<pmd_struct::EnglishName> m_englishName;

    FilePath m_filepath;
    String m_modelName;
    String m_comment;

    bool m_isLoaded = false;

  public:

    const pmd_struct::Vertices& getVertices() const { return m_vertices; }
    const pmd_struct::Faces& getFaces() const { return m_faces; }
    const pmd_struct::Materials& getMaterials() const { return m_materials; }
    const pmd_struct::Bones& getBones() const { return m_bones; }
    const pmd_struct::IkData& getIkData() const { return m_ikData; }
    const pmd_struct::RigidBodies& getRigidBodies()const { return m_rigidBodies; }
    const pmd_struct::Joints& getJoints() const { return m_joints; }
    const pmd_struct::SkinData& getSkinData() const { return m_skinData; }


    const FilePath& getFilePath()const { return m_filepath; }
    const String& getModelName()const { return m_modelName; }
    const String& getComment()const { return m_comment; }

    bool isLoaded() const { return m_isLoaded; }

    void load(IReader& reader);

    explicit PMDReader(IReader& reader);
    explicit PMDReader(const FilePath& path);
    ~PMDReader();

  };

}
