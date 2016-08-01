#pragma once
#include <Siv3D.hpp>
#include "PMDStruct.h"
namespace s3d_mmd
{

  class PMDReader
  {

    pmd::Vertices m_vertices;
    pmd::Faces m_faces;
    pmd::Materials m_materials;
    pmd::Bones m_bones;
    pmd::IkData m_ikData;
    pmd::RigidBodies m_rigidBodies;
    pmd::Joints m_joints;
    pmd::SkinData m_skinData;
    pmd::SkinIndices m_skinIndices;
    pmd::BoneDispNames m_boneDispNames;
    pmd::BoneDisps m_boneDisps;
    Optional<pmd::EnglishName> m_englishName;

    FilePath m_filepath;
    String m_modelName;
    String m_comment;

  public:

    const pmd::Vertices& getVertices() const { return m_vertices; }
    const pmd::Faces& getFaces() const { return m_faces; }
    const pmd::Materials& getMaterials() const { return m_materials; }
    const pmd::Bones& getBones() const { return m_bones; }
    const pmd::IkData& getIkData() const { return m_ikData; }
    const pmd::RigidBodies& getRigidBodies()const { return m_rigidBodies; }
    const pmd::Joints& getJoints() const { return m_joints; }
    const pmd::SkinData& getSkinData() const { return m_skinData; }


    const FilePath& getFilePath()const { return m_filepath; }
    const String& getModelName()const { return m_modelName; }
    const String& getComment()const { return m_comment; }
    PMDReader(const FilePath& path);
    ~PMDReader();

  };

}
