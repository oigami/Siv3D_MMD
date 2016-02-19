#include "../include/PMDReader.h"
#include "ReaderHelper.h"
namespace s3d_mmd {
  PMDReader::PMDReader(const FilePath& path) { // �w�b�_
    BinaryReader reader(path);
    if (!reader.isOpened()) return;
    pmd::Header pmdHeader;
    reader.read(pmdHeader);
    m_modelName = Widen(pmdHeader.model_name);
    m_comment = Widen(pmdHeader.comment);
    ReadSizeAndArray<std::uint32_t>(reader, m_vertices);  // ���_�f�[�^
    ReadSizeAndArray<std::uint32_t>(reader, m_faces);     // �|���S���f�[�^
    ReadSizeAndArray<std::uint32_t>(reader, m_materials); // �ޗ��f�[�^
    ReadSizeAndArray<std::uint16_t>(reader, m_bones);     // �{�[���f�[�^
    const size_t numPmdBone = m_bones.size();
    // IK�f�[�^
    std::uint16_t numPmdIkData;
    reader.read(numPmdIkData);
    m_ikData.resize(numPmdIkData);
    for (auto& pmdIkData : m_ikData) {
      pmd::IkDataWithoutArray *pmdIkDataWithoutArray = &pmdIkData;
      reader.read(*pmdIkDataWithoutArray);
      ReadArray(reader, pmdIkData.ik_child_bone_length, pmdIkData.ik_child_bone_index);
    }
    // �\��f�[�^
    std::uint16_t numPmdSkin;
    reader.read(numPmdSkin);
    m_skinData.resize(numPmdSkin);
    for (auto& skinData : m_skinData) {
      pmd::SkinDataWithoutArray *pmdSkinDataWithoutArray = &skinData;
      reader.read(*pmdSkinDataWithoutArray);
      ReadArray(reader, skinData.skin_vert_count, skinData.skin_vert_data);
    }
    // �\��p�g�\�����X�g
    ReadSizeAndArray<std::uint8_t>(reader, m_skinIndices);
    // �{�[���g�p�g�����X�g
    ReadSizeAndArray<std::uint8_t>(reader, m_boneDispNames);
    const size_t numBoneDispName = m_boneDispNames.size();
    // �{�[���g�p�\�����X�g
    ReadSizeAndArray<std::uint32_t>(reader, m_boneDisps);
    // �p���Ή�
    std::uint8_t english_name_compatibility = 0;
    reader.read(english_name_compatibility);
    // �e��p��
    if (english_name_compatibility == 1) {
      pmd::EnglishName tmp;
      reader.read(tmp.modelName); // ���f����
      reader.read(tmp.comment);   // �R�����g
      ReadArray(reader, numPmdBone, tmp.boneName); // �{�[�����X�g
      if (numPmdSkin)
        ReadArray(reader, numPmdSkin - 1, tmp.skinName); // �\��X�g
      ReadArray(reader, numBoneDispName, tmp.boneDispName); // �{�[���g�p�g�����X�g
      m_englishName = tmp;
    }
    // �g�D�[���e�N�X�`�����X�g
    constexpr int numToonFileName = 10;
    Array<pmd::ToonTexture> toonFileName;
    ReadArray(reader, numToonFileName, toonFileName);
    // ���̃f�[�^
    ReadSizeAndArray<std::uint32_t>(reader, m_rigidBodies);
    // �W���C���g�f�[�^
    ReadSizeAndArray<std::uint32_t>(reader, m_joints);
  }

  PMDReader::~PMDReader() {
  }
}
