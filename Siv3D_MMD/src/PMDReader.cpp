#include <MMD/pmd_reader.h>
#include "ReaderHelper.h"
namespace s3d_mmd
{
  PMDReader::PMDReader(const FilePath& path)
  { // ヘッダ
    BinaryReader reader(path);
    if ( !reader.isOpened() ) return;
    pmd_struct::Header pmdHeader;
    reader.read(pmdHeader);
    m_modelName = Widen(pmdHeader.model_name);
    m_comment = Widen(pmdHeader.comment);
    ReadSizeAndArray<std::uint32_t>(reader, m_vertices);  // 頂点データ
    ReadSizeAndArray<std::uint32_t>(reader, m_faces);     // ポリゴンデータ
    ReadSizeAndArray<std::uint32_t>(reader, m_materials); // 材料データ
    ReadSizeAndArray<std::uint16_t>(reader, m_bones);     // ボーンデータ
    const size_t numPmdBone = m_bones.size();

    // IKデータ
    std::uint16_t numPmdIkData;
    reader.read(numPmdIkData);
    m_ikData.resize(numPmdIkData);
    for ( auto& pmdIkData : m_ikData )
    {
      pmd_struct::IkDataWithoutArray *pmdIkDataWithoutArray = &pmdIkData;
      reader.read(*pmdIkDataWithoutArray);
      ReadArray(reader, pmdIkData.ik_child_bone_length, pmdIkData.ik_child_bone_index);
    }

    // 表情データ
    std::uint16_t numPmdSkin;
    reader.read(numPmdSkin);
    m_skinData.resize(numPmdSkin);
    for ( auto& skinData : m_skinData )
    {
      pmd_struct::SkinDataWithoutArray *pmdSkinDataWithoutArray = &skinData;
      reader.read(*pmdSkinDataWithoutArray);
      ReadArray(reader, skinData.skin_vert_count, skinData.skin_vert_data);
    }

    // 表情用枠表示リスト
    ReadSizeAndArray<std::uint8_t>(reader, m_skinIndices);

    // ボーン枠用枠名リスト
    ReadSizeAndArray<std::uint8_t>(reader, m_boneDispNames);
    const size_t numBoneDispName = m_boneDispNames.size();

    // ボーン枠用表示リスト
    ReadSizeAndArray<std::uint32_t>(reader, m_boneDisps);

    // 英名対応
    std::uint8_t english_name_compatibility = 0;
    reader.read(english_name_compatibility);

    // 各種英名
    if ( english_name_compatibility == 1 )
    {
      pmd_struct::EnglishName tmp;
      reader.read(tmp.modelName); // モデル名
      reader.read(tmp.comment);   // コメント
      ReadArray(reader, numPmdBone, tmp.boneName); // ボーンリスト
      if ( numPmdSkin )
        ReadArray(reader, numPmdSkin - 1, tmp.skinName); // 表情リスト
      ReadArray(reader, numBoneDispName, tmp.boneDispName); // ボーン枠用枠名リスト
      m_englishName = tmp;
    }

    // トゥーンテクスチャリスト
    constexpr int numToonFileName = 10;
    Array<pmd_struct::ToonTexture> toonFileName;
    ReadArray(reader, numToonFileName, toonFileName);

    // 剛体データ
    ReadSizeAndArray<std::uint32_t>(reader, m_rigidBodies);

    // ジョイントデータ
    ReadSizeAndArray<std::uint32_t>(reader, m_joints);
  }

  PMDReader::~PMDReader()
  {
  }
}
