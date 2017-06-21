#pragma once
#include <Siv3D.hpp>

namespace s3d_mmd
{
  namespace mmd
  {
    struct Ik
    {
      int ik_bone_index;
      int ik_target_bone_index;
      int iterations;
      float control_weight;
      Array<int> ik_child_bone_index;
    };

    // ボーン
    struct alignas(16) Bone
    {
      Bone();

      Vector bonePosition; // 現在のボーンのPosition(ボーンローカル座標系)
      Quaternion boneRotation; // 現在のボーンのQuaternion(ボーンローカル座標系)
      Vector initMatBoneLocal; // 初期姿勢行列(ボーンローカル座標系)
      Vector initMatML; // 初期姿勢行列(モデルローカル座標系)
      Vector offsetMat; // ボーンオフセット行列(モデルローカル座標系)
      Mat4x4 boneMatML; // 独自に動かす場合のボーン行列(extraBoneControlがtrueの時使用)
      String name; // ボーン名
      int id; // ボーンID（通し番号）
      int parent; // 親ボーン
      int firstChild; // 子ボーン
      int sibling; // 兄弟ボーン
      unsigned char type; // ボーンタイプ (MMDの場合 0:回転 1:回転と移動 2:IK 3:不明 4:IK影響下 5:回転影響下 6:IK接続先 7:非表示 8:捻り 9:回転運動 )
      bool extraBoneControl; // 階層構造を使わず直接boneMatを操作
    };

    class Bones
    {
      std::unordered_map<String, int> m_boneNameIndex;
      Array<mmd::Bone> m_bones;
      Array<mmd::Ik> m_ikData;
      Array<Mat4x4> m_lastUpdatedModelLocals;
      Array<Mat4x4> m_lastUpdatedWorlds;
      void initMatCalc(mmd::Bone* me, Vector parentoffsetMat);

      void calcWorld(const Bone& me, const Mat4x4& parentWorldMat);

    public:

      Bones(Array<Bone> bones, Array<mmd::Ik> ikData);
      Bones() = default;

      Array<mmd::Bone>::iterator begin() { return m_bones.begin(); }

      Array<mmd::Bone>::iterator end() { return m_bones.end(); }

      const int size() const { return static_cast<int>(m_bones.size()); }

      // InitMatをボーンローカル座標系に変換する再帰関数
      void initMatCalc();

      //ワールド変換行列の配列を計算する再帰関数
      Array<Mat4x4>& calcWorld(const Mat4x4 & world);

      Array<Mat4x4>& lastUpdatedWorld() { return m_lastUpdatedWorlds; }
      const Array<Mat4x4>& lastUpdatedWorld()const { return m_lastUpdatedWorlds; }

      Array<Mat4x4>& lastUpdatedModelLocal() { return m_lastUpdatedModelLocals; }
      const Array<Mat4x4>& lastUpdatedModelLocal()const { return m_lastUpdatedModelLocals; }


      // モデルローカル座標系でのボーン行列を計算
      Mat4x4 calcBoneMatML(int index) const;
      Vector calcBonePositionML(int index) const;
      Optional<Mat4x4> calcBoneMatML(const String& boneName) const;

      Mat4x4 calcParentBoneMat(int index) const;
      Optional<Mat4x4> calcParentBoneMat(const String& boneName) const;

      const Array<Ik>& ikData() const { return m_ikData; }

      Optional<int> getBoneIndex(const String& boneName) const;

      const mmd::Bone& get(int i) const { return m_bones[i]; }

      mmd::Bone& get(int i) { return m_bones[i]; }

      const mmd::Bone& operator[](int i) const { return m_bones[i]; }

      mmd::Bone& operator[](int i) { return m_bones[i]; }
    };
  }
}
