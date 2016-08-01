﻿#pragma once
#include <cstdint>
#include <vector>
namespace s3d_mmd
{
  namespace pmd
  {

#pragma pack(push, 1)

    /// PMD構造体定義
    struct Header
    {
      std::uint8_t magic[3];
      float version;
      char model_name[20];
      char comment[256];
    };

    struct Vertex
    {
      float pos[3];
      float normal_vec[3];
      float uv[2];
      std::uint16_t bone_num[2];
      std::uint8_t bone_weight;
      std::uint8_t edge_flag;
    };

    struct Material
    {
      float diffuse_color[3];
      float alpha;
      float specularity;
      float specular_color[3];
      float mirror_color[3];
      std::uint8_t toon_index;
      std::uint8_t edge_flag;
      std::uint32_t face_vert_count; // この材料の面頂点数 → 材料番号iのポリゴン番号： pmdMaterial[i - 1].face_vert_count/3 ～ pmdMaterial[i].face_vert_count/3 - 1
      char texture_file_name[20];
    };

    struct Bone
    {
      char bone_name[20];
      std::uint16_t parent_bone_index;    // ない場合は0xFFFF
      std::uint16_t tail_pos_bone_index;  // ない場合は0xFFFF
      std::uint8_t bone_type;             // 0:回転 1:回転と移動 2:IK 3:不明 4:IK影響下 5:回転影響下 6:IK接続先 7:非表示 8:捻り 9:回転運動 (8, 9はMMD4.0以降)
      std::uint16_t ik_parent_bone_index; // ない場合は0
      float bone_head_pos[3];
    };

    struct IkDataWithoutArray
    {
      std::uint16_t ik_bone_index;        // IKボーン番号
      std::uint16_t ik_target_bone_index; // IKボーンの位置にこのボーンを一致させるようにIK処理が行われる
      std::uint8_t ik_child_bone_length;  // IKチェーンの長さ
      std::uint16_t iterations;           // 再帰演算回数
      float control_weight;               // ボーンの単位制限角 1.0 → 4.0[rad]。また「ひざ」を含むボーン名をもつボーンはX軸方向にしか動かない制限がある。

                                          // unsigned short ik_child_bone_index[ik_chain_length]; // IK影響下のボーン番号 (配列の大きさが変化)
    };

    struct SkinDataWithoutArray
    {
      char skin_name[20];            // 表情名
      std::uint32_t skin_vert_count; // 表情用の頂点数
      std::uint8_t skin_type;        // 表情の種類 → 0:base、1:まゆ、2:目、3:リップ、4:その他

                                     // PmdSkinVertexData skin_vert_data[skin_vert_count];	// (配列の大きさが変化)
    };

    struct SkinVertexData
    {
      std::uint32_t skin_vert_index; // 表情用の頂点の番号(頂点リストにある番号)
      Float3 skin_vert_pos;
      static_assert(std::is_pod<Float3>::value, "");
      static_assert(sizeof(Float3) == sizeof(float[3]), "");
    };

    struct BoneDispName
    {
      char name[50];
    };

    struct BoneDisp
    {
      std::uint16_t bone_index;           // 枠用ボーン番号
      std::uint8_t bone_disp_frame_index; // 表示枠番号
    };

    struct RigidBody
    {
      char rigidbody_name[20];                // 名称
      std::uint16_t rigidbody_rel_bone_index; // 関連ボーン番号 （なし→0xFFFF）
      std::uint8_t rigidbody_group_index;     // グループ(1～16)
      std::uint16_t rigidbody_group_target;   // 非衝突グループ(ビット単位)
      std::uint8_t shape_type;                // 形状タイプ 0:球 1:箱 2:カプセル
      float shape_w;                          // 幅/2(又は半径)
      float shape_h;                          // 高さ/2(カプセルの場合は球中心間の距離)
      float shape_d;                          // 奥行/2
      float pos_pos[3];                       // 位置 (x, y, z)  : 関連ボーンがある場合は、剛体タイプによらず関連ボーン位置からの相対位置(PMDエディタではモデルのローカル座標系であり、値が違うので注意！)
      float pos_rot[3];                       // 回転 : 回転行列は MATRIXRotationYawPitchRoll(&rotation, pos_rot[1], pos_rot[0], pos_rot[2])
      float rigidbody_weight;                 // 質量
      float rigidbody_pos_dim;                // 移動減衰
      float rigidbody_rot_dim;                // 回転減衰
      float rigidbody_recoil;                 // 反発力
      float rigidbody_friction;               // 摩擦力
      std::uint8_t rigidbody_type;            // 剛体タイプ 0:Bone追従 1:物理演算 2:物理演算(Bone位置あわせ)
    };

    struct Joint
    {
      char joint_name[20];             // 名称
      std::uint32_t joint_rigidbody_a; // 剛体A
      std::uint32_t joint_rigidbody_b; // 剛体B
      float joint_pos[3];              // 位置 (x, y, z) : モデルローカル座標系
      float joint_rot[3];              // 回転 (rad(x), rad(y), rad(z))
      float constrain_pos_1[3];        // 移動制限下限 (x, y, z)
      float constrain_pos_2[3];        // 移動制限上限 (x, y, z)
      float constrain_rot_1[3];        // 回転制限下限 (rad(x), rad(y), rad(z))
      float constrain_rot_2[3];        // 回転制限上限 (rad(x), rad(y), rad(z))
      float spring_pos[3];             // 移動ばね (x, y, z)
      float spring_rot[3];             // 回転ばね (rad(x), rad(y), rad(z))
    };

    struct BoneEnglishName
    {
      char name[20];
    };

    struct SkinEnglishName
    {
      char name[20];
    };

    struct BoneDispEnglishName
    {
      char name[50];
    };

    struct ToonTexture
    {
      char filename[100];
    };

#pragma pack(pop)

    struct EnglishName
    {
      char modelName[20];
      char comment[256];
      std::vector<BoneEnglishName> boneName;
      std::vector<SkinEnglishName> skinName;
      std::vector<BoneDispEnglishName> boneDispName;
    };

    struct Ik : public IkDataWithoutArray
    {
      std::vector<std::uint16_t> ik_child_bone_index; // unsigned short ik_child_bone_index[ik_chain_length]; // IK影響下のボーン番号
    };

    struct Skin : public SkinDataWithoutArray
    {
      std::vector<SkinVertexData> skin_vert_data; //表情用頂点データ
    };

    using Vertices = Array<Vertex>;
    using Faces = Array<std::uint16_t>;
    using Materials = Array<Material>;
    using Bones = Array<Bone>;
    using IkData = Array<Ik>;
    using RigidBodies = Array<RigidBody>;
    using Joints = Array<Joint>;
    using SkinData = Array<Skin>;
    using SkinIndices = Array<std::uint16_t>;
    using BoneDispNames = Array<BoneDispName>;
    using BoneDisps = Array<BoneDisp>;

  }


}
