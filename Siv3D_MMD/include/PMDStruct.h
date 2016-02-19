#pragma once
#include <cstdint>
#include <vector>
namespace s3d_mmd {
  namespace pmd {

#pragma pack(push, 1)

    /// PMD�\���̒�`
    struct Header {
      std::uint8_t magic[3];
      float version;
      char model_name[20];
      char comment[256];
    };

    struct Vertex {
      float pos[3];
      float normal_vec[3];
      float uv[2];
      std::uint16_t bone_num[2];
      std::uint8_t bone_weight;
      std::uint8_t edge_flag;
    };

    struct Material {
      float diffuse_color[3];
      float alpha;
      float specularity;
      float specular_color[3];
      float mirror_color[3];
      std::uint8_t toon_index;
      std::uint8_t edge_flag;
      std::uint32_t face_vert_count; // ���̍ޗ��̖ʒ��_�� �� �ޗ��ԍ�i�̃|���S���ԍ��F pmdMaterial[i - 1].face_vert_count/3 �` pmdMaterial[i].face_vert_count/3 - 1
      char texture_file_name[20];
    };

    struct Bone {
      char bone_name[20];
      std::uint16_t parent_bone_index;    // �Ȃ��ꍇ��0xFFFF
      std::uint16_t tail_pos_bone_index;  // �Ȃ��ꍇ��0xFFFF
      std::uint8_t bone_type;             // 0:��] 1:��]�ƈړ� 2:IK 3:�s�� 4:IK�e���� 5:��]�e���� 6:IK�ڑ��� 7:��\�� 8:�P�� 9:��]�^�� (8, 9��MMD4.0�ȍ~)
      std::uint16_t ik_parent_bone_index; // �Ȃ��ꍇ��0
      float bone_head_pos[3];
    };

    struct IkDataWithoutArray {
      std::uint16_t ik_bone_index;        // IK�{�[���ԍ�
      std::uint16_t ik_target_bone_index; // IK�{�[���̈ʒu�ɂ��̃{�[������v������悤��IK�������s����
      std::uint8_t ik_child_bone_length;  // IK�`�F�[���̒���
      std::uint16_t iterations;           // �ċA���Z��
      float control_weight;               // �{�[���̒P�ʐ����p 1.0 �� 4.0[rad]�B�܂��u�Ђ��v���܂ރ{�[���������{�[����X�������ɂ��������Ȃ�����������B
                                          // unsigned short ik_child_bone_index[ik_chain_length]; // IK�e�����̃{�[���ԍ� (�z��̑傫�����ω�)
    };

    struct SkinDataWithoutArray {
      char skin_name[20];            // �\�
      std::uint32_t skin_vert_count; // �\��p�̒��_��
      std::uint8_t skin_type;        // �\��̎�� �� 0:base�A1:�܂�A2:�ځA3:���b�v�A4:���̑�
                                     // PmdSkinVertexData skin_vert_data[skin_vert_count];	// (�z��̑傫�����ω�)
    };

    struct SkinVertexData {
      std::uint32_t skin_vert_index; // �\��p�̒��_�̔ԍ�(���_���X�g�ɂ���ԍ�)
      float skin_vert_pos[3];
    };

    struct BoneDispName {
      char name[50];
    };

    struct BoneDisp {
      std::uint16_t bone_index;           // �g�p�{�[���ԍ�
      std::uint8_t bone_disp_frame_index; // �\���g�ԍ�
    };

    struct RigidBody {
      char rigidbody_name[20];                // ����
      std::uint16_t rigidbody_rel_bone_index; // �֘A�{�[���ԍ� �i�Ȃ���0xFFFF�j
      std::uint8_t rigidbody_group_index;     // �O���[�v(1�`16)
      std::uint16_t rigidbody_group_target;   // ��Փ˃O���[�v(�r�b�g�P��)
      std::uint8_t shape_type;                // �`��^�C�v 0:�� 1:�� 2:�J�v�Z��
      float shape_w;                          // ��/2(���͔��a)
      float shape_h;                          // ����/2(�J�v�Z���̏ꍇ�͋����S�Ԃ̋���)
      float shape_d;                          // ���s/2
      float pos_pos[3];                       // �ʒu (x, y, z)  : �֘A�{�[��������ꍇ�́A���̃^�C�v�ɂ�炸�֘A�{�[���ʒu����̑��Έʒu(PMD�G�f�B�^�ł̓��f���̃��[�J�����W�n�ł���A�l���Ⴄ�̂Œ��ӁI)
      float pos_rot[3];                       // ��] : ��]�s��� MATRIXRotationYawPitchRoll(&rotation, pos_rot[1], pos_rot[0], pos_rot[2])
      float rigidbody_weight;                 // ����
      float rigidbody_pos_dim;                // �ړ�����
      float rigidbody_rot_dim;                // ��]����
      float rigidbody_recoil;                 // ������
      float rigidbody_friction;               // ���C��
      std::uint8_t rigidbody_type;            // ���̃^�C�v 0:Bone�Ǐ] 1:�������Z 2:�������Z(Bone�ʒu���킹)
    };

    struct Joint {
      char joint_name[20];             // ����
      std::uint32_t joint_rigidbody_a; // ����A
      std::uint32_t joint_rigidbody_b; // ����B
      float joint_pos[3];              // �ʒu (x, y, z) : ���f�����[�J�����W�n
      float joint_rot[3];              // ��] (rad(x), rad(y), rad(z))
      float constrain_pos_1[3];        // �ړ��������� (x, y, z)
      float constrain_pos_2[3];        // �ړ�������� (x, y, z)
      float constrain_rot_1[3];        // ��]�������� (rad(x), rad(y), rad(z))
      float constrain_rot_2[3];        // ��]������� (rad(x), rad(y), rad(z))
      float spring_pos[3];             // �ړ��΂� (x, y, z)
      float spring_rot[3];             // ��]�΂� (rad(x), rad(y), rad(z))
    };

    struct BoneEnglishName {
      char name[20];
    };

    struct SkinEnglishName {
      char name[20];
    };

    struct BoneDispEnglishName {
      char name[50];
    };

    struct ToonTexture {
      char filename[100];
    };

#pragma pack(pop)

    struct EnglishName {
      char modelName[20];
      char comment[256];
      std::vector<BoneEnglishName> boneName;
      std::vector<SkinEnglishName> skinName;
      std::vector<BoneDispEnglishName> boneDispName;
    };

    struct Ik : public IkDataWithoutArray {
      std::vector<std::uint16_t> ik_child_bone_index; // unsigned short ik_child_bone_index[ik_chain_length]; // IK�e�����̃{�[���ԍ�
    };

    struct Skin : public SkinDataWithoutArray {
      std::vector<SkinVertexData> skin_vert_data; //�\��p���_�f�[�^
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
