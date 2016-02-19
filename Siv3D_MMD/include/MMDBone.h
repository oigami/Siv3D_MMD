#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd {
  namespace mmd {
    struct Ik {

      int ik_bone_index;
      int ik_target_bone_index;
      int iterations;
      float control_weight;
      Array<int> ik_child_bone_index;

    };

    // �{�[��
    struct Bone {

      Bone();

      int id;                // �{�[��ID�i�ʂ��ԍ��j
      int parent;            // �e�{�[��
      int firstChild;        // �q�{�[��
      int sibling;           // �Z��{�[��
      std::string name;      // �{�[����
      unsigned char type;    // �{�[���^�C�v (MMD�̏ꍇ 0:��] 1:��]�ƈړ� 2:IK 3:�s�� 4:IK�e���� 5:��]�e���� 6:IK�ڑ��� 7:��\�� 8:�P�� 9:��]�^�� )
      bool extraBoneControl; // �K�w�\�����g�킸����boneMat�𑀍�
      Mat4x4 boneMatML;   // �Ǝ��ɓ������ꍇ�̃{�[���s��(extraBoneControl��true�̎��g�p)
      Mat4x4 initMat;        // �����p���s��(�{�[�����[�J�����W�n)
      Mat4x4 initMatML;      // �����p���s��(���f�����[�J�����W�n)
      Mat4x4 boneMat;        // ���݂̃{�[���s��(�{�[�����[�J�����W�n)
      Mat4x4 offsetMat;      // �{�[���I�t�Z�b�g�s��(���f�����[�J�����W�n)

    };

    class Bones {

      Array<mmd::Bone> m_bones;
      Array<mmd::Ik> m_ikData;
      void InitMatCalc(mmd::Bone* me, const Matrix &parentoffsetMat);

      void CalcWorld(const mmd::Bone* me, const Mat4x4 &parentWorldMat, Array<Mat4x4> *worlds) const;

    public:

      Bones(const Array<mmd::Ik> &ikData) : m_ikData(ikData) {}
      Bones() = default;

      Array<mmd::Bone>::iterator begin() { return m_bones.begin(); }
      Array<mmd::Bone>::iterator end() { return m_bones.end(); }
      void resize(size_t size) { m_bones.resize(size); }
      const int size() const { return static_cast<int>(m_bones.size()); }

      // InitMat���{�[�����[�J�����W�n�ɕϊ�����ċA�֐�
      void InitMatCalc();

      //���[���h�ϊ��s��̔z����v�Z����ċA�֐�
      void CalcWorld(const Mat4x4 &world, Array<Mat4x4> &worlds) const;

      // ���f�����[�J�����W�n�ł̃{�[���s����v�Z
      Mat4x4 CalcBoneMatML(int index) const;

      Mat4x4 CalcParentBoneMat(int index)const {
        if (m_bones[index].parent == -1) return Mat4x4::Identity();
        return CalcBoneMatML(m_bones[index].parent);
      };
      const Array<Ik> &ikData() const { return m_ikData; }

      mmd::Bone &get(int i) { return m_bones[i]; }
      const mmd::Bone &operator[](int i) const { return m_bones[i]; }
      mmd::Bone &operator[](int i) { return m_bones[i]; }

    };
  }
}
