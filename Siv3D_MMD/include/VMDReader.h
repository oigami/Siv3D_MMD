#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd {
  namespace vmd {

#pragma pack(push, 1)
    //http://harigane.at.webry.info/201103/article_1.html

    /// VMD�\���̒�`
    struct Header {
      char vmdHeader[30];
      char vmdModelName[20];
    };

    struct Bone {
      char boneName[15];
      std::uint32_t frameNo;
      float location[3]; // �ړ���
      float rotation[4]; // ���f�����[�J�����W�n
      std::uint8_t interpolation[64];
    };

    struct Morph {
      char name[15];
      std::uint32_t frameNo;
      float weight;
    };

    struct Camera {
      std::uint32_t frameNo;   // �t���[���ԍ�
      float distance;          // �ڕW�_�ƃJ�����̋���(�ڕW�_���J�����O�ʂŃ}�C�i�X)
      float x;                 // �ڕW�_��X���ʒu
      float y;                 // �ڕW�_��Y���ʒu
      float z;                 // �ڕW�_��Z���ʒu
      float rx;                // �J������x����](rad)(MMD���l���͂̃}�C�i�X�l)
      float ry;                // �J������y����](rad)
      float rz;                // �J������z����](rad)
      std::uint8_t bezier[24]; // ��ԃp�����[�^
      std::uint32_t viewAngle; // ����p(deg)
      std::uint8_t parth;      // �p�[�X�y�N�e�B�u, 0:ON, 1:OFF
    };

    struct Light {
      std::uint32_t frame; // �t���[���ԍ�
      float r;             // �Ɩ��F��(MMD���͒l��256�Ŋ������l)
      float g;             // �Ɩ��F��(MMD���͒l��256�Ŋ������l)
      float b;             // �Ɩ��F��(MMD���͒l��256�Ŋ������l)
      float x;             // �Ɩ�x�ʒu(MMD���͒l)
      float y;             // �Ɩ�y�ʒu(MMD���͒l)
      float z;             // �Ɩ�z�ʒu(MMD���͒l)
    };

    struct SelfShadow {
      std::uint32_t frame; // �t���[���ԍ�
      std::uint8_t type;   // �Z���t�V���h�E���, 0:OFF, 1:mode1, 2:mode2
      float distance;      // �V���h�E����(MMD���͒lL��(10000-L)/100000�Ƃ����l)
    };
    
    struct InfoIk {
      char name[20];       // "�E���h�j\0"�Ȃǂ�IK�{�[�����̕����� 20byte
      std::uint8_t on_off; // IK��on/off, 0:OFF, 1:ON
    };

    struct ShowIkWithoutArray {
      std::uint32_t frame;    // �t���[���ԍ�
      std::uint8_t show;      // ���f���\��, 0:OFF, 1:ON
      std::uint32_t ik_count; // �L�^����IK�̐�
      //InfoIK ik[ik_count];  // IK on/off���z��
    };

#pragma pack(pop)

    struct ShowIk : ShowIkWithoutArray {
      Array<InfoIk> ik;
    };

    /// 0�`1�ɋK�i�����ꂽ�x�W�F�Ȑ�
    class Bezie {

      Vec2 p1, p2; /// ����_

    public:

      Bezie() = default;
      Bezie(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);
      float GetY(float x) const;	/// x�ɂ�����y���擾

    };

    struct KeyFrame {

      std::string boneName; /// <summary>�{�[����</summary>
      int frameNo;          /// <summary>�t���[���ԍ�</summary>
      Vec3 position;        /// <summary>�ʒu</summary> 
      Quaternion rotation;  /// <summary>��]</summary>
      Bezie bezie_x;
      Bezie bezie_y;
      Bezie bezie_z;
      Bezie bezie_r;

      // �t���[���ԍ��Ŕ�r
      bool operator<(const KeyFrame &k) const {
        return frameNo < k.frameNo;
      }

    };
  }

  class VMDReader {

    int last_frame;
    std::map<std::string, std::shared_ptr<Array<vmd::KeyFrame>>> keyFrames;
  public:

    /// <summary>VMD�t�@�C������f�[�^�����o��</summary>
    /// <param name="file_name"></param>
    VMDReader(const FilePath &file_name);

    /// <summary>�{�[�����ɉ������L�[�t���[����Ԃ�</summary>
    /// <param name="bone_name">�{�[����</param>
    /// <returns>�L�[�t���[��</returns>
    std::shared_ptr<Array<vmd::KeyFrame>> getKeyFrames(const std::string& bone_name);

    /// <summary>�{�[�����ɉ������L�[�t���[����Ԃ�</summary>
    /// <param name="bone_name">�{�[����</param>
    /// <returns>�L�[�t���[��</returns>
    std::map<std::string, std::shared_ptr<Array<vmd::KeyFrame>>> getKeyFrames() {
      return keyFrames;
    }

    /// <summary>���[�V�����̍ŏI�t���[����Ԃ�</summary>
    /// <returns>���[�V�����̍ŏI�t���[��</returns>
    int GetLastFrame() { return last_frame; }

  };
}
