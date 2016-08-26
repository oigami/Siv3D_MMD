#pragma once
namespace s3d_mmd
{
  namespace mmd
  {


    namespace key_frame
    {
      struct IKeyFrameData
      {
        int frameNo; /// <summary>フレーム番号 60fps</summary>

        // フレーム番号で比較
        bool operator<(const IKeyFrameData& m)const
        {
          return frameNo < m.frameNo;
        }

      };


#pragma warning(disable:4324) // アラインメントのパディング報告の抑制
      struct BoneFrame : IKeyFrameData
      {
        BoneFrame() = default;
        String set(const vmd_struct::Bone& boneFrame);
        vmd_struct::Bone Convert(const String& boneName);
        Vector position;      /// <summary>位置</summary>
        Quaternion rotation;  /// <summary>回転</summary>
        math::Bezie bezie_x;
        math::Bezie bezie_y;
        math::Bezie bezie_z;
        math::Bezie bezie_r;

        struct CalcFrame
        {
          Vector position;
          Quaternion rotation;
        };

        CalcFrame calcFrame() const;

        CalcFrame calcFrame(int nowFrameNo, const BoneFrame& next) const;
      };
#pragma warning(default:4324)

      struct MorphFrame :IKeyFrameData
      {

        String set(const vmd_struct::Morph& morph);
        vmd_struct::Morph convert(const String& name) const;
        float m_weight;

        float calcFrame() const;

        float calcFrame(int nowFrameNo, const MorphFrame& next) const;

      };
    }

  }
}
