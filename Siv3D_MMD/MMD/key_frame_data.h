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

        CalcFrame calcFrame() const
        {
          return{ position, rotation };
        }

        CalcFrame calcFrame(int nowFrameNo, const BoneFrame& next) const
        {
          //次のフレームとの間の位置を計算する
          const auto& p0 = position;
          const auto& p1 = next.position;
          const int& t0 = frameNo;
          const int& t1 = next.frameNo;
          const float& s = (float) (nowFrameNo - t0) / float(t1 - t0);
          const auto rot = Math::Slerp(rotation, next.rotation, next.bezie_r.GetY(s));
          auto bonePos = DirectX::XMVectorMultiplyAdd(DirectX::XMVectorSubtract(p1, p0),
                                                      DirectX::XMVectorSet(next.bezie_x.GetY(s),
                                                                           next.bezie_y.GetY(s),
                                                                           next.bezie_z.GetY(s), 0), p0);
          return{ bonePos, rot };
        }
      };
#pragma warning(default:4324)

      struct MorphFrame :IKeyFrameData
      {
        float m_weight;

        float calcFrame() const
        {
          return m_weight;
        }

        float calcFrame(int nowFrameNo, const MorphFrame& next) const
        {
          const float& t = float(nowFrameNo - frameNo) / (next.frameNo - frameNo);
          return Math::Lerp(m_weight, next.m_weight, t);
        }

      };
    }

  }
}
