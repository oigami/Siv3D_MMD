#include <MMD/mmd_motion.h>
namespace s3d_mmd
{
  namespace mmd
  {
    MMDMotion::MMDMotion(const VMDReader & reader)
    {
      constexpr int frame_rate = 60;     // 本プログラムのフレームレート
      constexpr int mmd_frame_rate = 30; // MMDのフレームレート

      for ( const auto& bone : reader.getKeyFrames() )
      {
        vmd::BoneFrame keyFrame;
        size_t endPos = 0;
        for ( int i = 0; i < sizeof(bone.boneName); i++ )
        {
          if ( bone.boneName[i] == L'\0' )break;
          endPos++;
        }
        keyFrame.boneName = Widen({ bone.boneName, endPos });
        keyFrame.frameNo = bone.frameNo;
        keyFrame.frameNo *= frame_rate / mmd_frame_rate;
        keyFrame.position = DirectX::XMVectorSet(bone.location.x, bone.location.y, bone.location.z, 0);
        keyFrame.rotation = Quaternion(bone.rotation.x, bone.rotation.y, bone.rotation.z, bone.rotation.w);
        keyFrame.bezie_x = vmd::Bezie(bone.x1.x, bone.y1.x, bone.x2.x, bone.y2.x);
        keyFrame.bezie_y = vmd::Bezie(bone.x1.y, bone.y1.y, bone.x2.y, bone.y2.y);
        keyFrame.bezie_z = vmd::Bezie(bone.x1.z, bone.y1.z, bone.x2.z, bone.y2.z);
        keyFrame.bezie_r = vmd::Bezie(bone.x1.w, bone.y1.w, bone.x2.w, bone.y2.w);
        m_keyFrames[keyFrame.boneName].push_back(std::move(keyFrame));
      }

      for ( auto& it : m_keyFrames )
      {
        auto &i = it.second;
        std::sort(i.begin(), i.end());
      }
    }

    const Array<BoneFrame>& MMDMotion::getKeyFrames(const String & bone_name) const
    {
      static const Array<BoneFrame> notFindBone;
      auto it = m_keyFrames.find(bone_name);
      if ( it == m_keyFrames.end() )return notFindBone;
      return it->second;
    }

    /// <summary>
    /// 最後のフレームの時間を返す
    /// </summary>
    /// <remarks>
    /// 計算量: O(N)
    /// <para> n := Boneの数 </para>
    /// </remarks>
    /// <returns></returns>

    int MMDMotion::getLastFrame() const
    {
      int lastFrame = 0;
      for ( auto& i : m_keyFrames )
      {
        auto& frame = i.second;
        if ( frame.size() )
          lastFrame = std::max(lastFrame, frame.back().frameNo);
      }
      return lastFrame;
    }

  }
}
