#pragma once
#include <MMD/VMDReader.h>
namespace s3d_mmd
{
  namespace mmd
  {
    using BoneFrame = vmd::BoneFrame;
    using MorphKeyFrame = vmd::Morph;
    class MMDMotion
    {
    public:
      MMDMotion() = default;
      MMDMotion(const VMDReader& reader);

      const Array<BoneFrame>& getKeyFrames(const String& bone_name) const;
      Array<BoneFrame>& getKeyFrames(const String& bone_name) { return m_keyFrames[bone_name]; }

      std::unordered_map<String, Array<BoneFrame>>& getKeyFrames() { return m_keyFrames; }

      Array<MorphKeyFrame>& getMorphFrames() { return m_morphs; }


      /// <summary>
      /// 最後のフレームの時間を返す
      /// </summary>
      /// <remarks>
      /// 計算量: O(N + M)
      /// <para> N := Boneの数 </para>
      /// <para> M := morphの数 </para>
      /// </remarks>
      /// <returns></returns>
      int getLastFrame() const;

    private:

      Array<MorphKeyFrame> m_morphs;

      std::unordered_map<String, Array<BoneFrame>> m_keyFrames;
    };
  }
}
