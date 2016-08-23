#pragma once
#include <MMD/VMDReader.h>
namespace s3d_mmd
{
  namespace mmd
  {

    using BoneFrame = vmd::BoneFrame;
    class BoneFrames
    {
      using FrameData = BoneFrame;
      using iterator = Array<FrameData>::iterator;
      Array<FrameData> m_frames;

      bool isSorted;

      void sort();
      void push(const FrameData& data);
      std::pair<iterator, bool> getFrameImpl(int frameNo);

    public:

      BoneFrames();

      BoneFrames(Array<FrameData> data);


      int size() const { return static_cast<int>(m_frames.size()); }

      int lastFrameNo() const;


      /// <summary>
      /// フレームを追加する
      /// </summary>
      /// <param name="data"></param>
      /// <returns>指定された時間にすでにフレームがあった場合に、そのフレームを返す</returns>
      Optional<FrameData> add(const FrameData& data);


      /// <summary>
      /// 指定された時間のフレームを削除する
      /// </summary>
      /// <param name="frameNo"></param>
      /// <returns>削除したかどうか</returns>
      bool remove(int frameNo);


      /// <summary>
      /// frameNo以下のフレームとframeNoより後のフレームを返す
      /// </summary>
      /// <param name="frameNo"></param>
      /// <returns></returns>
      std::pair<Optional<const BoneFrame&>, Optional<const BoneFrame&>> getSection(int frameNo);

      Optional<const BoneFrame&> getFrame(int frameNo);

      Array<FrameData> createFrames() const;
    };


    using MorphKeyFrame = vmd_struct::Morph;
    using CameraKeyFrame = vmd_struct::Camera;
    using LightKeyFrame = vmd_struct::Light;
    using SelfShadowKeyFrame = vmd_struct::SelfShadow;
    using ShowIkKeyFrame = vmd_struct::ShowIk;

    class MMDMotion
    {
    public:
      MMDMotion() = default;
      MMDMotion(const VMDReader& reader);

      const BoneFrames& getBoneFrames(const String& bone_name) const;
      BoneFrames& getBoneFrames(const String& bone_name) { return m_keyFrames[bone_name]; }

      std::unordered_map<String, BoneFrames>& getBoneFrames() { return m_keyFrames; }

      Array<MorphKeyFrame>& getMorphFrames() { return m_morphs; }

      void sort();

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

      bool saveVMD(const FilePath& filename);

    private:

      String m_modelName;

      std::unordered_map<String, BoneFrames> m_keyFrames;

      Array<MorphKeyFrame> m_morphs;

      Array<CameraKeyFrame> m_cameras;

      Array<LightKeyFrame> m_lights;

      Array<SelfShadowKeyFrame> m_selfShadows;

      Array<ShowIkKeyFrame> m_showIks;

    };
  }
}
