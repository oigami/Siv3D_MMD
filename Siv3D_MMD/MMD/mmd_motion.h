#pragma once
#include <MMD/vmd_reader.h>
#include <MMD/key_frame_data.h>
namespace s3d_mmd
{
  namespace mmd
  {

    template<class FrameData>
    class Frames
    {
      using iterator = typename Array<FrameData>::iterator;
      Array<FrameData> m_frames;

      bool isSorted;

      void sort();
      void push(const FrameData& data);
      std::pair<iterator, bool> getFrameImpl(int frameNo);

    public:

      Frames();

      Frames(Array<FrameData> data);


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
      std::pair<Optional<const FrameData&>, Optional<const FrameData&>> getSection(int frameNo);

      Optional<const FrameData&> getFrame(int frameNo);

      Array<FrameData>& frames();

      Array<FrameData> createFrames() const;
    };


    using MorphKeyFrame = key_frame::MorphFrame;
    using MorphFrames = Frames<MorphKeyFrame>;

    using CameraKeyFrame = vmd_struct::Camera;
    using CameraFrames = Frames<CameraKeyFrame>;

    using LightKeyFrame = vmd_struct::Light;
    using LightFrames = Frames<LightKeyFrame>;

    using SelfShadowKeyFrame = vmd_struct::SelfShadow;
    using SelfShadowFrames = Frames<SelfShadowKeyFrame>;

    using ShowIkKeyFrame = vmd_struct::ShowIk;
    using ShowIkFrames = Frames<ShowIkKeyFrame>;

    using BoneKeyFrame = key_frame::BoneFrame;
    using BoneFrames = Frames<BoneKeyFrame>;

    class MMDMotion
    {
    public:
      MMDMotion() = default;
      MMDMotion(const VMDReader& reader);

      std::unordered_map<String, BoneFrames>& bones() { return m_keyFrames; }

      std::unordered_map<String, Array<MorphKeyFrame>>& morph() { return m_morphs; }

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

      std::unordered_map<String, Array<MorphKeyFrame>> m_morphs;

      Array<CameraKeyFrame> m_cameras;

      Array<LightKeyFrame> m_lights;

      Array<SelfShadowKeyFrame> m_selfShadows;

      Array<ShowIkKeyFrame> m_showIks;

    };
  }
}
