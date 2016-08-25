#pragma once
#include <MMD/VMDController.h>
#include <MMD/PMDStruct.h>
#include <MMD/mmd_motion.h>
namespace s3d_mmd
{

  namespace vmd
  {
    namespace detail
    {
      struct Morph
      {
        uint32 frameNo;
        float m_weight;
        bool operator<(const Morph& m)const
        {
          return frameNo < m.frameNo;
        }
      };

      /// キーフレームアニメーション
      struct KeyFrameData
      {
        KeyFrameData() :m_nowFrameNum(0) {}

        const vmd::BoneFrame& getNowFrame() const
        {
          return m_keyFrames[m_nowFrameNum];
        }

        const vmd::BoneFrame& getNextFrame() const
        {
          return m_keyFrames[m_nowFrameNum + 1];
        }

        bool haveNextFrame() const
        {
          return m_nowFrameNum + 1 < m_keyFrames.size();
        }

        bool haveNowFrame() const
        {
          return m_nowFrameNum < m_keyFrames.size();
        }

        Array<vmd::BoneFrame> m_keyFrames;
        int m_nowFrameNum;
      };

      struct MorphData
      {
        const Morph& getNowFrame() const
        {
          return (m_morph)[m_nowFrameNum];
        }

        const Morph& getNextFrame() const
        {
          return (m_morph)[m_nowFrameNum + 1];
        }

        bool haveNextFrame() const
        {
          return m_nowFrameNum + 1 < m_morph.size();
        }

        bool haveNowFrame() const
        {
          return m_nowFrameNum < m_morph.size();
        }

        Array<Morph> m_morph;
        int m_nowFrameNum;
      };
    }
  }

  class VMD::Pimpl
  {

    static float msToFrameCount(int ms) { return ms * (60.0f / 1000); }

    bool m_isFrameEnd;  /// <summary> フレームが終了したか true:終了 </summary>
    bool m_isLoop;      /// <summary> ループするかどうか </summary>
    SecondsF m_loopBegin; /// <summary> ループする時間 </summary>
    SecondsF m_loopEnd;   /// <summary> ループする時間 </summary>
    Stopwatch m_nowTime;   /// <summary> 現在の再生時間 </summary>


    /// <summary> キーフレームの名前とデータ </summary>
    std::unordered_map<String, vmd::detail::KeyFrameData> m_keyFrameData;

    std::unordered_map<std::string, vmd::detail::MorphData> m_morphData;

  public:

    Pimpl(mmd::MMDMotion& data);
    Pimpl() = default;

    void UpdateIK(mmd::Bones &bones) const;		// IKボーン影響下ボーンの行列を更新
    void UpdateIK(mmd::Bones &bones, const mmd::Ik &ik) const;		// IKボーン影響下ボーンの行列を更新
    void UpdateBone(mmd::Bones &bons);
    void UpdateTime();
    void UpdateMorph(mmd::FaceMorph& m_morph)const;

    void play();

    void pause();

    void stop();

    bool isPlaying()const;

    bool isPaused()const;

    void setTime(MillisecondsF time);

    bool isLoop()const;

    void setLoop(bool loop);

    void setLoopBySec(bool loop, const SecondsF& loopBegin, const SecondsF& loopEnd = SecondsF::zero());

    void setPosSec(const SecondsF& pos);

    /// <summary>
    /// 再生位置を変更する（60fps)
    /// </summary>
    /// <param name="frameNo"></param>
    void SetPosFrame(const int frameNo);

    SecondsF lengthSec() const;
    SecondsF lengthFrame() const;
  };

}
