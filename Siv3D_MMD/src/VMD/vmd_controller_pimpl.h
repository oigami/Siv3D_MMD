#pragma once
#include <MMD/vmd_controller.h>
#include <MMD/pmd_struct.h>
#include <MMD/key_frame_data.h>
#include <MMD/mmd_motion.h>
namespace s3d_mmd
{

  namespace vmd
  {
    namespace detail
    {


      /// キーフレームアニメーション
      template<
        class T /* IKeyFrameData を継承
                   auto calcFrame() const;
                   auto calcFrame(int nowFrameNo, const T& next) const; があること */
      > struct KeyFrameData
      {
        KeyFrameData() :m_nowFrameIndex(0), m_nowFrameNo(0) {}

        const T& getNowFrame() const
        {
          return m_keyFrames[m_nowFrameIndex];
        }

        const T& getNextFrame() const
        {
          return m_keyFrames[m_nowFrameIndex + 1];
        }

        bool haveNextFrame() const
        {
          return m_nowFrameIndex + 1 < m_keyFrames.size();
        }

        bool haveNowFrame() const
        {
          return m_nowFrameIndex < m_keyFrames.size();
        }

        /// <summary>
        /// フレーム時間を進ませる
        /// <para> m_nowFrameIndex &lt; newFrameNo が成り立っていることが必須条件 </para>
        /// </summary>
        /// <remarks>
        /// 計算量 O(N) ただし前回の update から1しか変化がないとき O(1)
        /// <para> N := フレームの数 </para>
        /// </remarks>
        /// <param name="newFrameNo"> 新たなフレーム時間（現在のフレームに依存しない） </param>
        void updateFrame(int newFrameNo)
        {
          while ( haveNextFrame() )
          {
            const int t1 = getNextFrame().frameNo;
            if ( t1 < newFrameNo ) ++m_nowFrameIndex;
            else break;
          }
          m_nowFrameNo = newFrameNo;
        }

        /// <summary>
        /// フレーム時間を更新する。
        /// <para> updateFrame と違い、条件はなし </para>
        /// </summary>
        /// <remarks>
        /// 計算量 O(logN)
        /// <para> N := フレームの数 </para>
        /// </remarks>
        /// <param name="updateFrameNo"> 新たなフレーム時間（現在のフレームに依存しない） </param>
        void resetFrame(int newFrameNo)
        {
          auto& keyFrames = m_keyFrames;
          assert(keyFrames.size() != 0);

          mmd::key_frame::IKeyFrameData findFrame{ newFrameNo };
          auto frame = std::upper_bound(keyFrames.begin(), keyFrames.end(), findFrame);

          if ( frame != keyFrames.begin() ) --frame;

          m_nowFrameIndex = static_cast<int>(std::distance(keyFrames.begin(), frame));
          m_nowFrameNo = newFrameNo;
        }

        /// <summary>
        /// 現在のフレームの値を計算する
        /// </summary>
        /// <returns></returns>
        auto calcFrame() const
        {
          if ( haveNextFrame() )
            return m_keyFrames[m_nowFrameIndex].calcFrame(m_nowFrameNo, getNextFrame());
          return m_keyFrames[m_nowFrameIndex].calcFrame();
        }

        Array<T> m_keyFrames;
      private:
        int m_nowFrameIndex;
        int m_nowFrameNo;
      };

      using MorphData = KeyFrameData<mmd::key_frame::MorphFrame>;


    }
  }

  class VMD::Pimpl
  {


    bool m_isFrameEnd;  /// <summary> フレームが終了したか true:終了 </summary>
    bool m_isLoop;      /// <summary> ループするかどうか </summary>
    SecondsF m_loopBegin; /// <summary> ループする時間 </summary>
    SecondsF m_loopEnd;   /// <summary> ループする時間 </summary>
    Stopwatch m_nowTime;   /// <summary> 現在の再生時間 </summary>


    /// <summary> キーフレームの名前とデータ </summary>
    std::unordered_map<String, vmd::detail::KeyFrameData<mmd::BoneFrame>> m_keyFrameData;

    std::unordered_map<String, vmd::detail::MorphData> m_morphData;

    void resetFrame();
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

    bool isLoop()const;

    void setLoop(bool loop);

    void setLoopBySec(bool loop, const SecondsF& loopBegin, const SecondsF& loopEnd = SecondsF::zero());

    void setPosSec(const SecondsF& pos);

    /// <summary>
    /// 再生位置を変更する (60fps)
    /// </summary>
    /// <param name="frameNo"></param>
    void setPosFrame(const int frameNo);

    /// <summary>
    /// 現在の再生位置を取得する (60fps)
    /// </summary>
    /// <returns></returns>
    int getPosFrame();

    SecondsF lengthSec() const;
    SecondsF lengthFrame() const;
  };

}
