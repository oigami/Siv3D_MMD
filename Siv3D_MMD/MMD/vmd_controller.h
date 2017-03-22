#pragma once
#include <Siv3D.hpp>
#include <MMD/mmd_bone.h>
#include <MMD/mmd_morph.h>
#include <MMD/mmd_motion.h>

namespace s3d_mmd
{
  class VMD
  {
    class Pimpl;
    std::shared_ptr<Pimpl> m_handle;

  public:

    VMD(const FilePath& filename);
    VMD(mmd::MMDMotion& motion);
    VMD();
    ~VMD();

    void UpdateBone(mmd::Bones& bones) const;
    void UpdateMorph(mmd::FaceMorph& m_morph) const;

    void play() const;

    void pause() const;

    void stop() const;

    bool isPlaying() const;

    bool isPaused() const;

    bool isLoop() const;

    void setLoop(bool loop) const;

    void setLoopBySec(bool loop, const SecondsF& loopBegin, const SecondsF& loopEnd = SecondsF::zero()) const;

    void setPosSec(const SecondsF& pos) const;

    /// <summary>
    /// 再生位置を変更する（60fps)
    /// </summary>
    /// <param name="frameNo"></param>
    void SetPosFrame(const int frameNo);

    bool isEmpty() const;
  };
}
