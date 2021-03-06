﻿#include <Siv3D.hpp>
#include <MMD/mmd_bone.h>
#include <MMD/pmd_struct.h>
#include <MMD/vmd.h>
#include <src/reader_helper.h>
#include <src/VMD/vmd_pimpl.h>
#include <MMD/mmd_motion.h>

namespace s3d_mmd
{
  VMD::VMD()
  {
    m_handle = std::make_shared<Pimpl>();
  }

  VMD::VMD(const FilePath& filename)
  {
    mmd::MMDMotion motion(filename);
    m_handle = std::make_shared<Pimpl>(motion);
  }

  VMD::VMD(mmd::MMDMotion& motion)
  {
    m_handle = std::make_shared<Pimpl>(motion);
  }

  VMD::~VMD() {}

  void VMD::UpdateBone(mmd::Bones& bones) const
  {
    m_handle->UpdateBone(bones);
  }

  void VMD::UpdateMorph(mmd::FaceMorph& m_morph) const
  {
    m_handle->UpdateMorph(m_morph);
  }

  bool VMD::isLoop() const
  {
    return m_handle->isLoop();
  }

  void VMD::setLoop(bool loop) const
  {
    return m_handle->setLoop(loop);
  }

  void VMD::setLoopBySec(bool loop, const SecondsF& loopBegin, const SecondsF& loopEnd) const
  {
    return m_handle->setLoopBySec(loop, loopBegin, loopEnd);
  }

  void VMD::setPosSec(const SecondsF& pos) const
  {
    return m_handle->setPosSec(pos);
  }

  void VMD::setPosFrame(const int frameNo) const
  {
    return m_handle->setPosFrame(frameNo);
  }

  bool VMD::isEmpty() const
  {
    return m_handle->isEmpty();
  }

  void VMD::play() const
  {
    m_handle->play();
  }

  void VMD::pause() const
  {
    return m_handle->pause();
  }

  void VMD::stop() const
  {
    return m_handle->stop();
  }

  bool VMD::isPlaying() const
  {
    return m_handle->isPlaying();
  }

  bool VMD::isPaused() const
  {
    return m_handle->isPaused();
  }
}
