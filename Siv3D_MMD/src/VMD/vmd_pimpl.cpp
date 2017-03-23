#include "vmd_pimpl.h"

namespace s3d_mmd
{
  void VMD::Pimpl::resetFrame()
  {
    auto nowFrameNo = getPosFrame();

    for ( auto& i : m_keyFrameData ) i.second.resetFrame(nowFrameNo);

    for ( auto& i : m_morphData ) i.second.resetFrame(nowFrameNo);
  }

  VMD::Pimpl::Pimpl(mmd::MMDMotion& data)
  {
    m_isEmpty = true;
    m_loopBegin = SecondsF::zero();
    m_loopEnd = SecondsF::zero();
    m_isLoop = false;
    m_isFrameEnd = true;

    for ( auto& i : data.bones() )
    {
      m_keyFrameData[i.first].m_keyFrames = i.second.createFrames();
      m_isEmpty = false;
    }

    for ( auto& i : data.morph() )
    {
      m_morphData[i.first].m_keyFrames = i.second;
      sort(m_morphData[i.first].m_keyFrames.begin(), m_morphData[i.first].m_keyFrames.end());
      m_isEmpty = false;
    }
  }

  void VMD::Pimpl::UpdateBone(mmd::Bones& bones)
  {
    UpdateTime();
    using namespace DirectX;
    for ( auto& i : bones )
    {
      const auto it = m_keyFrameData.find(i.name);
      if ( it == m_keyFrameData.end() ) continue;

      const auto& frame = it->second.calcFrame();

      // 親ボーン座標系のボーン行列を求める
      i.boneMat = frame.rotation.toMatrix() * DirectX::XMMatrixTranslationFromVector(frame.position) * i.initMat;
    }
  }

  void VMD::Pimpl::UpdateTime()
  {
    const auto frameCount = getPosFrame();

    // ループチェック
    if ( m_isLoop && m_loopEnd != SecondsF::zero() && m_nowTime.elapsed() >= m_loopEnd )
    {
      setPosSec(m_loopBegin);
      return;
    }

    for ( auto& i : m_keyFrameData ) i.second.updateFrame(frameCount);

    for ( auto& i : m_morphData ) i.second.updateFrame(frameCount);
  }

  void VMD::Pimpl::UpdateMorph(mmd::FaceMorph& morph) const
  {
    for ( auto& i : m_morphData )
    {
      if ( !i.second.haveNowFrame() ) continue;
      if ( auto index = morph.getFaceIndex(i.first) ) morph.setWeight(*index, i.second.calcFrame());
    }
  }

  void VMD::Pimpl::play()
  {
    m_nowTime.start();
  }

  void VMD::Pimpl::pause()
  {
    m_nowTime.pause();
  }

  void VMD::Pimpl::stop()
  {
    m_nowTime.reset();
    resetFrame();
  }

  bool VMD::Pimpl::isPlaying() const { return m_nowTime.isActive(); }

  bool VMD::Pimpl::isPaused() const { return m_nowTime.isPaused(); }

  bool VMD::Pimpl::isLoop() const { return m_isLoop; }

  void VMD::Pimpl::setLoop(bool loop)
  {
    m_isLoop = loop;
  }

  void VMD::Pimpl::setLoopBySec(bool loop, const SecondsF& loopBegin, const SecondsF& loopEnd)
  {
    m_isLoop = loop;
    m_loopBegin = loopBegin;
    m_loopEnd = loopEnd;
  }

  void VMD::Pimpl::setPosSec(const SecondsF& pos)
  {
    m_nowTime.set(pos);
    resetFrame();
  }

  void VMD::Pimpl::setPosFrame(const int frameNo)
  {
    m_nowTime.set(SecondsF(frameNo / 60.0));
    resetFrame();
  }

  int VMD::Pimpl::getPosFrame() { return m_nowTime.ms() * 60 / 1000; }

  bool VMD::Pimpl::isEmpty() const
  {
    return m_isEmpty;
  }
}
