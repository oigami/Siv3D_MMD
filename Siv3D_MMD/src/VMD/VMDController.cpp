#include <Siv3D.hpp>
#include <MMD/MMDBone.h>
#include <MMD/PMDStruct.h>
#include <MMD/VMDController.h>
#include "../ReaderHelper.h"
#include <src/VMD/vmd_controller_pimpl.h>
#include <MMD/mmd_motion.h>
namespace s3d_mmd
{
  namespace vmd
  {
    Bezie::Bezie(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)
    {
      p1.x = x1 / 127.0f * 3.0f;
      p1.y = y1 / 127.0f * 3.0f;
      p2.x = x2 / 127.0f * 3.0f;
      p2.y = y2 / 127.0f * 3.0f;
      pre_x = 0.0f;
      pre_out = 0.0f;
    }


    /// http://d.hatena.ne.jp/edvakf/20111016/1318716097
    /// 二分探索

    float Bezie::GetY(float x) const
    {
      //return newton(0.5f, x);
      float l, r;
      if ( pre_x <= x )
      {
        l = pre_out;
        r = 1.0f;
      }
      else
      {
        l = 0.0f;
        r = pre_out;
      }
      float t, s;
      constexpr int N = 16; // 計算繰り返し回数
      for ( int i = N - 1; i >= 0; --i )
      {
        t = (l + r) / 2.0f;
        s = 1.0f - t;
        // s * s * t * 3 * p1.x + s * t * t * 3 * p2.x + t * t * t
        float ft = t * (s * (s * p1.x + t * p2.x) + t * t);
        //if ( fabs(ft) < 1e-6 ) break; // 誤差が定数以内なら終了
        if ( ft > x )
          r = t;
        else
          l = t;
      }
      pre_x = x;
      pre_out = l;
      return s * (t * (s * p1.y + t * p2.y)) + t * t * t;
    }

    float Bezie::newton(float t, float x) const
    {
      auto  f = [&](float t, float x)
      {
        return 3 * (1 - t) * (1 - t) * t * p1.x + 3 * (1 - t) * t * t * p2.x + t * t * t - x;
      };
      auto fd = [&](float t)
      {
        return 3 * t * t * (3 * (p1.x - p2.x) + 1) + 6 * t * (p2.x - 2 * p1.x) + 3 * p1.x;
      };
      constexpr int N = 16;
      for ( int i = N - 1; i >= 0; --i )
      {
        float t1 = t - f(t, x) / fd(t);
        if ( fabs(t1 - t) < 1e-6 ) break;
        t = t1;
      }
      return t;
    }
  }



  VMD::VMD()
  {
    m_handle = std::make_shared<Pimpl>();
  }

  VMD::VMD(const FilePath & filename)
  {
    mmd::MMDMotion motion(filename);
    m_handle = std::make_shared<Pimpl>(motion);
  }

  VMD::VMD(mmd::MMDMotion & motion)
  {
    m_handle = std::make_shared<Pimpl>(motion);
  }

  VMD::~VMD() {}

  void VMD::UpdateBone(mmd::Bones &bones) const
  {
    m_handle->UpdateBone(bones);
  }

  void VMD::UpdateMorph(mmd::FaceMorph & m_morph) const
  {
    m_handle->UpdateMorph(m_morph);
  }

  void VMD::UpdateTime() const
  {
    m_handle->UpdateTime();
  }

  void VMD::setTime(MillisecondsF time) const
  {
    m_handle->setTime(time);
    UpdateTime();
  }

  bool VMD::isLoop() const
  {
    return m_handle->isLoop();
  }

  void VMD::setLoop(bool loop) const
  {
    return m_handle->setLoop(loop);
  }

  void VMD::setLoopBySec(bool loop, const SecondsF & loopBegin, const SecondsF & loopEnd) const
  {
    return m_handle->setLoopBySec(loop, loopBegin, loopEnd);
  }

  void VMD::setPosSec(const SecondsF & pos) const
  {
    return m_handle->setPosSec(pos);
  }

  void VMD::SetPosFrame(const int frameNo)
  {
    return m_handle->SetPosFrame(frameNo);
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
