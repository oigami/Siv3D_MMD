#include <Siv3D.hpp>
#include <MMD/MMDBone.h>
#include <MMD/PMDStruct.h>
#include <MMD/VMDController.h>
#include "../ReaderHelper.h"
#include <src/VMD/vmd_controller_pimpl.h>
namespace s3d_mmd
{
  namespace vmd
  {
    Bezie::Bezie(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)
    {
      p1.x = x1 / 127.0f;
      p1.y = y1 / 127.0f;
      p2.x = x2 / 127.0f;
      p2.y = y2 / 127.0f;
    }

    /// http://d.hatena.ne.jp/edvakf/20111016/1318716097
    /// 二分探索
    float Bezie::GetY(float x) const
    {
      //return newton(0.5f, x);
      float t = 0.5f;
      float t1, t2, t3;

      constexpr int N = 16; // 計算繰り返し回数
      for ( int i = 0; i < N; ++i )
      {
        const float s = 1 - t;
        t1 = s * s * t * 3.0f;
        t2 = s * t * t * 3.0f;
        t3 = t * t * t;
        float ft = t1 * p1.x + t2 * p2.x + t3 - x;
        if ( fabs(ft) < 1e-6 ) break; // 誤差が定数以内なら終了
        if ( ft > 0 )
        { // 範囲を変更して再計算
          t -= 1.0f / (4 << i);
        }
        else
        {
          t += 1.0f / (4 << i);
        }
      }
      return t1 * p1.y + t2 * p2.y + t3;
    }
  }



  VMD::VMD()
  {
    m_handle = std::make_shared<Pimpl>();
  }

  VMD::VMD(const FilePath & filename)
  {
    m_handle = std::make_shared<Pimpl>(VMDReader(filename));
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

  void VMD::IsLoop(bool loop, int startTime) const
  {
    m_handle->setLoopBySec(loop, SecondsF(startTime));
  }

  void VMD::play() const
  {
    m_handle->play();
  }


}
