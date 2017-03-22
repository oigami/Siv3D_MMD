#include <MMD/math_util.h>

namespace s3d_mmd
{
  namespace math
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
        if ( ft > x ) r = t;
        else l = t;
      }
      pre_x = x;
      pre_out = l;
      return s * (t * (s * p1.y + t * p2.y)) + t * t * t;
    }

    DirectX::XMVECTOR Bezie::newton(float _x,
                                    const Bezie& x,
                                    const Bezie& y,
                                    const Bezie& z,
                                    const Bezie& r
    )
    {
      using namespace DirectX;
      const DirectX::XMVECTOR p1_3x = { x.p1.x, y.p1.x, z.p1.x, r.p1.x };
      const DirectX::XMVECTOR p2_3x = { x.p2.x, y.p2.x, z.p2.x, r.p2.x };
      constexpr DirectX::XMVECTOR twoNegative{ -2, -2, -2, -2 };
      const DirectX::XMVECTOR b = DirectX::XMVectorMultiplyAdd(p1_3x, twoNegative, p2_3x);
      const DirectX::XMVECTOR c = p1_3x - p2_3x + DirectX::g_XMOne;
      const DirectX::XMVECTOR d = c * 3;
      const DirectX::XMVECTOR e = p2_3x * DirectX::g_XMTwo - p1_3x * DirectX::g_XMFour;
      const DirectX::XMVECTOR xx{ _x, _x, _x, _x };
      DirectX::XMVECTOR t1{ 0.5f, 0.5f, 0.5f, 0.5f };
      DirectX::XMVECTOR t2 = t1 * t1;

      constexpr int N = 16;

      for ( int i = N - 1; i >= 0; --i )
      {
        using DirectX::XMVectorMultiplyAdd;
        t1 = t1
          - (t1 * XMVectorMultiplyAdd(t1, b, XMVectorMultiplyAdd(t2, c, p1_3x)) - xx)
          / XMVectorMultiplyAdd(t2, d, XMVectorMultiplyAdd(t1, e, p1_3x));
        t2 = t1 * t1;
      }

      const DirectX::XMVECTOR p1_3y{ x.p1.y, y.p1.y, z.p1.y, r.p1.y };
      const DirectX::XMVECTOR p2_3y{ x.p2.y, y.p2.y, z.p2.y, r.p2.y };
      return t1 * DirectX::XMVectorMultiplyAdd(t1, p2_3y + twoNegative * p1_3y, DirectX::XMVectorMultiplyAdd(t2, p1_3y - p2_3y + DirectX::g_XMOne, p1_3y));
    }
  }
}
