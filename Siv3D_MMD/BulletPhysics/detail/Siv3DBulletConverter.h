#pragma once

#include <Siv3D.hpp>
#include "BulletPhysicsDetail.h"
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>
namespace s3d_bullet {
  namespace bullet {
    namespace {

      /// <summary>
      /// Bullet形式とSiv3D形式の変換 
      /// </summary>
      /// <param name="v"> 変換するVECTOR3 </param>
      /// <remarks>
      /// z軸を反転しない
      /// </remarks>
      inline btVector3 ConvertFloat(const Float3 &v) {
        return btVector3(v.x, v.y, v.z);
      }

      /// <summary>
      /// Bullet形式とSiv3D形式の変換 
      /// </summary>
      /// <param name="v"> 変換するarray </param>
      /// <remarks>
      /// z軸を反転しない
      /// </remarks>
      inline btVector3 ConvertFloat(const std::array<float, 3> &v) {
        return btVector3(v[0], v[1], v[2]);
      };

      ///       /// <summary>
      /// Bullet形式とSiv3D形式の変換 
      /// </summary>
      /// <param name="v"> 変換するVECTOR3 </param>
      /// <returns> pOutのポインタ </returns>
      inline btVector3 ConvertVector(const Float3 &v) {
        const float x = static_cast<float>(v.x);
        const float y = static_cast<float>(v.y);
        const float z = static_cast<float>(-v.z);
        return btVector3(x, y, z);
      }

      /// <summary>
      /// Bullet形式とDirectX形式の変換 
      /// </summary>
      /// <param name="v"> 変換するbtVector3 </param>
      inline Float3 ConvertVector(const btVector3 &v) {
        return Float3(v.x(), v.y(), -v.z());
      }

      inline ColorF ConvertColor(const btVector3 &btColor) {
        return ColorF(btColor.x(), btColor.y(), btColor.z(), 1.f);
      }

      inline Quaternion ConvertQuaternion(const btQuaternion &q) {
        return Quaternion(-q.x(), -q.y(), q.z(), q.w());
      }

      /// <summary>
      /// Bullet形式とDirectX形式の変換 
      /// </summary>
      /// <param name="v"> 変換するbtVector3 </param>
      /// <returns> pOutのポインタ </returns>
      inline btTransform ConvertMatrix(const Matrix &m) {
        btTransform ret;
        // 鏡像変換＋転置 
#ifndef _XM_NO_INTRINSICS_
        DirectX::XMFLOAT4 r[4];
        for (auto& i : step(4))
          DirectX::XMStoreFloat4(r + i, m.r[i]);
        const btMatrix3x3 basis(
          r[0].x, r[1].x, -r[2].x,
          r[0].y, r[1].y, -r[2].y,
          -r[0].z, -r[1].z, r[2].z);
        ret.setOrigin(btVector3(r[3].x, r[3].y, -r[3].z));
#else
        const btMatrix3x3 basis(m._11, m._21, -m._31,
          m._12, m._22, -m._32,
          -m._13, -m._23, m._33);
        ret.setOrigin(btVector3(m._41, m._42, -m._43));
#endif // !_XM_NO_INTRINSICS_

        //btTransform bm;

        ret.setBasis(basis);
        return ret;
      }

      /// <summary>
      /// Bullet形式とDirectX形式の変換 
      /// </summary>
      /// <param name="v"> 変換するbtTransform </param>
      /// <returns> pOutのポインタ </returns>
      inline Matrix ConvertMatrix(const btTransform &t) {
        Matrix ret;
        const btMatrix3x3 basis = t.getBasis();
        const btVector3 R = basis.getColumn(0);
        const btVector3 U = basis.getColumn(1);
        const btVector3 L = basis.getColumn(2);
        const btVector3 P = t.getOrigin();
        // 鏡像変換＋転置 
#ifndef _XM_NO_INTRINSICS_
        ret.r[0] = DirectX::XMVectorSet(R.x(), R.y(), -R.z(), 0.f);
        ret.r[1] = DirectX::XMVectorSet(U.x(), U.y(), -U.z(), 0.f);
        ret.r[2] = DirectX::XMVectorSet(-L.x(), -L.y(), L.z(), 0.f);
        ret.r[3] = DirectX::XMVectorSet(P.x(), P.y(), -P.z(), 1.f);
#else
        ret._11 = R.x(), ret._12 = R.y(), ret._13 = -R.z(), ret._14 = 0.f;
        ret._21 = U.x(), ret._22 = U.y(), ret._23 = -U.z(), ret._24 = 0.f;
        ret._31 = -L.x(), ret._32 = -L.y(), ret._33 = L.z(), ret._34 = 0.f;
        ret._41 = P.x(), ret._42 = P.y(), ret._43 = -P.z(), ret._44 = 1.f;
#endif // !_XM_NO_INTRINSICS_

        return ret;
      }

    }
  }
}
