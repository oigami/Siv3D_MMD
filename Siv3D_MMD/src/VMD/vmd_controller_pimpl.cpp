﻿#include <src/VMD/vmd_controller_pimpl.h>
namespace s3d_mmd
{
  namespace detail
  {

    //https://github.com/mmd-for-unity-proj/mmd-for-unity/blob/master/.MMDIKBaker/MMDIKBakerLibrary/Misc/DefaultIKLimitter.cs#L204
    float IKRotateLimit(float val, float min, float max)
    {
      if ( min == max ) return min;
      if ( max < val )
      {
        return max * 2.f - val;
      }
      else if ( val < min )
      {
        return min * 2.f - val;
      }
      return val;
    }

    float Clamp(float val, float min, float max)
    {
      return std::min(std::max(min, val), max);
    }

    class EulerAngles
    {
      enum class Type
      {
        XYZ, YZX, ZXY
      };
      static bool IsGimballock(float val)
      {
        constexpr float eps = 1.0e-4f;
        if ( val < -1 + eps || 1 - eps < val )
        {
          return true;
        }
        return false;
      }
      bool CreateXYZ(const Mat4x4 &rot)
      {
        using namespace DirectX;
        y = -asinf(XMVectorGetZ(rot.r[0]));
        if ( IsGimballock(y) ) return false;
        x = atan2f(XMVectorGetZ(rot.r[1]), XMVectorGetZ(rot.r[2]));
        z = atan2f(XMVectorGetY(rot.r[0]), XMVectorGetX(rot.r[0]));
        type = Type::XYZ;
        return true;
      }
      bool CreateYZX(const Mat4x4 &rot)
      {
        using namespace DirectX;
        z = -asinf(XMVectorGetX(rot.r[1]));
        if ( IsGimballock(z) ) return false;
        x = atan2f(XMVectorGetX(rot.r[2]), XMVectorGetX(rot.r[0]));
        y = atan2f(XMVectorGetZ(rot.r[1]), XMVectorGetY(rot.r[1]));
        type = Type::YZX;
        return true;
      }
      bool CreateZXY(const Mat4x4 &rot)
      {
        using namespace DirectX;
        x = -asinf(XMVectorGetY(rot.r[2]));
        if ( IsGimballock(x) ) return false;
        y = atan2f(XMVectorGetX(rot.r[2]), XMVectorGetZ(rot.r[2]));
        z = atan2f(XMVectorGetY(rot.r[0]), XMVectorGetY(rot.r[1]));
        type = Type::ZXY;
        return true;
      }
      Mat4x4 CreateX() const
      {
        return DirectX::XMMatrixRotationX(x);
      }
      Mat4x4 CreateY() const
      {
        return DirectX::XMMatrixRotationY(y);
      }
      Mat4x4 CreateZ() const
      {
        return DirectX::XMMatrixRotationZ(z);
      }
      Type type;
    public:
      float x, y, z;


      EulerAngles(const Mat4x4 &rot)
      {
        if ( !CreateXYZ(rot) )
          if ( !CreateYZX(rot) )
            if ( !CreateZXY(rot) )Println(L"error");
      }

      Mat4x4 CreateRot() const
      {
        Mat4x4 rot;
        using namespace DirectX;
        switch ( type )
        {
        case Type::XYZ:
          return CreateX() * CreateY() * CreateZ();
        case Type::ZXY:
          return CreateZ() * CreateX() * CreateY();
        case Type::YZX:
          return CreateY() * CreateZ() * CreateX();
        }
        assert(0);
        return Mat4x4::Identity();
      }
    };

  }


  VMD::Pimpl::Pimpl(VMDReader & data)
  {
    m_loopBegin = SecondsF::zero();
    m_loopEnd = SecondsF::zero();
    m_isLoop = false;
    m_isFrameEnd = true;
    for ( auto& i : data.getKeyFrames() )
    {
      m_keyFrameData[i.first].m_keyFrames = i.second;
    }

    m_isFrameEnd = true;

    for ( auto& i : data.getMorph() )
    {
      m_morphData[i.name].m_morph.push_back(vmd::detail::Morph{ i.frameNo,i.m_weight });
    }
    for ( auto& i : m_morphData )
    {
      sort(i.second.m_morph.begin(), i.second.m_morph.end());
    }
  }

  // IKボーン影響下ボーンの行列を更新
  void VMD::Pimpl::UpdateIK(mmd::Bones &bones) const
  {
    for ( auto& ikData : bones.ikData() )
      UpdateIK(bones, ikData);
  }
  void VMD::Pimpl::UpdateIK(mmd::Bones &bones, const mmd::Ik &ikData) const
  {
    Vector localEffectorPos = DirectX::XMVectorSet(0, 0, 0, 0);
    Vector localTargetPos = DirectX::XMVectorSet(0, 0, 0, 0);
    Mat4x4 targetMat = bones.CalcBoneMatML(ikData.ik_bone_index);

    for ( int i : step_backward(ikData.iterations) )
    {
      for ( auto& attentionIdx : ikData.ik_child_bone_index )
      {
        using namespace DirectX;
        Mat4x4 effectorMat = bones.CalcBoneMatML(ikData.ik_target_bone_index);
        XMVECTOR Determinant;
        const Mat4x4 invCoord = XMMatrixInverse(&Determinant, bones.CalcBoneMatML(attentionIdx));
        localEffectorPos = XMVector3TransformCoord(effectorMat.r[3], invCoord); // 注目ボーン基準に変換
        localTargetPos = XMVector3TransformCoord(targetMat.r[3], invCoord);     // 注目ボーン基準に変換

                                                                                // エフェクタのローカル方向（注目ボーン基準）
        Vector localEffectorDir = XMVector3Normalize(localEffectorPos);

        // ターゲットのローカル方向（注目ボーン基準）
        Vector localTargetDir = XMVector3Normalize(localTargetPos);

        const auto findIt = bones[attentionIdx].name.indexOf(L"ひざ") != std::string::npos;

        float p;
        XMStoreFloat(&p, XMVector3Dot(localEffectorDir, localTargetDir));
        if ( p > 1 - 1.0e-8f ) continue; // 計算誤差により1を越えるとacos()が発散するので注意!
        const float limitAngle = ikData.control_weight;
        const float angle = detail::Clamp(acos(p), -limitAngle, limitAngle);

        if ( angle == 0.f ) continue;
        Vector axis = XMVector3Normalize(XMVector3Cross(localEffectorDir, localTargetDir));
        if ( XMVector3Equal(axis, XMVectorZero()) ) continue;

        XMMATRIX rotation = DirectX::XMMatrixRotationAxis(axis, angle);
        const XMMATRIX xmboneMatBL = bones[attentionIdx].boneMat;
        if ( findIt )
        {
          Mat4x4 local = rotation * bones[attentionIdx].boneMat;
          Vector rv = DirectX::XMQuaternionRotationMatrix(local);
          Quaternion rot(rv);
          rot.normalize();
          detail::EulerAngles eulerAngle(rot.toMatrix());

          eulerAngle.x = detail::IKRotateLimit(eulerAngle.x, Radians(-180.f), Radians(-10.f));
          eulerAngle.y = detail::IKRotateLimit(eulerAngle.y, Radians(0.f), Radians(0.f));
          eulerAngle.z = detail::IKRotateLimit(eulerAngle.z, Radians(0.f), Radians(0.f));
          local = eulerAngle.CreateRot();

          bones[attentionIdx].boneMat = local * XMMatrixTranslationFromVector(xmboneMatBL.r[3]);
        }
        else
        {
          bones[attentionIdx].boneMat = rotation * xmboneMatBL;
        }
      }
      constexpr float errToleranceSq = 0.000001f;
      using namespace DirectX;
      if ( DirectX::XMVectorGetY(DirectX::XMVector3LengthSq(localEffectorPos - localTargetPos)) < errToleranceSq )
      {
        return;
      }
    }
  }

  void VMD::Pimpl::UpdateBone(mmd::Bones &bones) const
  {
    using namespace DirectX;
    for ( auto &i : bones )
    {
      auto it = m_keyFrameData.find(i.name);
      if ( it == m_keyFrameData.end() ) continue;
      const auto &keyData = it->second;

      // キーフレーム補完
      if ( keyData.haveNowFrame() )
      {
        const auto &nowFrame = keyData.getNowFrame();
        const int t0 = nowFrame.frameNo;
        Quaternion boneRot = nowFrame.rotation;
        Vec3 bonePos = nowFrame.position;
        if ( keyData.haveNextFrame() )
        {

          //次のフレームとの間の位置を計算する
          const Vec3 &p0 = bonePos;
          const vmd::KeyFrame &next = keyData.getNextFrame();
          const int t1 = next.frameNo;
          const Vec3 &p1 = next.position;
          float s = (float) (msToFrameCount(m_nowTime.ms()) - t0) / float(t1 - t0);
          boneRot = Math::Slerp(boneRot, next.rotation, next.bezie_r.GetY(s));
          bonePos.x = p0.x + (p1.x - p0.x) * next.bezie_x.GetY(s);
          bonePos.y = p0.y + (p1.y - p0.y) * next.bezie_y.GetY(s);
          bonePos.z = p0.z + (p1.z - p0.z) * next.bezie_z.GetY(s);
        }

        // 親ボーン座標系のボーン行列を求める
        const XMMATRIX rot = boneRot.toMatrix();
        const Mat4x4 trans = Mat4x4::Translate(bonePos.x, bonePos.y, bonePos.z);
        i.boneMat = rot * trans * i.initMat;
      }
    }
    UpdateIK(bones);
  }

  void VMD::Pimpl::UpdateTime()
  {
    const auto frameCount = msToFrameCount(m_nowTime.ms());
    if ( m_isLoop )
    {
      if ( m_loopEnd != SecondsF::zero() && m_nowTime.elapsed() >= m_loopEnd )
      {
        for ( auto& i : m_keyFrameData )
        {
          i.second.m_nowFrameNum = 0;
        }
        for ( auto& i : m_morphData )
        {
          i.second.m_nowFrameNum = 0;
        }
        m_nowTime.set(m_loopBegin);
      }
    }
    for ( auto& i : m_keyFrameData )
    {
      vmd::detail::KeyFrameData &keyData = i.second;
      const size_t keyframe_size = keyData.m_keyFrames->size();
      int &nowFrameNum = keyData.m_nowFrameNum;
      if ( nowFrameNum + 1 < keyframe_size )
      {
        const Array<vmd::KeyFrame> &key_frame = *keyData.m_keyFrames;
        const uint32 t1 = key_frame[nowFrameNum + 1].frameNo;
        if ( frameCount > t1 ) ++nowFrameNum;
      }
    }
    for ( auto& i : m_morphData )
    {
      auto& data = i.second;
      const size_t size = data.m_morph.size();
      int &nowFrameNum = data.m_nowFrameNum;
      if ( data.haveNextFrame() )
      {
        const auto &frame = data.m_morph;
        const uint32 t1 = frame[nowFrameNum + 1].frameNo;
        if ( frameCount > t1 ) ++nowFrameNum;
      }
    }
  }

  void VMD::Pimpl::UpdateMorph(mmd::FaceMorph & morph) const
  {
    for ( auto& i : m_morphData )
    {
      if ( !i.second.haveNowFrame() )continue;
      if ( auto index = morph.getFaceIndex(Widen(i.first)) )
      {
        auto now = i.second.getNowFrame();
        float w = now.m_weight;
        if ( i.second.haveNextFrame() )
        {
          auto& next = i.second.getNextFrame();
          float t = float(msToFrameCount(m_nowTime.ms()) - now.frameNo) / (next.frameNo - now.frameNo);
          w = Math::Lerp(w, next.m_weight, t);
        }
        morph.setWeight(*index, w);
      }
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
  }

  bool VMD::Pimpl::isPlaying() const { return m_nowTime.isActive(); }

  bool VMD::Pimpl::isPaused() const { return m_nowTime.isPaused(); }

  void VMD::Pimpl::setTime(MillisecondsF time)
  {
    m_nowTime.set(time);
  }

  bool VMD::Pimpl::isLoop() const { return m_isLoop; }

  void VMD::Pimpl::setLoop(bool loop)
  {
    m_isLoop = loop;
  }

  void VMD::Pimpl::setLoopBySec(bool loop, const SecondsF & loopBegin, const SecondsF & loopEnd)
  {
    m_isLoop = loop;
    m_loopBegin = loopBegin;
    m_loopEnd = loopEnd;
  }

  void VMD::Pimpl::setPosSec(const SecondsF & pos)
  {
    m_nowTime.set(pos);
  }

}
