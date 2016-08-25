#include <src/VMD/vmd_controller_pimpl.h>
#include <MMD/mmd_motion.h>
#include <MMD/math_util.h>
namespace s3d_mmd
{
  VMD::Pimpl::Pimpl(mmd::MMDMotion & data)
  {
    m_loopBegin = SecondsF::zero();
    m_loopEnd = SecondsF::zero();
    m_isLoop = false;
    m_isFrameEnd = true;
    for ( auto& i : data.getBoneFrames() )
    {
      m_keyFrameData[i.first].m_keyFrames = i.second.createFrames();
    }

    m_isFrameEnd = true;

    for ( auto& i : data.getMorphFrames() )
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
    Vector localEffectorPos = DirectX::g_XMZero;
    Vector localTargetPos = DirectX::g_XMZero;
    Mat4x4 targetMat = bones.calcBoneMatML(ikData.ik_bone_index);

    for ( int i = ikData.iterations - 1; i >= 0; i-- )
    {
      for ( auto& attentionIdx : ikData.ik_child_bone_index )
      {
        auto& bone = bones[attentionIdx];
        using namespace DirectX;
        Mat4x4 effectorMat = bones.calcBoneMatML(ikData.ik_target_bone_index);
        XMVECTOR Determinant;
        const auto invCoord = XMMatrixInverse(&Determinant, bones.calcBoneMatML(attentionIdx));
        localEffectorPos = XMVector3TransformCoord(effectorMat.r[3], invCoord); // 注目ボーン基準に変換
        localTargetPos = XMVector3TransformCoord(targetMat.r[3], invCoord);     // 注目ボーン基準に変換

        // エフェクタのローカル方向（注目ボーン基準）
        const Vector& localEffectorDir = XMVector3Normalize(localEffectorPos);
        // ターゲットのローカル方向（注目ボーン基準）
        const Vector& localTargetDir = XMVector3Normalize(localTargetPos);

        const float& p = XMVectorGetX(XMVector3Dot(localEffectorDir, localTargetDir));
        if ( p >= 1.0f ) continue;
        float angle = std::min(std::acos(std::max(p, -1.0f)), ikData.control_weight);

        Vector axis = XMVector3Cross(localEffectorDir, localTargetDir);
        if ( bone.name == L"左足" || bone.name == L"右足" )
          axis = XMVectorSetY(axis, 0.0f);

        if ( XMVector3Equal(axis, XMVectorZero()) ) continue;

        const Quaternion& rotation = DirectX::XMQuaternionRotationAxis(axis, angle);
        const XMMATRIX& xmboneMatBL = bone.boneMat;
        if ( bone.name == L"左ひざ" || bone.name == L"右ひざ" )
        {
          const Quaternion& rv = (rotation * DirectX::XMQuaternionRotationMatrix(xmboneMatBL)).normalize();
          math::EulerAngles eulerAngle(rv.toMatrix());
          eulerAngle.x = Clamp(eulerAngle.x, Radians(-180.f), Radians(-10.f));
          eulerAngle.y = 0;
          eulerAngle.z = 0;
          bone.boneMat = DirectX::XMMatrixMultiply(eulerAngle.CreateRot(), DirectX::XMMatrixTranslationFromVector(xmboneMatBL.r[3]));
        }
        else
        {
          bone.boneMat = rotation.toMatrix() * xmboneMatBL;
        }
      }
    }
  }

  void VMD::Pimpl::UpdateBone(mmd::Bones &bones)
  {
    UpdateTime();
    using namespace DirectX;
    for ( auto &i : bones )
    {
      i.boneMat = i.initMat;
      auto it = m_keyFrameData.find(i.name);
      if ( it == m_keyFrameData.end() ) continue;
      const auto &keyData = it->second;

      // キーフレーム補完
      if ( keyData.haveNowFrame() )
      {
        const auto &nowFrame = keyData.getNowFrame();
        if ( keyData.haveNextFrame() )
        {

          const vmd::BoneFrame &next = keyData.getNextFrame();
          //次のフレームとの間の位置を計算する
          const auto& p0 = nowFrame.position;
          const auto& p1 = next.position;
          const int& t0 = nowFrame.frameNo;
          const int& t1 = next.frameNo;
          const float& s = (float) (msToFrameCount(m_nowTime.ms()) - t0) / float(t1 - t0);
          const auto rot = Math::Slerp(nowFrame.rotation, next.rotation, next.bezie_r.GetY(s));
          auto bonePos = XMVectorMultiplyAdd(XMVectorSubtract(p1, p0), XMVectorSet(next.bezie_x.GetY(s), next.bezie_y.GetY(s), next.bezie_z.GetY(s), 0), p0);

          // 親ボーン座標系のボーン行列を求める
          i.boneMat = DirectX::XMMatrixAffineTransformation(DirectX::g_XMOne, DirectX::g_XMZero, rot.component, bonePos) * i.initMat;
        }
        else
        {
          // 親ボーン座標系のボーン行列を求める
          i.boneMat = DirectX::XMMatrixAffineTransformation(DirectX::g_XMOne, DirectX::g_XMZero, nowFrame.rotation.component, nowFrame.position) * i.initMat;
        }

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
      const size_t keyframe_size = keyData.m_keyFrames.size();
      int &nowFrameNum = keyData.m_nowFrameNum;
      if ( nowFrameNum + 1 < keyframe_size )
      {
        const Array<vmd::BoneFrame> &key_frame = keyData.m_keyFrames;
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
    for ( auto& i : m_keyFrameData )
    {
      i.second.m_nowFrameNum = 0;
    }
    for ( auto& i : m_morphData )
    {
      i.second.m_nowFrameNum = 0;
    }
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

  void VMD::Pimpl::SetPosFrame(const int frameNo)
  {
    m_nowTime.set(SecondsF(frameNo / 60.0));
  }

}
