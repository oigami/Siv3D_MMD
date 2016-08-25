#include <src/VMD/vmd_controller_pimpl.h>
#include <MMD/mmd_motion.h>
#include <MMD/math_util.h>
namespace s3d_mmd
{
  void VMD::Pimpl::resetFrame()
  {
    auto nowFrameNo = msToFrameCount(m_nowTime.ms());

    for ( auto& i : m_keyFrameData )
      i.second.resetFrame(nowFrameNo);

    for ( auto& i : m_morphData )
      i.second.resetFrame(nowFrameNo);

  }

  VMD::Pimpl::Pimpl(mmd::MMDMotion & data)
  {
    m_loopBegin = SecondsF::zero();
    m_loopEnd = SecondsF::zero();
    m_isLoop = false;
    m_isFrameEnd = true;
    for ( auto& i : data.getBoneFrames() )
      m_keyFrameData[i.first].m_keyFrames = i.second.createFrames();

    for ( auto& i : data.getMorphFrames() )
    {
      m_morphData[i.first].m_keyFrames = i.second;
      sort(m_morphData[i.first].m_keyFrames.begin(), m_morphData[i.first].m_keyFrames.end());
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

      const auto& frame = it->second.calcFrame();

      // 親ボーン座標系のボーン行列を求める
      i.boneMat = DirectX::XMMatrixAffineTransformation(DirectX::g_XMOne, DirectX::g_XMZero,
                                                        frame.rotation.component, frame.position) * i.initMat;

    }
    UpdateIK(bones);
  }

  void VMD::Pimpl::UpdateTime()
  {
    const auto frameCount = msToFrameCount(m_nowTime.ms());

    // ループチェック
    if ( m_isLoop && m_loopEnd != SecondsF::zero() && m_nowTime.elapsed() >= m_loopEnd )
      setPosSec(m_loopBegin);

    for ( auto& i : m_keyFrameData )
      i.second.updateFrame(frameCount);

    for ( auto& i : m_morphData )
      i.second.updateFrame(frameCount);

  }

  void VMD::Pimpl::UpdateMorph(mmd::FaceMorph & morph) const
  {
    for ( auto& i : m_morphData )
    {
      if ( !i.second.haveNowFrame() ) continue;
      if ( auto index = morph.getFaceIndex(i.first) )
        morph.setWeight(*index, i.second.calcFrame());
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

  void VMD::Pimpl::setLoopBySec(bool loop, const SecondsF & loopBegin, const SecondsF & loopEnd)
  {
    m_isLoop = loop;
    m_loopBegin = loopBegin;
    m_loopEnd = loopEnd;
  }

  void VMD::Pimpl::setPosSec(const SecondsF & pos)
  {
    m_nowTime.set(pos);
    resetFrame();
  }

  void VMD::Pimpl::SetPosFrame(const int frameNo)
  {
    m_nowTime.set(SecondsF(frameNo / 60.0));
    resetFrame();
  }

}
