#pragma once
namespace s3d_mmd
{
  class MouseCamera3D
  {
  public:

    void update();

    void setCamera() const;

    s3d::Camera camera;

  private:
    void updateREvent();
    void updateLEvent();
    enum class NowState
    {
      LEvent, REvent, None,
    };
    NowState nowState = NowState::None;
  };

  inline void MouseCamera3D::update()
  {
    switch ( nowState )
    {
    case MouseCamera3D::NowState::LEvent:
      updateLEvent(); break;

    case MouseCamera3D::NowState::REvent:
      updateREvent(); break;

    case MouseCamera3D::NowState::None:

      if ( s3d::Input::MouseL.pressed )
        nowState = NowState::LEvent;
      else if ( s3d::Input::MouseR.pressed )
        nowState = NowState::REvent;
      else if ( s3d::Mouse::Wheel() != 0 )
        camera.pos += (camera.pos - camera.lookat).normalize() * s3d::Mouse::Wheel();
      break;
    }
  }

  inline void MouseCamera3D::setCamera() const
  {
    s3d::Graphics3D::SetCamera(camera);
  }

  inline void MouseCamera3D::updateREvent()
  {
    if ( s3d::Input::MouseR.released )
    {
      nowState = NowState::None;
      return;
    }

#if 1

    // カメラ自体を回転させる
    auto& target = camera.pos;
    const auto center = camera.lookat;

#else
    // 視点を回転させる
    auto& target = camera.lookat;
    const auto center = camera.pos;
#endif

    const DirectX::XMVECTOR targetVec = ToVector(target, 0.0f);
    const DirectX::XMVECTOR centerVec = ToVector(center, 0.0f);

    s3d::Float2 delta(s3d::Mouse::DeltaF());
    delta.y *= -1;
    delta = s3d::Math::Radians(delta);

    // 回転は基準点で行うのでターゲットのベクトルから基準点を引いて基準点が原点になるようにする
    const auto localTarget = DirectX::XMVectorSubtract(targetVec, centerVec);

    // ターゲットベクトルと上方向ベクトルの外積を取り上下方向の回転軸を計算する
    const DirectX::XMVECTOR axis = DirectX::XMVector3Cross(localTarget, ToVector(camera.up, 0.0f));

    // 上で計算した軸と回転量でクォータニオンを作り、残りの左右方向のクォータニオンをかける（左右方向の回転軸はy軸固定）
    const s3d::Quaternion qt = s3d::Quaternion(s3d::ToVec3(axis).normalize(), delta.y) * s3d::Quaternion(s3d::ToVec3(DirectX::g_XMIdentityR1).normalize(), delta.x);

    // ローカルで回転させて、原点をワールド座標に戻すために基準点を足す
    target = s3d::ToVec3(DirectX::XMVectorAdd(DirectX::XMVector3Rotate(localTarget, qt.component), centerVec));

    camera.up = s3d::ToVec3(DirectX::XMVector3Rotate(ToVector(camera.up, 0.0f), qt.component));
  }

  inline void MouseCamera3D::updateLEvent()
  {
    if ( s3d::Input::MouseL.released )
    {
      nowState = NowState::None;
      return;
    }

    s3d::Float3 delta(s3d::Mouse::DeltaF(), 0.0f);
    delta.x *= -1;

    // カメラ行列と差のベクトルと掛けて新しい座標を求める
    const auto newPos = camera.calcInvViewMatrix().transform(delta / 10);

    // 視点を同じ量だけ移動させる
    camera.lookat += newPos - camera.pos;

    camera.pos = newPos;
  }

}
