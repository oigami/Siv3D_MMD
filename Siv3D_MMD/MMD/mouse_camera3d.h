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
    void endEvent();

    enum class NowState
    {
      LEvent,
      REvent,
      None,
    };

    s3d::Vec2 pressedPos;
    NowState nowState = NowState::None;

    s3d::Mat4x4 cameraMatrix;
  };

  inline void MouseCamera3D::update()
  {
    switch ( nowState )
    {
    case s3d_mmd::MouseCamera3D::NowState::LEvent:
      updateLEvent(); break;

    case s3d_mmd::MouseCamera3D::NowState::REvent:
      updateREvent(); break;

    case s3d_mmd::MouseCamera3D::NowState::None:

      if ( s3d::Input::MouseL.pressed )
      {
        pressedPos = Mouse::PosF();
        cameraMatrix = camera.calcInvViewMatrix();
        nowState = NowState::LEvent;
      }
      else if ( s3d::Input::MouseR.pressed )
      {
        pressedPos = Mouse::PosF();
        cameraMatrix = camera.calcInvViewMatrix();
        nowState = NowState::REvent;
      }
      else if ( Mouse::Wheel() != 0 )
      {
        camera.pos += (camera.pos - camera.lookat).normalize() * Mouse::Wheel();
      }
      break;

    }

  }

  inline void MouseCamera3D::setCamera() const
  {
    Graphics3D::SetCamera(camera);
  }

  inline void MouseCamera3D::updateREvent()
  {
    if ( s3d::Input::MouseR.released )
    {
      endEvent();
      return;
    }
    // 現在との差分を取る
#if 0 // カメラ自体を回転させる
    Float3 diff(Mouse::PosF() - *pressedPos, 0.0f);
    diff.y *= -1;
    const auto rot = s3d::Quaternion::RollPitchYaw(s3d::Math::Radians(0),
                                                   s3d::Math::Radians(diff.y),
                                                   s3d::Math::Radians(diff.x)).toMatrix();
    const auto newPos = DirectX::XMMatrixTranslationFromVector(cameraMatrix.r[3]) * rot;
    camera.pos = s3d::ToVec3(newPos.r[3]);
#else // 視点を回転させる

    Float3 diff(Mouse::PosF() - pressedPos, 0.0f);
    diff /= 5;
    const auto rot = s3d::Quaternion::RollPitchYaw(s3d::Math::Radians(0),
                                                   s3d::Math::Radians(diff.y) + s3d::Math::TwoPiF,
                                                   s3d::Math::Radians(diff.x)).toMatrix();


    // カメラを軸にして回転させる
    const auto newPos = Mat4x4::Translate(cameraMatrix.inverse().transform(camera.lookat)) * rot;

    // ワールド座標系に戻す
    camera.lookat = cameraMatrix.transform(ToVec3(newPos.r[3]));

    pressedPos = Mouse::PosF();
#endif
  }

  inline void MouseCamera3D::updateLEvent()
  {
    if ( s3d::Input::MouseL.released )
    {
      endEvent();
      return;
    }
    // 現在との差分を取る
    Float3 diff(Mouse::PosF() - pressedPos, 0.0f);
    diff.x *= -1;
    // カメラ行列と差のベクトルと掛けて新しい座標を求める
    const auto newPos = cameraMatrix.transform(diff / 10);

    // 視点を同じ量だけ移動させる
    camera.lookat += newPos - camera.pos;

    camera.pos = newPos;
  }

  inline void MouseCamera3D::endEvent()
  {
    nowState = NowState::None;
  }



}
