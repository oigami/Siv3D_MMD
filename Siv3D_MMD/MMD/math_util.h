#pragma once
namespace s3d_mmd
{
  namespace math
  {
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
        const float tmp = XMVectorGetZ(rot.r[0]);
        if ( IsGimballock(tmp) ) return false;
        y = -asinf(tmp);
        x = atan2f(XMVectorGetZ(rot.r[1]), XMVectorGetZ(rot.r[2]));
        z = atan2f(XMVectorGetY(rot.r[0]), XMVectorGetX(rot.r[0]));
        type = Type::XYZ;
        return true;
      }
      bool CreateYZX(const Mat4x4 &rot)
      {
        using namespace DirectX;
        const float tmp = XMVectorGetX(rot.r[1]);
        if ( IsGimballock(tmp) ) return false;
        z = -asinf(tmp);
        x = atan2f(XMVectorGetZ(rot.r[1]), XMVectorGetY(rot.r[1]));
        y = atan2f(XMVectorGetX(rot.r[2]), XMVectorGetX(rot.r[0]));
        type = Type::YZX;
        return true;
      }
      bool CreateZXY(const Mat4x4 &rot)
      {
        using namespace DirectX;
        const float tmp = XMVectorGetY(rot.r[2]);
        if ( IsGimballock(tmp) ) return false;
        x = -asinf(tmp);
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
            //if ( !CreateZXY(rot) ) // x が90度以内に制限されると膝が上がらなくなってしまう
            Println(L"error");
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
}
