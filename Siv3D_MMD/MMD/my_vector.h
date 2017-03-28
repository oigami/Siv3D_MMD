#pragma once
#include <Siv3D.hpp>

namespace s3d_mmd
{
  struct alignas(16) MyVector;

  namespace detail
  {
#ifdef _XM_VECTORCALL_
    using FMyVector = const MyVector;
    using FVec3 = const s3d::Vec3;
    using FVec4 = const s3d::Vec4;
#else
    using FMyVector = const MyVector&;
    using FVec3 = const s3d::Vec3&;
    using FVec4 = const s3d::Vec4&;
#endif // _XM_VECTORCALL_
  }

  struct alignas(16) MyVector
  {
    DirectX::XMVECTOR component;

    constexpr XM_CALLCONV operator DirectX::XMVECTOR() const
    {
      return component;
    }

    XM_CALLCONV operator DirectX::XMVECTOR&()
    {
      return component;
    }


    MyVector() {}

    constexpr MyVector(const MyVector& v)
      : component(v.component) { }

    constexpr MyVector(DirectX::FXMVECTOR v)
      : component(v) { }

    constexpr MyVector(float x, float y, float z, float w = 0.0f)
      : component({ x, y, z, w }) { }

    constexpr MyVector(detail::FVec4 v)
      : component(s3d::ToVector(v)) { }

    constexpr MyVector(detail::FVec3 v, double w)
      : component(s3d::ToVector(v, w)) { }

    bool isZero3() const
    {
      return DirectX::XMVector3Equal(component, DirectX::XMVectorZero());
    }

    //-----------------------------------------------
    //
    //  初期化関数
    //  すべてstaticで内部状態を変えるわけではないので注意
    //

    static MyVector XM_CALLCONV TransformCoord2(DirectX::FXMVECTOR v, DirectX::FXMMATRIX mat4x4)
    {
      return DirectX::XMVector2TransformCoord(v, mat4x4);
    }

    static MyVector XM_CALLCONV TransformCoord3(DirectX::FXMVECTOR v, DirectX::FXMMATRIX mat4x4)
    {
      return DirectX::XMVector3TransformCoord(v, mat4x4);
    }

    constexpr static MyVector XM_CALLCONV Zero()
    {
      return { 0.0f, 0.0f, 0.0f, 0.0f };
    }


    //-----------------------------------------------
    //
    //  内部状態を変える関数
    //

    MyVector& XM_CALLCONV normalize2()
    {
      component = DirectX::XMVector2Normalize(component);
      return *this;
    }

    MyVector& XM_CALLCONV normalize3()
    {
      component = DirectX::XMVector3Normalize(component);
      return *this;
    }

    MyVector& XM_CALLCONV normalize4()
    {
      component = DirectX::XMVector4Normalize(component);
      return *this;
    }

    MyVector& XM_CALLCONV setX(float x)
    {
      component = DirectX::XMVectorSetX(component, x);
      return *this;
    }

    MyVector& XM_CALLCONV setY(float y)
    {
      component = DirectX::XMVectorSetY(component, y);
      return *this;
    }

    MyVector& XM_CALLCONV setZ(float z)
    {
      component = DirectX::XMVectorSetZ(component, z);
      return *this;
    }

    MyVector& XM_CALLCONV setW(float w)
    {
      component = DirectX::XMVectorSetW(component, w);
      return *this;
    }


    //-----------------------------------------------
    //
    //  内部状態を変えない関数
    //

    MyVector XM_CALLCONV dot2(DirectX::FXMVECTOR v) const
    {
      return DirectX::XMVector2Dot(component, v);
    }

    MyVector XM_CALLCONV dot3(DirectX::FXMVECTOR v) const
    {
      return DirectX::XMVector3Dot(component, v);
    }

    MyVector XM_CALLCONV dot4(DirectX::FXMVECTOR v) const
    {
      return DirectX::XMVector4Dot(component, v);
    }

    MyVector XM_CALLCONV cross2(DirectX::FXMVECTOR v) const
    {
      return DirectX::XMVector2Cross(component, v);
    }

    MyVector XM_CALLCONV cross3(DirectX::FXMVECTOR v) const
    {
      return DirectX::XMVector3Cross(component, v);
    }

    MyVector XM_CALLCONV cross4(DirectX::FXMVECTOR v, DirectX::FXMVECTOR v2) const
    {
      return DirectX::XMVector4Cross(component, v, v2);
    }

    float XM_CALLCONV getX() const { return DirectX::XMVectorGetX(component); }

    float XM_CALLCONV getY() const { return DirectX::XMVectorGetY(component); }

    float XM_CALLCONV getZ() const { return DirectX::XMVectorGetZ(component); }

    float XM_CALLCONV getW() const { return DirectX::XMVectorGetW(component); }
  };
}
