#pragma once

#include <Siv3D.hpp>
namespace s3d_mmd
{
  namespace
  {
    template<class T> class ShaderAttacher;
    template<> class ShaderAttacher<VertexShader>
    {
    public:

      ShaderAttacher(const VertexShader &shader)
      {
        isBegin = Graphics3D::BeginVS(shader);
      }

      ~ShaderAttacher()
      {
        if ( isBegin ) Graphics3D::EndVS();
      }

      operator bool() const { return isBegin; }

    private:
      bool isBegin;
    };

    template<> class ShaderAttacher<PixelShader>
    {
    public:

      ShaderAttacher(const PixelShader &shader)
      {
        isBegin = Graphics3D::BeginPS(shader);
      }

      ~ShaderAttacher()
      {
        if ( isBegin )Graphics3D::EndPS();
      }

      operator bool() const { return isBegin; }

    private:
      bool isBegin;
    };

    struct PSVSShaderAttacher
    {
      ShaderAttacher<PixelShader> ps_shader;
      ShaderAttacher<VertexShader> vs_shader;
      operator bool()const { return ps_shader || vs_shader; }
    };

    template<class T>
    ShaderAttacher<T> ShaderAttach(const T &shader)
    {
      return{ shader };
    }

    PSVSShaderAttacher ShaderAttach(const PixelShader &ps, const VertexShader&vs)
    {
      return{ ps, vs };
    }

    PSVSShaderAttacher ShaderAttach(const VertexShader&vs, const PixelShader &ps)
    {
      return{ ps, vs };
    }


    template<class T> class ShaderForwardAttacher;
    template<> class ShaderForwardAttacher<VertexShader>
    {
    public:

      ShaderForwardAttacher(const VertexShader &shader)
      {
        isBegin = Graphics3D::BeginVSForward(shader);
      }

      ~ShaderForwardAttacher()
      {
        if ( isBegin ) Graphics3D::EndVSForward();
      }

      operator bool() const { return isBegin; }

    private:
      bool isBegin;
    };
    template<> class ShaderForwardAttacher<PixelShader>
    {
    public:

      ShaderForwardAttacher(const PixelShader &shader)
      {
        isBegin = Graphics3D::BeginPSForward(shader);
      }

      ~ShaderForwardAttacher()
      {
        if ( isBegin )Graphics3D::EndPSForward();
      }

      operator bool() const { return isBegin; }

    private:
      bool isBegin;
    };
    template<class T>
    ShaderForwardAttacher<T> ShaderForwardAttach(const T &shader)
    {
      return{ shader };
    }
  }
}
