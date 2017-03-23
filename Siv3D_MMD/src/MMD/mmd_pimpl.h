#pragma once
#include <MMD/MMD.h>
#include <src/mmd_physics.h>
#include "../../ShaderAttacher.h"
#include <MMD/my_vector.h>

namespace s3d_mmd
{
  namespace mmd
  {
    namespace
    {
      constexpr double alphaEps = 1e-9;

      struct Node
      {
        int indexStart;
        int indexCount;
        mmd::Material material;
      };

      class EdgeShader
      {
        constexpr static auto edgePath = L"Data/Shaders/MMD3DEdge.hlsl";

        static void Compile(const wchar* outPath, ShaderType type)
        {
          if ( !FileSystem::Exists(outPath) ) Shader::Compile(edgePath, outPath, type);
        }

      public:

        static void init()
        {
          if ( m_isInit ) return;

          {
            constexpr auto edgeVSCompiledPath = L"Data/Shaders/MMD3DEdge.vs";
            Compile(edgeVSCompiledPath, ShaderType::VS_4_0);
            m_vsEdge = VertexShader{ edgeVSCompiledPath };
            if ( m_vsEdge.isEmpty() ) Println(L"MMD vertex shader compile error");
          }

          {
            constexpr auto edgePSCompiledPath = L"Data/Shaders/MMD3DEdge.ps";
            Compile(edgePSCompiledPath, ShaderType::PS_4_0);
            m_psEdge = PixelShader{ edgePSCompiledPath };
            if ( m_psEdge.isEmpty() ) Println(L"MMD pixel shader compile error");
          }
          m_isInit = true;
        }

        static void release()
        {
          m_isInit = false;
          m_vsEdge.release();
          m_psEdge.release();
        }

        static const VertexShader& vs() { return m_vsEdge; }

        static const PixelShader& ps() { return m_psEdge; }

      private:

        static VertexShader m_vsEdge;
        static PixelShader m_psEdge;
        static bool m_isInit;
      };

      VertexShader EdgeShader::m_vsEdge;
      PixelShader EdgeShader::m_psEdge;
      bool EdgeShader::m_isInit = false;

      class MMDShader
      {
        constexpr static auto mmdShaderPath = L"Data/Shaders/MMD.hlsl";

        static void Compile(const wchar* outPath, ShaderType type)
        {
          if ( !FileSystem::Exists(outPath) )
          {
            Shader::Compile(mmdShaderPath, outPath, type);
          }
        }

      public:

        static void init()
        {
          if ( m_isInit ) return;

          {
            constexpr auto vsPath = L"Data/Shaders/MMD.vs";
            Compile(vsPath, ShaderType::VS_4_0);
            m_vs = VertexShader{ vsPath };
            if ( m_vs.isEmpty() ) Println(L"MMD VMD vertex shader compile error");
          }

          m_isInit = true;
        }

        static void release()
        {
          m_isInit = false;
          m_vs.release();
        }

        static const VertexShader& vs() { return m_vs; }

      private:

        static VertexShader m_vs;
        static bool m_isInit;
      };

      VertexShader MMDShader::m_vs;
      bool MMDShader::m_isInit = false;

      struct ConstantBoneData
      {
        Matrix bones[256];
      };

      struct ConstatnMorphWeightData
      {
        float m_weight[4096];
      };
    }
  }

  class MMD::Pimpl
  {
    void pushGPUData();
  public:

    Pimpl(const MMDModel& model, const physics3d::Physics3DWorld& world);

    void drawForward(const Mat4x4& worldMat);

    void drawEdge(double edgeSize, const Mat4x4& worldMat);

    void physicsUpdate();

    void update();
    void updateIK(const mmd::Ik& ikData);

    void attach(const VMD& vmd);

    const VMD& vmd() const;


    Array<mmd::Node> m_nodes;
    Array<mmd::Node> m_edges;
    String m_name;
    String m_comment;
    std::shared_ptr<mmd::Bones> m_bones;
    mmd::FaceMorph m_faceMorph;
    Texture m_vertexTexture;
    Array<Mat4x4> worlds;

    VMD m_vmd;

    MmdPhysics m_mmdPhysics;
    bool isPhysicsEnabled = true;
    Mesh m_mesh;
  };
}
