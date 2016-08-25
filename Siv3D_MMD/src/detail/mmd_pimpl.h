﻿#pragma once
#include <MMD/MMD.h>
#include "../../ShaderAttacher.h"
namespace s3d_mmd
{
  namespace mmd
  {
    namespace
    {
      constexpr double alphaEps = 1e-9;
      struct Node
      {
        Mesh mesh;
        mmd::Material material;
      };

      struct ModelNodeTex
      {
        Array<mmd::Node> nodeNotTex;
        Array<mmd::Node> nodeWithTex;
        void set(const mmd::Node &node)
        {
          if ( node.material.diffuseTextureName.isEmpty ) nodeNotTex.push_back(node);
          else
          {
            nodeWithTex.push_back(node);
            TextureAsset::Register(node.material.diffuseTextureName, node.material.diffuseTextureName);
          }
        }
      };

      struct ModelNodeAlpha
      {
        ModelNodeTex nodeNotAlpha;
        ModelNodeTex nodeWithAlpha;

        void set(const mmd::Node &node)
        {
          if ( node.material.diffuse.a + alphaEps < 1.0 ) nodeWithAlpha.set(node);
          else nodeNotAlpha.set(node);
        }

      };

      struct AllNode
      {
        ModelNodeAlpha nodeCullBack;
        ModelNodeAlpha nodeCullNone;

        void set(const mmd::Node &node)
        {
          if ( node.material.isCullNone ) nodeCullNone.set(node);
          else nodeCullBack.set(node);
        }

      };

      class EdgeShader
      {
        constexpr static auto edgePath = L"Data/Shaders/MMD3DEdge.hlsl";
        static void Compile(const wchar *outPath, ShaderType type)
        {
          if ( !FileSystem::Exists(outPath) )
            Shader::Compile(edgePath, outPath, type);
        }
      public:

        static void init()
        {
          if ( m_isInit ) return;

          {
            constexpr auto edgeVSCompiledPath = L"Data/Shaders/MMD3DEdge.vs";
            Compile(edgeVSCompiledPath, ShaderType::VS_4_0);
            m_vsEdge = VertexShader{ edgeVSCompiledPath };
            if ( m_vsEdge.isEmpty() )
              Println(L"MMD vertex shader compile error");
          }

          {
            constexpr auto edgePSCompiledPath = L"Data/Shaders/MMD3DEdge.ps";
            Compile(edgePSCompiledPath, ShaderType::PS_4_0);
            m_psEdge = PixelShader{ edgePSCompiledPath };
            if ( m_psEdge.isEmpty() )
              Println(L"MMD pixel shader compile error");
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
        static void Compile(const wchar *outPath, ShaderType type)
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
            if ( m_vs.isEmpty() )
              Println(L"MMD VMD vertex shader compile error");
          }

          {
            constexpr auto psPath = L"Data/Shaders/MMD.ps";
            Compile(psPath, ShaderType::VS_4_0);
            m_ps = PixelShader{ psPath };
            if ( m_ps.isEmpty() )
              Println(L"MMD VMD pixel shader compile error");

          }
          m_isInit = true;
        }

        static void release()
        {
          m_isInit = false;
          m_vs.release();
          m_ps.release();
        }

        static const VertexShader& vs() { return m_vs; }
        static const PixelShader& ps() { return m_ps; }

      private:

        static VertexShader m_vs;
        static PixelShader m_ps;
        static bool m_isInit;
      };

      VertexShader MMDShader::m_vs;
      PixelShader MMDShader::m_ps;
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
  public:

    Float2 WriteTextureVertex(ImageRGBA32F &vertexImage, const mmd::MeshVertex &v, int &vPos, Array<Float4> morph)
    {
      int x = vPos % 1016;
      int y = vPos / 1016;
      assert(y < 1016);
      int resize = morph.size() % 16;
      if ( resize != 0 )
        morph.resize(morph.size() + 16 - resize);
      const int dataSize = (3 + morph.size() + 1);
      if ( 1024 < x + dataSize )
      {
        vPos += -x + vertexImage.width;
        x = 0;
        y++;
      }

      auto image = vertexImage[y];
      auto asFloat = [](int a) { return *reinterpret_cast<float*>(&a); };
      assert(v.boneNum[0] < 256 && v.boneNum[1] < 256);
      image[x] = RGBA32F(asFloat(v.boneNum[0]), asFloat(v.boneNum[1]), 0.0f, 0.0f);
      image[x + 1] = RGBA32F(v.boneWeight.x, v.boneWeight.y, v.boneWeight.z, v.boneWeight.w);
      int f = morph.size();
      image[x + 2] = RGBA32F(v.texcoord.x, v.texcoord.y, asFloat(f), 0.0f);

      int now = x + 2;
      for ( auto& i : morph )
      {
        image[++now] = RGBA32F(i.x, i.y, i.z, asFloat(static_cast<int>(i.w)));
      }
      vPos += dataSize;
      return{ asFloat(x), asFloat(y) };
    }

    Pimpl(const MMDModel& model)
#ifdef USE_BULLET_PHYSICS
      : m_mmdPhysics(s3d_bullet::getBulletPhysics())
#endif
    {
      m_bones = model.bones();
      if ( m_bones->size() >= 256 )
      {
        Println(L"ボーンのサイズが多すぎる");
        return;
      }
      m_name = model.name();
      m_comment = model.comment();
#ifdef USE_BULLET_PHYSICS
      m_mmdPhysics.Create(m_bones, model.rigidBodies(), model.joints());
#endif
      ImageRGBA32F vertexImage(1024, 1024);
      struct FaceMorph
      {
        Array<Float4> vertex;
      };
      std::unordered_map<int, FaceMorph> faceMorphVertex;

      {
        struct BaseFaceMorph
        {
          int num;
          Array<int> index;
        };
        Array<int> baseMorph;
        // 表情モーフの設定
        std::unordered_map<String, int> faceIndex(model.skinData().size());
        int index = 0;
        for ( auto& i : model.skinData() )
        {
          // baseの時はindexのみ配列化する
          if ( i.skin_type == 0 )
          {
            for ( auto& j : i.skin_vert_data )
            {
              baseMorph.push_back(j.skin_vert_index);
            }
            continue;
          }
          // 通常のタイプの時はbaseのindexを使い頂点indexをグローバルな方に戻す
          for ( auto& j : i.skin_vert_data )
          {
            faceMorphVertex[baseMorph[j.skin_vert_index]].vertex.push_back({ j.skin_vert_pos,
                                                                            static_cast<float>(index) });
          }
          faceIndex.insert({ Widen(i.skin_name), index });
          index++;
        }
        m_faceMorph = mmd::FaceMorph(std::move(faceIndex));
      }


      // メッシュの生成
      int vPos = 0;
      mmd::AllNode nodes;
      for ( auto& node : model.nodes() )
      {
        const int size = static_cast<int>(node.mesh.vertices.size());
        Array<MeshVertex> vertex(size);
        for ( auto &i : step(size) )
        {
          const mmd::MeshVertex &v = node.mesh.vertices[i];
          auto it = faceMorphVertex.find(v.vertexNum);
          Float2 pos = WriteTextureVertex(vertexImage, v, vPos, it != faceMorphVertex.end() ? it->second.vertex : Array<Float4>{});
          vertex[i] = { v.position , v.normal, pos };
        }
        nodes.set(mmd::Node{ Mesh({ vertex, node.mesh.indices }), node.material });
      }
      m_nodes = std::move(nodes);


      // エッジの生成
      m_edges.reserve(model.edgeNodes().size());
      for ( auto& node : model.edgeNodes() )
      {
        const int size = static_cast<int>(node.mesh.vertices.size());
        Array<MeshVertex> vertex(size);
        for ( auto& i : step(size) )
        {
          mmd::MeshVertex &v = node.mesh.vertices[i];
          auto it = faceMorphVertex.find(v.vertexNum);
          Float2 pos = WriteTextureVertex(vertexImage, v, vPos, it != faceMorphVertex.end() ? it->second.vertex : Array<Float4>{});
          vertex[i] = { v.position, v.normal, pos };
        }
        m_edges.push_back(mmd::Node{ Mesh({ vertex, node.mesh.indices }), node.material });
      }
      m_vertexTexture = Texture(vertexImage);
    }

    static void draw(const mmd::ModelNodeTex &nodes)
    {
      for ( auto& node : nodes.nodeNotTex )
      {
        Graphics3D::SetAmbientLight(node.material.ambient);
        node.mesh.draw(node.material.diffuse);
      }
      for ( auto& node : nodes.nodeWithTex )
      {
        Graphics3D::SetAmbientLight(node.material.ambient);
        node.mesh.draw(TextureAsset(node.material.diffuseTextureName), node.material.diffuse);
      }
    }

    static void drawForward(const mmd::ModelNodeTex &nodes)
    {
      for ( auto& node : nodes.nodeNotTex )
      {
        Graphics3D::SetAmbientLightForward(node.material.ambient);
        node.mesh.drawForward(node.material.diffuse);
      }
      for ( auto& node : nodes.nodeWithTex )
      {
        Graphics3D::SetAmbientLightForward(node.material.ambient);
        node.mesh.drawForward(TextureAsset(node.material.diffuseTextureName), node.material.diffuse);
      }
    }

    static void draw(const mmd::ModelNodeAlpha &nodes)
    {
      draw(nodes.nodeNotAlpha);
      drawForward(nodes.nodeWithAlpha);
    }

    static void draw(const mmd::ModelNodeAlpha &nodes, const RasterizerState& state)
    {
      Graphics3D::SetRasterizerState(state);
      Graphics3D::SetRasterizerStateForward(state);
      draw(nodes);
    }

    void draw(const Mat4x4& worldMat)
    {
      mmd::MMDShader::init();
      auto vsAttach = ShaderAttach(mmd::MMDShader::vs());
      if ( !vsAttach ) return;
      auto vsForwardAttach = ShaderForwardAttach(mmd::MMDShader::vs());
      if ( !vsForwardAttach ) return;

      // ボーンをGPUに送信
      {
        ConstantBuffer<mmd::ConstantBoneData> data;
        PhysicsUpdate(worlds);
        m_bones->calcWorld(worldMat, worlds);
        for ( auto& i : step(static_cast<int>(worlds.size())) )
        {
          data->bones[i] = worlds[i].transposed();
        }
        Graphics3D::SetConstant(ShaderStage::Vertex, 1, data);
        Graphics3D::SetConstantForward(ShaderStage::Vertex, 1, data);
      }

      // 表情モーフの重みをGPUに送信
      {
        ConstantBuffer<std::array<Float4, 1024>> morphData{ {} };
        const auto& weights = m_faceMorph.weights();

        for ( auto& i : step(static_cast<int>(weights.size())) )
          morphData.get()[i].x = weights[i];

        Graphics3D::SetConstant(ShaderStage::Vertex, 2, morphData);
        Graphics3D::SetConstantForward(ShaderStage::Vertex, 2, morphData);
      }

      Graphics3D::SetTexture(ShaderStage::Vertex, 1, m_vertexTexture);

      const auto rasterizerState = Graphics3D::GetRasterizerState();
      const auto rasterizerStateForawrt = Graphics3D::GetRasterizerStateForward();
      draw(m_nodes.nodeCullBack, RasterizerState::SolidCullBack);
      draw(m_nodes.nodeCullNone, RasterizerState::SolidCullNone);
      Graphics3D::SetRasterizerState(rasterizerState);
      Graphics3D::SetRasterizerStateForward(rasterizerStateForawrt);
    }

    void drawEdge() const
    {
      mmd::EdgeShader::init();
      if ( auto attach = ShaderAttach(mmd::EdgeShader::vs(), mmd::EdgeShader::ps()) )
      {
        const auto rasterizerState = Graphics3D::GetRasterizerState();
        Graphics3D::SetRasterizerState(RasterizerState::SolidCullFront);
        for ( const auto& node : m_edges )
        {
          node.mesh.draw();
        }
        Graphics3D::SetRasterizerState(rasterizerState);
      }
    }
#ifdef USE_BULLET_PHYSICS
    MmdPhysics m_mmdPhysics;

#endif
    void PhysicsUpdate(Array<Mat4x4> &boneWorld)
    {
#ifdef USE_BULLET_PHYSICS
      m_mmdPhysics.BoneUpdate(Mat4x4::Identity(), boneWorld);
#endif
    }
    mmd::AllNode m_nodes;
    Array<mmd::Node> m_edges;
    String m_name;
    String m_comment;
    std::shared_ptr<mmd::Bones> m_bones;
    mmd::FaceMorph m_faceMorph;
    Texture m_vertexTexture;
    Array<Mat4x4> worlds;

  };
}