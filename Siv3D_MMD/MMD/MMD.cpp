#include "../include/MMD.h"
#include "../include/MMDModel.h"
#include "../ShaderAttacher.h"
#ifdef USE_BULLET_PHYSICS
#include "MMDPhysics.h"
#endif // USE_BULLET_PHYSICS

namespace s3d_mmd {
  namespace mmd {
    namespace {
      constexpr double alphaEps = 1e-9;
      struct Node {
        Mesh mesh;
        mmd::Material material;
      };

      struct ModelNodeTex {
        Array<mmd::Node> nodeNotTex;
        Array<mmd::Node> nodeWithTex;
        void set(const mmd::Node &node) {
          if (node.material.diffuseTextureName.isEmpty) nodeNotTex.push_back(node);
          else {
            nodeWithTex.push_back(node);
            TextureAsset::Register(node.material.diffuseTextureName, node.material.diffuseTextureName);
          }
        }
      };

      struct ModelNodeAlpha {
        ModelNodeTex nodeNotAlpha;
        ModelNodeTex nodeWithAlpha;

        void set(const mmd::Node &node) {
          if (node.material.diffuse.a + alphaEps < 1.0) nodeWithAlpha.set(node);
          else nodeNotAlpha.set(node);
        }

      };

      struct AllNode {
        ModelNodeAlpha nodeCullBack;
        ModelNodeAlpha nodeCullNone;

        void set(const mmd::Node &node) {
          if (node.material.isCullNone) nodeCullNone.set(node);
          else nodeCullBack.set(node);
        }

      };

      class EdgeShader {
      public:

        static void init() {
          if (m_isInit) return;
          m_vsEdge = VertexShader{ L"Data/Shaders/MMD3DEdge.hlsl",true };
          if (m_vsEdge.isEmpty())
            Println(L"MMD vertex shader compile error");

          m_psEdge = PixelShader{ L"Data/Shaders/MMD3DEdge.hlsl" ,true };
          if (m_psEdge.isEmpty())
            Println(L"MMD pixel shader compile error");

          m_isInit = true;
        }

        static void release() {
          m_isInit = false;
          m_vsEdge.release();
          m_psEdge.release();
        }

        static const VertexShader& vsInstance() { return m_vsEdge; }
        static const PixelShader& psInstance() { return m_psEdge; }

      private:

        static VertexShader m_vsEdge;
        static PixelShader m_psEdge;
        static bool m_isInit;
      };

      VertexShader EdgeShader::m_vsEdge;
      PixelShader EdgeShader::m_psEdge;
      bool EdgeShader::m_isInit = false;

      class Shader {
      public:

        static void init() {
          if (m_isInit) return;
          m_vs = VertexShader{ L"Data/Shaders/MMD.hlsl",true };
          if (m_vs.isEmpty())
            Println(L"MMD VMD vertex shader compile error");

          m_ps = PixelShader{ L"Data/Shaders/MMD.hlsl" ,true };
          if (m_ps.isEmpty())
            Println(L"MMD VMD pixel shader compile error");

          m_isInit = true;
        }

        static void release() {
          m_isInit = false;
          m_vs.release();
          m_ps.release();
        }

        static const VertexShader& vsInstance() { return m_vs; }
        static const PixelShader& psInstance() { return m_ps; }

      private:

        static VertexShader m_vs;
        static PixelShader m_ps;
        static bool m_isInit;
      };

      VertexShader Shader::m_vs;
      PixelShader Shader::m_ps;
      bool Shader::m_isInit = false;

      struct ConstantBoneData {
        Matrix bones[256];
      };
    }
  }
  class MMD::Pimpl {
  public:

    Float2 WriteTextureVertex(ImageRGBA32F &vertexImage, const mmd::MeshVertex &v, int &vPos) {
      const int x = vPos % 1020;
      const int y = vPos / 1020;
      auto image = vertexImage[y];
      image[x] = RGBA32F(v.position.x, v.position.y, v.position.z, 0);
      image[x + 1].r = v.boneNum[0] / 256.0f;
      image[x + 1].g = v.boneNum[1] / 256.0f;
      image[x + 2] = RGBA32F(v.boneWeight.x, v.boneWeight.y, v.boneWeight.z, v.boneWeight.w);
      image[x + 3].r = v.texcoord.x;
      image[x + 3].g = v.texcoord.y;
      vPos += 4;
      return{ x + 0.1, y + 0.1 };
    }

    Pimpl(const MMDModel& model) 
#ifdef USE_BULLET_PHYSICS
      : m_mmdPhysics(getBulletInstance()) 
#endif
    {
      m_bones = model.bones();
      m_name = model.name();
      m_comment = model.comment();
#ifdef USE_BULLET_PHYSICS
      m_mmdPhysics.Create(m_bones, model.rigidBodies(), model.joints());
#endif
      mmd::AllNode nodes;
      ImageRGBA32F vertexImage(1024, 1024);
      int vPos = 0;
      for (auto& node : model.nodes()) {
        const int size = static_cast<int>(node.mesh.vertices.size());
        Array<MeshVertex> vertex(size);
        for (auto &i : step(size)) {
          const mmd::MeshVertex &v = node.mesh.vertices[i];
          Float2 pos = WriteTextureVertex(vertexImage, v, vPos);
          vertex[i] = { v.position , v.normal, pos };
        }
        nodes.set(mmd::Node{ Mesh({ vertex, node.mesh.indices }), node.material });
      }
      m_nodes = std::move(nodes);
      m_edges.reserve(model.edgeNodes().size());
      for (auto& node : model.edgeNodes()) {
        const int size = static_cast<int>(node.mesh.vertices.size());
        Array<MeshVertex> vertex(size);
        for (auto& i : step(size)) {
          mmd::MeshVertex &v = node.mesh.vertices[i];
          Float2 pos = WriteTextureVertex(vertexImage, v, vPos);
          vertex[i] = { v.position, v.normal, pos };
        }
        m_edges.push_back(mmd::Node{ Mesh({ vertex, node.mesh.indices }), node.material });
      }
      m_vertexTexture = Texture(vertexImage);
    }

    static void draw(const mmd::ModelNodeTex &nodes) {
      for (auto& node : nodes.nodeNotTex) {
        Graphics3D::SetAmbientLight(node.material.ambient);
        node.mesh.draw(node.material.diffuse);
      }
      for (auto& node : nodes.nodeWithTex) {
        Graphics3D::SetAmbientLight(node.material.ambient);
        node.mesh.draw(TextureAsset(node.material.diffuseTextureName), node.material.diffuse);
      }
    }

    static void drawForward(const mmd::ModelNodeTex &nodes) {
      for (auto& node : nodes.nodeNotTex) {
        Graphics3D::SetAmbientLightForward(node.material.ambient);
        node.mesh.drawForward(node.material.diffuse);
      }
      for (auto& node : nodes.nodeWithTex) {
        Graphics3D::SetAmbientLightForward(node.material.ambient);
        node.mesh.drawForward(TextureAsset(node.material.diffuseTextureName), node.material.diffuse);
      }
    }

    static void draw(const mmd::ModelNodeAlpha &nodes) {
      draw(nodes.nodeNotAlpha);
      drawForward(nodes.nodeWithAlpha);
    }

    static void draw(const mmd::ModelNodeAlpha &nodes, const RasterizerState& state) {
      Graphics3D::SetRasterizerState(state);
      Graphics3D::SetRasterizerStateForward(state);
      draw(nodes);
    }

    void draw() {
#ifdef USE_BULLET_PHYSICS
      m_mmdPhysics.BoneUpdate(Mat4x4::Identity());
#endif
      const auto rasterizerState = Graphics3D::GetRasterizerState();
      const auto rasterizerStateForawrt = Graphics3D::GetRasterizerStateForward();
      draw(m_nodes.nodeCullBack, RasterizerState::SolidCullBack);
      draw(m_nodes.nodeCullNone, RasterizerState::SolidCullNone);
      Graphics3D::SetRasterizerState(rasterizerState);
      Graphics3D::SetRasterizerStateForward(rasterizerStateForawrt);
    }

    void drawEdge() const {
      mmd::EdgeShader::init();
      const auto attach_vs = ShaderAttach(mmd::EdgeShader::vsInstance());
      if (!attach_vs) return;
      const auto attach_ps = ShaderAttach(mmd::EdgeShader::psInstance());
      if (!attach_ps) return;
      const auto rasterizerState = Graphics3D::GetRasterizerState();
      Graphics3D::SetRasterizerState(RasterizerState::SolidCullFront);
      for (const auto& node : m_edges) {
        node.mesh.draw();
      }
      Graphics3D::SetRasterizerState(rasterizerState);
    }
#ifdef USE_BULLET_PHYSICS
    MmdPhysics m_mmdPhysics;
#endif
    mmd::AllNode m_nodes;
    Array<mmd::Node> m_edges;
    String m_name;
    String m_comment;
    std::shared_ptr<mmd::Bones> m_bones;
    Texture m_vertexTexture;
  };

  MMD::MMD(const MMDModel &model) {
    m_handle = std::make_shared<Pimpl>(model);
  }

  MMD::~MMD() {
  }

  void MMD::draw(double edgeSize) const {
    drawEdge(edgeSize);
    draw();
  }

  void MMD::drawEdge(double edgeSize) const {
    m_handle->drawEdge();
  }

  void MMD::draw() const {
    m_handle->draw();
  }

  void MMD::draw(const VMD &vmd) const {
    DrawableVMD(*this, vmd).draw();
  }

  const Texture & MMD::vertexTexture() const {
    return m_handle->m_vertexTexture;
  }

  std::shared_ptr<mmd::Bones> MMD::bones() const {
    return m_handle->m_bones;
  }

  const String & MMD::name() const {
    return m_handle->m_name;
  }

  const String & MMD::comment() const {
    return m_handle->m_comment;
  }

  DrawableVMD::DrawableVMD(const MMD &mmd, const VMD &vmd) :m_mmd(mmd), m_vmd(vmd) {
  }

  Array<Mat4x4> worlds;
  void DrawableVMD::draw() const {
    mmd::Shader::init();
    /*auto psAttach = ShaderAttach(MMDShader::psInstance());
    if (!psAttach) return;*/
    {
      auto vsAttach = ShaderAttach(mmd::Shader::vsInstance());
      if (!vsAttach) return;
      auto vsForwardAttach = ShaderForwardAttach(mmd::Shader::vsInstance());
      if (!vsForwardAttach) return;
      ConstantBuffer<mmd::ConstantBoneData> data;
      auto bones = m_mmd.bones();
      m_vmd.UpdateBone(*bones);
      Mat4x4 world = Mat4x4::Identity();
      bones->CalcWorld(world, worlds);
      for (auto& i : step(static_cast<int>(worlds.size()))) {
        data->bones[i] = worlds[i];
      }
      Graphics3D::SetConstant(ShaderStage::Vertex, 1, data);
      Graphics3D::SetConstantForward(ShaderStage::Vertex, 1, data);
      Graphics3D::SetTexture(ShaderStage::Vertex, 1, m_mmd.vertexTexture());
      m_mmd.draw();
    }
    {
      m_mmd.drawEdge(1.0);
    }
  }

  std::shared_ptr<s3d_bullet::BulletPhysics> getBulletInstance() {
    static std::shared_ptr<s3d_bullet::BulletPhysics> bulletPhysics =
      std::make_shared<s3d_bullet::BulletPhysics>(Vec3(0, -9.8, 0));
    return bulletPhysics;
  }
}
