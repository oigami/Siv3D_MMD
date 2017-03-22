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

      struct AllNode
      {
        Array<mmd::Node> nodes;

        void push(mmd::Node node)
        {
          nodes.push_back(std::move(node));
        }
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

    Float2 WriteTextureVertex(ImageRGBA32F& vertexImage, const mmd::MeshVertex& v, int& vPos, Array<Float4> morph);

    Pimpl(const MMDModel& model, const physics3d::Physics3DWorld& world);

    void drawForward(const Mat4x4& worldMat);

    void drawEdge(const Mat4x4& worldMat);

    void physicsUpdate();

    void update();
    void updateIK(const mmd::Ik& ikData);

    void attach(const VMD& vmd);

    const VMD& vmd() const;


    mmd::AllNode m_nodes;
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
    Mesh m_edgeMesh;
  };

  Float2 MMD::Pimpl::WriteTextureVertex(ImageRGBA32F& vertexImage, const mmd::MeshVertex& v, int& vPos, Array<Float4> morph)
  {
    int x = vPos % 1016;
    int y = vPos / 1016;
    assert(y < 1016);

    // モーフの数を16個ずつ処理することで高速化する
    morph.resize((morph.size() + 15) / 16 * 16);
    const int dataSize = static_cast<int>(3 + morph.size() + 1);
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
    int f = static_cast<int>(morph.size());
    image[x + 2] = RGBA32F(v.texcoord.x, v.texcoord.y, asFloat(f), 0.0f);

    int now = x + 2;
    for ( auto& i : morph )
    {
      image[++now] = RGBA32F(i.x, i.y, i.z, asFloat(static_cast<int>(i.w)));
    }
    vPos += dataSize;
    return { asFloat(x), asFloat(y) };
  }

  MMD::Pimpl::Pimpl(const MMDModel& model, const physics3d::Physics3DWorld& world)
    : m_mmdPhysics(world)
  {
    m_bones = model.bones();
    if ( m_bones->size() >= 256 )
    {
      Println(L"ボーンのサイズが多すぎる");
      return;
    }
    m_name = model.name();
    m_comment = model.comment();

    // 物理演算データの生成
    m_mmdPhysics.Create(m_bones, model.rigidBodies(), model.joints());

    ImageRGBA32F vertexImage(1024, 1024);
    struct FaceMorph
    {
      Array<Float4> vertex;
    };
    std::unordered_map<int, FaceMorph> faceMorphVertex;

    { // モーフデータの生成

      Array<int> baseMorph;
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


    Array<uint32> indices(model.indices().size());
    for ( auto& i : step(indices.size()) )
    {
      indices[i] = model.indices()[i];
    }

    int vPos = 0;
    { // メッシュの生成
      MeshData meshData;
      meshData.indices = indices;

      meshData.vertices.reserve(model.vertices().size());
      for ( auto& v : model.vertices() )
      {
        auto it = faceMorphVertex.find(v.vertexNum);
        Float2 pos = WriteTextureVertex(vertexImage, v, vPos, it != faceMorphVertex.end() ? it->second.vertex : Array<Float4>{});
        meshData.vertices.push_back({ v.position , v.normal, pos });
      }
      mmd::AllNode nodes;
      for ( auto& i : model.nodes() )
      {
        nodes.push(mmd::Node{ i.indexStart, i.indexCount, i.material });
      }
      m_nodes = std::move(nodes);
      m_mesh = Mesh(meshData);
    }

    { // エッジの生成
      MeshData meshData;
      meshData.indices = indices;
      meshData.vertices.resize(model.vertices().size());
      m_edges.reserve(model.nodes().size());
      for ( auto& i : step(model.vertices().size()) )
      {
        const auto& v = model.vertices()[i];
        auto it = faceMorphVertex.find(v.vertexNum);
        Float2 pos = WriteTextureVertex(vertexImage, v, vPos, it != faceMorphVertex.end() ? it->second.vertex : Array<Float4>{});
        meshData.vertices[i] = { v.position, v.normal, pos };
      }
      for ( auto& node : model.nodes() )
      {
        if ( node.material.isEdge )
        {
          m_edges.push_back({ node.indexStart,node.indexCount,node.material });
        }
      }

      m_edges.shrink_to_fit();
      m_edgeMesh = Mesh(meshData);
    }
    m_vertexTexture = Texture(vertexImage);
  }

  template<class Getter, class Setter, Getter getter, Setter setter>
  class StateBinderClass
  {
  public:
    template<class NowState>
    constexpr StateBinderClass(NowState&& nowState)
      : before(getter())
    {
      setter(std::forward<NowState>(nowState));
    }

    constexpr StateBinderClass() : before(getter()) {}

    ~StateBinderClass()
    {
      setter(before);
    }

  private:
    const decltype(getter()) before;
  };

  template<class Type, Type(*getter)(), void(*setter)(const Type&)>
  struct StateBinderClassHelper
  {
    using type = StateBinderClass<decltype(*getter), decltype(*setter), getter, setter>;
  };

  using RasterizerStateBinder2D = StateBinderClassHelper<RasterizerState,
                                                         Graphics3D::GetRasterizerState,
                                                         Graphics3D::SetRasterizerState>::type;
  using RasterizerStateForwardBinder2D = StateBinderClassHelper<RasterizerState,
                                                                Graphics3D::GetRasterizerStateForward,
                                                                Graphics3D::SetRasterizerStateForward>::type;

  void MMD::Pimpl::drawForward(const Mat4x4& worldMat)
  {
    mmd::MMDShader::init();
    auto vsForwardAttach = ShaderForwardAttach(mmd::MMDShader::vs());
    if ( !vsForwardAttach ) return;

    pushGPUData();

    const RasterizerStateForwardBinder2D rasterizerStateForward;

    for ( auto& i : m_nodes.nodes )
    {
      if ( i.material.isCullNone )
      {
        Graphics3D::SetRasterizerStateForward(RasterizerState::SolidCullNone);
      }
      else
      {
        Graphics3D::SetRasterizerStateForward(RasterizerState::SolidCullBack);
      }
      Graphics3D::SetAmbientLightForward(i.material.ambient);
      if(i.material.diffuseTextureName.isEmpty)
      {
        m_mesh.drawSubsetForward(i.indexStart, i.indexCount, i.material.diffuse);
      }else
      {
        m_mesh.drawSubsetForward(i.indexStart, i.indexCount, TextureAsset(i.material.diffuseTextureName), i.material.diffuse);
      }
    }
  }

  void MMD::Pimpl::drawEdge(const Mat4x4& worldMat)
  {
    mmd::EdgeShader::init();
    if ( auto attach = ShaderAttach(mmd::EdgeShader::vs(), mmd::EdgeShader::ps()) )
    {
      pushGPUData();

      const auto rasterizerState = Graphics3D::GetRasterizerState();
      Graphics3D::SetRasterizerState(RasterizerState::SolidCullFront);
      for ( const auto& node : m_edges )
      {
        m_edgeMesh.drawSubset(node.indexStart, node.indexCount, node.material.diffuse);
      }
      Graphics3D::SetRasterizerState(rasterizerState);
    }
  }

  void MMD::Pimpl::pushGPUData()
  {
    // ボーンをGPUに送信
    {
      ConstantBuffer<mmd::ConstantBoneData> data;

      for ( auto& i : step(static_cast<int>(worlds.size())) )
      {
        data->bones[i] = worlds[i].transposed();
      }
      Graphics3D::SetConstant(ShaderStage::Vertex, 1, data);
      Graphics3D::SetConstantForward(ShaderStage::Vertex, 1, data);
    }

    // 表情モーフの重みをGPUに送信
    {
      ConstantBuffer<std::array<Float4, 1024>> morphData;
      const auto& weights = m_faceMorph.weights();

      for ( auto& i : step(static_cast<int>(weights.size())) ) morphData.get()[i].x = weights[i];

      Graphics3D::SetConstant(ShaderStage::Vertex, 2, morphData);
      Graphics3D::SetConstantForward(ShaderStage::Vertex, 2, morphData);
    }

    Graphics3D::SetTexture(ShaderStage::Vertex, 1, m_vertexTexture);
    Graphics3D::SetTextureForward(ShaderStage::Vertex, 1, m_vertexTexture);
  }

  void MMD::Pimpl::physicsUpdate()
  {
    if ( isPhysicsEnabled ) m_mmdPhysics.BoneUpdate(Mat4x4::Identity(), worlds);
  }

  void MMD::Pimpl::update()
  {
    if ( !m_vmd.isEmpty() )
    {
      m_vmd.UpdateBone(*m_bones);
      m_vmd.UpdateMorph(m_faceMorph);
    }

    for ( auto& i : m_bones->ikData() ) updateIK(i);

    m_bones->calcWorld(Mat4x4::Identity(), worlds);

    physicsUpdate();
  }

  void MMD::Pimpl::updateIK(const mmd::Ik& ikData)
  {
    auto& bones = *m_bones;
    const Mat4x4& targetMat = bones.calcBoneMatML(ikData.ik_bone_index);

    for ( int i = ikData.iterations - 1; i >= 0; i-- )
    {
      for ( auto& attentionIdx : ikData.ik_child_bone_index )
      {
        auto& bone = bones[attentionIdx];
        const Mat4x4& effectorMat = bones.calcBoneMatML(ikData.ik_target_bone_index);
        const auto& invCoord = bones.calcBoneMatML(attentionIdx).inverse();


        // エフェクタのローカル方向（注目ボーン基準）
        const MyVector& localEffectorDir = MyVector::TransformCoord3(effectorMat.r[3], invCoord).normalize3();
        // ターゲットのローカル方向（注目ボーン基準）
        const MyVector& localTargetDir = MyVector::TransformCoord3(targetMat.r[3], invCoord).normalize3();

        const float angle = std::min(DirectX::XMVectorGetX(DirectX::XMVectorACos(DirectX::XMVectorClamp(localEffectorDir.dot3(localTargetDir), DirectX::g_XMNegativeOne, DirectX::g_XMOne))), ikData.control_weight);

        MyVector axis = localEffectorDir.cross3(localTargetDir);
        if ( bone.name == L"左足" || bone.name == L"右足" )
        {
          axis = axis * MyVector(1.0f, 0.0f, 1.0f, 0.0f);
        }

        if ( DirectX::XMVector3Equal(axis, DirectX::XMVectorZero()) ) continue;

        const Quaternion& rotation = DirectX::XMQuaternionRotationAxis(axis, angle);
        const Mat4x4& xmboneMatBL = bone.boneMat;
        if ( bone.name == L"左ひざ" || bone.name == L"右ひざ" )
        {
          const Quaternion& rv = (rotation * DirectX::XMQuaternionRotationMatrix(xmboneMatBL)).normalize();
          math::EulerAngles eulerAngle(rv.toMatrix());
          eulerAngle.x = Clamp(eulerAngle.x, Radians(-180.f), Radians(-10.f));
          eulerAngle.y = 0;
          eulerAngle.z = 0;
          bone.boneMat = DirectX::XMMatrixMultiply(eulerAngle.CreateMatrix(), DirectX::XMMatrixTranslationFromVector(xmboneMatBL.r[3]));
        }
        else
        {
          bone.boneMat = rotation.toMatrix() * xmboneMatBL;
        }
      }
    }
  }

  void MMD::Pimpl::attach(const VMD& vmd)
  {
    m_vmd = vmd;
  }

  const VMD& MMD::Pimpl::vmd() const
  {
    return m_vmd;
  }
}
