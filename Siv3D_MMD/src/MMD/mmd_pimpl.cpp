#include "mmd_pimpl.h"

namespace s3d_mmd
{
  namespace
  {
    class TextureVertex
    {
    public:
      explicit TextureVertex(int x, int y) : m_width(x), m_image(x, y), m_pos(0) {}

      std::pair<RGBA32F*, Point> createIndex(int writeSize)
      {
        int x = m_pos % m_width;
        if ( x + writeSize >= m_width )
        {
          x = 0;
          m_pos += m_width - x;
        }
        int y = m_pos / m_width;
        m_pos += writeSize;
        return { m_image[y] + x, { x, y } };
      }

      Texture createTexture() const { return Texture(m_image); }

    private:

      int m_width;
      ImageRGBA32F m_image;
      int m_pos;
    };

    struct FaceMorph
    {
      Float3 vertex;
      int index;
    };
  }

  int GetWriteMorphSize(const Array<FaceMorph>& morph)
  {
    // モーフの数を16個ずつ処理することで高速化する
    return static_cast<int>(morph.size() + 15) / 16 * 16;
  }

  Float2 WriteTextureVertex(const mmd::MeshVertex& v, TextureVertex& tex, const Array<FaceMorph>& morph)
  {
    const int writeMorphCount = GetWriteMorphSize(morph);
    const int dataSize = static_cast<int>(3 + writeMorphCount);
    auto p = tex.createIndex(dataSize);
    auto image = p.first;

    auto asFloat = [](int a) { return *reinterpret_cast<float*>(&a); };
    assert(v.boneNum[0] < 256 && v.boneNum[1] < 256);
    // ボーン番号2つ
    image[0] = RGBA32F(asFloat(v.boneNum[0]), asFloat(v.boneNum[1]), 0.0f, 0.0f);
    // ボーンの重み4つ（現在2つしか処理していない）
    image[1] = RGBA32F(v.boneWeight.x, v.boneWeight.y, v.boneWeight.z, v.boneWeight.w);
    // テクスチャの座標とモーフの数とエッジかどうかのフラグ
    const int f = static_cast<int>(writeMorphCount);
    image[2] = RGBA32F(v.texcoord.x, v.texcoord.y, asFloat(f), v.isEdge ? 1.0f : 0.0f);

    int now = 2;
    for ( auto& m : morph )
    {
      // 頂点モーフを動かす時の方向と対応するモーフの番号
      const auto& i = m.vertex;
      image[++now] = RGBA32F(i.x, i.y, i.z, asFloat(m.index));
    }

    for ( auto& i : step(writeMorphCount - morph.size()) )
    {
      // 高速化のためのパディング
      image[++now] = RGBA32F(0, 0, 0, asFloat(static_cast<int>(0)));
    }
    const auto& pos = p.second;
    return { asFloat(pos.x), asFloat(pos.y) };
  }

  MMD::Pimpl::Pimpl(const MMDModel& model, std::shared_ptr<IMMDPhysics> world)
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
    m_mmdPhysics->create(m_bones, model.rigidBodies(), model.joints());

    std::unordered_map<int, Array<FaceMorph>> faceMorphVertex;

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
          faceMorphVertex[baseMorph[j.skin_vert_index]].push_back({ j.skin_vert_pos,index });
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
    { // メッシュの生成
      const Array<FaceMorph> emptyMorph;
      int allWriteSize = 0;
      for ( auto& v : model.vertices() )
      {
        auto it = faceMorphVertex.find(v.vertexNum);
        allWriteSize += 3 + GetWriteMorphSize(it != faceMorphVertex.end() ? it->second : emptyMorph);
      }

      auto nextPow2 = [](uint32 x)
        {
          x--;
          x |= x >> 1;
          x |= x >> 2;
          x |= x >> 4;
          x |= x >> 8;
          x |= x >> 16;
          return ++x;
        };
      int size = nextPow2(allWriteSize);
      // 折り返しの都合で全部埋まるとは限らないので適当に増やしておく
      int padding = model.skinData().size() * (size / 4096) / 4096;
      TextureVertex textureVertex(4096, nextPow2(size / 4096 + 1 + padding));

      MeshData meshData;
      meshData.indices = indices;

      meshData.vertices.reserve(model.vertices().size());
      for ( auto& v : model.vertices() )
      {
        auto it = faceMorphVertex.find(v.vertexNum);
        Float2 pos = WriteTextureVertex(v, textureVertex, it != faceMorphVertex.end() ? it->second : emptyMorph);
        meshData.vertices.push_back({ v.position , v.normal, pos });
      }
      m_nodes.reserve(model.nodes().size());
      for ( auto& i : model.nodes() )
      {
        m_nodes.push_back(mmd::Node{ i.indexStart, i.indexCount, i.material });
      }
      m_mesh = Mesh(meshData);
      m_vertexTexture = textureVertex.createTexture();
    }

    { // エッジの生成
      m_edges.reserve(model.nodes().size());
      for ( auto& node : model.nodes() )
      {
        if ( node.material.isEdge )
        {
          m_edges.push_back({ node.indexStart,node.indexCount,node.material });
        }
      }

      m_edges.shrink_to_fit();
    }
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

    for ( auto& i : m_nodes )
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
      if ( i.material.diffuseTextureName.isEmpty )
      {
        m_mesh.drawSubsetForward(i.indexStart, i.indexCount, worldMat, i.material.diffuse);
      }
      else
      {
        m_mesh.drawSubsetForward(i.indexStart, i.indexCount, worldMat, i.material.texture, i.material.diffuse);
      }
    }
  }

  void MMD::Pimpl::drawEdge(double edgeSize, const Mat4x4& worldMat)
  {
    ConstantBuffer<float[4]> data;
    (*data)[0] = static_cast<float>(edgeSize);
    Graphics3D::SetConstant(ShaderStage::Vertex, 3, data);

    mmd::EdgeShader::init();
    if ( auto attach = ShaderAttach(mmd::EdgeShader::vs(), mmd::EdgeShader::ps()) )
    {
      pushGPUData();

      const auto rasterizerState = Graphics3D::GetRasterizerState();
      Graphics3D::SetRasterizerState(RasterizerState::SolidCullFront);
      for ( const auto& node : m_edges )
      {
        m_mesh.drawSubset(node.indexStart, node.indexCount, worldMat, node.material.diffuse);
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
    if ( isPhysicsEnabled ) m_mmdPhysics->boneUpdate(Mat4x4::Identity(), worlds);
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

        const float angle = std::min(DirectX::XMVectorGetX(DirectX::XMVectorACos(DirectX::XMVectorClamp(localEffectorDir.dot3(localTargetDir),
                                                                                                        DirectX::g_XMNegativeOne, DirectX::g_XMOne))), ikData.control_weight);

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
