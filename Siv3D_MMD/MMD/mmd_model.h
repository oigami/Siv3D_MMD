#pragma once
#include <Siv3D.hpp>
#include <MMD/pmd_struct.h>
#include <MMD/mmd_bone.h>

namespace s3d_mmd
{
  namespace mmd
  {
    using RigidBodies = pmd_struct::RigidBodies;

    using Joints = pmd_struct::Joints;

    using SkinData = pmd_struct::SkinData;

    struct Material
    {
      ColorF ambient;
      ColorF diffuse;
      ColorF specular;
      String diffuseTextureName;
      bool isCullNone;
      bool isEdge;

      //TODO: スフィア
      String SphereTextureName;

      Texture texture;

      //TODO: Toon
      int toonTextureIndex;

      //s3d::Materialと同じように扱えるように不要なデータも追加
    };

    struct MeshVertex
    {
      Float3 position;
      Float3 normal;
      Float2 texcoord;
      std::array<int, 2> boneNum;
      Float4 boneWeight;
      bool isEdge;
      int vertexNum;
    };

    struct MeshData
    {
      Array<mmd::MeshVertex> vertices;
      Array<uint32> indices;
    };

    struct ModelNode
    {
      int indexStart;
      int indexCount;
      mmd::Material material;
    };
  }

  struct ITextureLoader
  {
  public:
    virtual ~ITextureLoader() = default;

    virtual Texture getTexture(FilePath baseDir, FilePath filename) = 0;
    virtual void removeTexture(FilePath baseDir, FilePath filename) = 0;
  };

  struct TextureAssetTextureLoader : ITextureLoader
  {
    Texture getTexture(FilePath baseDir, FilePath filename) override
    {
      const auto& path = baseDir + filename;
      if ( !TextureAsset::IsRegistered(path) )
      {
        TextureAsset::Register(path, path);
      }
      return static_cast<Texture>(TextureAsset(path));
    }

    void removeTexture(FilePath baseDir, FilePath filename) override
    {
      TextureAsset::Unregister(baseDir + filename);
    }
  };

  class MMDModel
  {
    class Pimpl;
    std::shared_ptr<Pimpl> m_handle;
    static const std::shared_ptr<ITextureLoader> defaultLoader;
  public:

    MMDModel();

    /// <summary>
    /// 3D モデルファイルからモデルをロードします。
    /// </summary>
    /// <param name="path">
    /// 3D モデルファイルのパス
    /// </param>
    /// <remarks>
    /// MikuMikuDance pmd_struct形式のみサポートしています。
    /// </remarks>
    MMDModel(const FilePath& path, std::shared_ptr<ITextureLoader> textureLoader = defaultLoader);

    /// <summary>
    /// デストラクタ
    /// </summary>
    ~MMDModel();

    /// <summary>
    /// モデルをリリースします。
    /// </summary>
    /// <returns>
    /// なし
    /// </returns>
    void release();

    explicit operator bool() const { return !isEmpty(); }

    HandleIDType id() const;

    bool isEmpty() const;

    bool operator ==(const MMDModel& model) const;

    bool operator !=(const MMDModel& model) const;

    /// <summary>
    /// モデルノードの一覧を取得します。
    /// </summary>
    /// <returns>
    /// モデルノードの一覧
    /// </returns>
    Array<mmd::ModelNode>& nodes() const;
    Array<mmd::MeshVertex>& vertices() const;
    Array<uint16>& indices() const;

    std::shared_ptr<mmd::Bones> bones() const;

    const mmd::RigidBodies& rigidBodies() const;

    const mmd::Joints& joints() const;

    const mmd::SkinData skinData() const;

    const String& name() const;
    const String& comment() const;
  };
}
