#pragma once
#include <Siv3D.hpp>
#include "PMDStruct.h"
#include "MMDBone.h"
namespace s3d_mmd {
  namespace mmd {
    struct Material {

      ColorF ambient;
      ColorF diffuse;
      ColorF specular;
      String diffuseTextureName;
      bool isCullNone;
      bool isEdge;
      //TODO: スフィア
      String SphereTextureName;
      //TODO: Toon
      int toonTextureIndex;

      //s3d::Materialと同じように扱えるように不要なデータも追加
    };

    struct MeshVertex {
      Float3 position;
      Float3 normal;
      Float2 texcoord;
      std::array<int, 2> boneNum;
      Float4 boneWeight;
      bool isEdge;
    };

    struct MeshData {
      Array<mmd::MeshVertex> vertices;
      Array<uint32> indices;
    };

    struct ModelNode {
      mmd::MeshData mesh;
      mmd::Material material;
    };
  }
  class MMDModel {

    class Pimpl;
    std::shared_ptr<Pimpl> m_handle;

  public:

    MMDModel();

    /// <summary>
    /// 3D モデルファイルからモデルをロードします。
    /// </summary>
    /// <param name="path">
    /// 3D モデルファイルのパス
    /// </param>
    /// <remarks>
    /// MikuMikuDance pmd形式のみサポートしています。
    /// </remarks>
    MMDModel(const FilePath& path);

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

    bool operator == (const MMDModel& model) const;

    bool operator != (const MMDModel& model) const;

    /// <summary>
    /// モデルノードの一覧を取得します。
    /// </summary>
    /// <returns>
    /// モデルノードの一覧
    /// </returns>
    Array<mmd::ModelNode>& nodes() const;
    Array<mmd::ModelNode>& edgeNodes() const;

    std::shared_ptr<mmd::Bones> bones() const;

    const pmd::RigidBodies& rigidBodies() const;

    const pmd::Joints& joints() const;

    const String& name()const;
    const String& comment()const;

  };


}
