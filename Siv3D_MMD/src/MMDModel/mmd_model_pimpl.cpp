#include "stdafx/stdafx.h"
#include "mmd_model_pimpl.h"

namespace s3d_mmd
{
  namespace
  {
    constexpr double alphaEps = 1e-9;
    uint32 handleCounter = 0;

    uint32 createHandle() { return ++handleCounter; }
  }

  const std::shared_ptr<ITextureLoader> MMDModel::defaultLoader = std::make_shared<TextureAssetTextureLoader>();

  mmd::MeshVertex CreateVertex(const pmd_struct::Vertex& vertex)
  {
    mmd::MeshVertex v;
    v.position = vertex.pos;
    v.texcoord = vertex.uv;
    v.normal = vertex.normal_vec;
    v.boneNum[0] = vertex.bone_num[0];
    v.boneNum[1] = vertex.bone_num[1];
    v.boneWeight.set(vertex.bone_weight / 100.0f, 1 - vertex.bone_weight / 100.0f, 0, 0);
    v.isEdge = vertex.edge_flag == 0; //0の場合有効
    v.normal.normalize();
    return v;
  }

  Array<mmd::MeshVertex> CreateVertices(const pmd_struct::Vertices& Vertices)
  {
    Array<mmd::MeshVertex> meshVertices;
    meshVertices.resize(Vertices.size());
    for ( auto& i : step(static_cast<int>(Vertices.size())) )
    {
      meshVertices[i] = CreateVertex(Vertices[i]);
      meshVertices[i].vertexNum = i;
    }
    return meshVertices;
  }

  Array<mmd::Ik> CreateIkData(const pmd_struct::IkData& ikData)
  {
    Array<mmd::Ik> newIkData(ikData.size());
    for ( auto& i : step(static_cast<int>(ikData.size())) )
    {
      newIkData[i].control_weight = ikData[i].control_weight * Math::PiF;
      newIkData[i].ik_bone_index = ikData[i].ik_bone_index;
      newIkData[i].ik_child_bone_index.reserve(ikData[i].ik_child_bone_length);
      for ( auto& j : ikData[i].ik_child_bone_index )
      {
        newIkData[i].ik_child_bone_index.push_back(j);
      }
      newIkData[i].ik_target_bone_index = ikData[i].ik_target_bone_index;
      newIkData[i].iterations = ikData[i].iterations;
    }
    return newIkData;
  }

  mmd::Bones CreateBones(const Array<pmd_struct::Bone>& pmdBones, const pmd_struct::IkData& ikData)
  {
    Array<mmd::Bone> bones;
    const int size = static_cast<int>(pmdBones.size());
    bones.resize(size);
    for ( int i : step(size) )
    {
      const auto& item = pmdBones[i];
      mmd::Bone& bone = bones[i];
      const uint16 parentBoneIndex = item.parent_bone_index;

      //自分と同じ親で自分よりあとのボーンが兄弟になる
      for ( int j = i + 1; j < size; ++j )
        if ( parentBoneIndex == pmdBones[j].parent_bone_index )
        {
          bone.sibling = j;
          break;
        }

      //自分が親担っていて一番早く現れるボーンが子になる
      for ( int j : step(size) )
        if ( i == pmdBones[j].parent_bone_index )
        {
          bone.firstChild = j;
          break;
        }

      if ( parentBoneIndex != 0xFFFF ) bone.parent = parentBoneIndex;

      char boneName[21] = { 0 }; // ボーン名が20byteのときのために最後に0を追加
      memcpy(boneName, item.bone_name, 20);
      bone.name = Widen(boneName);
      bone.id = i;
      bone.type = item.bone_type;
      const Mat4x4 modelLocalInitMat = Mat4x4::Translate(item.bone_head_pos[0], item.bone_head_pos[1], item.bone_head_pos[2]);
      bone.boneMatML = modelLocalInitMat;
      bone.initMatML = bone.initMatBoneLocal = DirectX::XMVectorSet(item.bone_head_pos[0], item.bone_head_pos[1], item.bone_head_pos[2], 1); // モデルローカル座標系
      bone.offsetMat = DirectX::XMVectorMultiply(Vector{ -1.0f, -1.0f, -1.0f, 0.0f }, bone.initMatBoneLocal);
    }
    return { std::move(bones), CreateIkData(ikData) };
  }

  MMDModel::Pimpl::Pimpl(const FilePath& path, const std::shared_ptr<ITextureLoader> textureLoader)
  {
    m_textureLoader = textureLoader;
    PMDReader loader(path);
    load(loader, path);
  }

  MMDModel::Pimpl::Pimpl(PMDReader& loader, const std::shared_ptr<ITextureLoader> textureLoader)
  {
    m_textureLoader = textureLoader;
    load(loader, L"");
  }

  void MMDModel::Pimpl::load(const PMDReader& loader, const String& path)
  {
    if ( loader.isLoaded() == false )
    {
      return;
    }
    m_baseDir = path;
    createNode(loader, path);
    m_bones = std::make_shared<mmd::Bones>();
    *m_bones = CreateBones(loader.getBones(), loader.getIkData());
    m_modelName = loader.getModelName();
    m_comment = loader.getComment();
    m_rigidBodies = loader.getRigidBodies();
    m_joints = loader.getJoints();
    m_skinData = loader.getSkinData();
    m_handle = createHandle();
  }

  void MMDModel::Pimpl::release()
  {
    for ( auto& i : m_nodes )
    {
      m_textureLoader->removeTexture(m_baseDir, FileSystem::FileName(i.material.diffuseTextureName));
    }
    m_textureLoader.reset();
    m_handle = NullHandleID;
    m_nodes.clear();
    m_nodes.shrink_to_fit();
    m_bones = std::make_shared<mmd::Bones>();
    m_modelName.clear();
    m_comment.clear();
  }


  mmd::Material MMDModel::Pimpl::createMaterial(const pmd_struct::Material& pmdMaterial, const FilePath& m_filepath) const
  {
    mmd::Material material;

    auto ToColorF = [](const float (&f)[3], float alpha) { return ColorF(f[0], f[1], f[2], alpha); };

    material.ambient = ToColorF(pmdMaterial.mirror_color, 1.0f);
    material.diffuse = ToColorF(pmdMaterial.diffuse_color, pmdMaterial.alpha);
    material.specular = ToColorF(pmdMaterial.specular_color, pmdMaterial.specularity);

    material.isEdge = pmdMaterial.edge_flag != 0;
    const float alpha = pmdMaterial.alpha;
    material.isCullNone = alpha + 1e-7 < 1.0;
    if ( pmdMaterial.texture_file_name[0] != '\0' )
    {
      //TODO: スフィアに未対応
      String filename = Widen({ pmdMaterial.texture_file_name,sizeof(pmdMaterial.texture_file_name) });
      const size_t pos = filename.lastIndexOf(L'*');
      if ( pos != String::npos )
      {
        filename = filename.substr(0, pos);
      }
      material.diffuseTextureName = FileSystem::ParentPath(m_filepath) + filename;

      material.texture = m_textureLoader->getTexture(FileSystem::ParentPath(m_filepath), filename);
    }


    return material;
  }

  void MMDModel::Pimpl::createNode(const PMDReader& loader, const FilePath& m_filepath)
  {
    const auto& pmdMaterials = loader.getMaterials();
    const auto& pmdVertices = loader.getVertices();
    m_indices = loader.getFaces();
    m_vertecies = CreateVertices(pmdVertices);
    m_nodes.reserve(pmdMaterials.size());

    int preVertexLen = 0;
    for ( auto& item : pmdMaterials )
    {
      const int face_vertex_len = item.face_vert_count;
      const mmd::Material material = createMaterial(item, m_filepath);

      // 材質とインデックスデータを追加
      m_nodes.push_back({ preVertexLen, face_vertex_len, material });
      preVertexLen += face_vertex_len;
    }
  }
}
