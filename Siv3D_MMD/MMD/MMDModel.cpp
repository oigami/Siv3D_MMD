#include "../include/MMDModel.h"
#include "../include/PMDReader.h"
namespace s3d_mmd {
  namespace {
    constexpr double alphaEps = 1e-9;
    uint32 handleCounter = 0;
    uint32 createHandle() { return ++handleCounter; }

    class IndexMap {
      std::unordered_map<int, int> m_map;
      int m_nowIndex;
    public:
      IndexMap() { reset(); }
      void reset() {
        m_map.clear();
        m_nowIndex = 0;
      }
      /// <summary>
      /// インデックスを0から振り直す
      /// </summary>
      /// <param name="index"></param>
      /// <returns> インデックス、 新規挿入したかどうか </returns>
      std::pair<int, bool> insert(int index) {
        auto iter = m_map.insert({ index , m_nowIndex });
        if (iter.second)
          m_nowIndex++;
        return{ iter.first->second, iter.second };
      }
    };

  }
  mmd::Material CreateMaterial(const pmd::Material &pmdMaterial, const FilePath &m_filepath) {
    mmd::Material material;
    material.ambient.a = 1;
    material.ambient.r = pmdMaterial.mirror_color[0];
    material.ambient.g = pmdMaterial.mirror_color[1];
    material.ambient.b = pmdMaterial.mirror_color[2];
    material.diffuse.a = pmdMaterial.alpha;
    material.diffuse.r = pmdMaterial.diffuse_color[0];
    material.diffuse.g = pmdMaterial.diffuse_color[1];
    material.diffuse.b = pmdMaterial.diffuse_color[2];
    material.specular.a = pmdMaterial.specularity;
    material.specular.r = pmdMaterial.specular_color[0];
    material.specular.g = pmdMaterial.specular_color[1];
    material.specular.b = pmdMaterial.specular_color[2];

    material.isEdge = pmdMaterial.edge_flag != 0;
    const float alpha = pmdMaterial.alpha;
    material.isCullNone = alpha + alphaEps < 1.0;
    if (pmdMaterial.texture_file_name[0] != '\0') {
      //TODO: スフィアに未対応
      String filename = Widen({ pmdMaterial.texture_file_name,sizeof(pmdMaterial.texture_file_name) });
      const size_t pos = filename.lastIndexOf(L'*');
      if (pos != String::npos) {
        filename = filename.substr(0, pos);
      }
      material.diffuseTextureName = FileSystem::ParentPath(m_filepath) + filename;
    }
    return material;
  }
  mmd::MeshVertex CreateVertex(const pmd::Vertex &vertex) {
    mmd::MeshVertex v{};
    v.position.set(vertex.pos[0], vertex.pos[1], vertex.pos[2]);
    v.texcoord.set(vertex.uv[0], vertex.uv[1]);
    v.normal.set(vertex.normal_vec[0], vertex.normal_vec[1], vertex.normal_vec[2]);
    v.boneNum[0] = vertex.bone_num[0];
    v.boneNum[1] = vertex.bone_num[1];
    v.boneWeight.set(vertex.bone_weight / 100.0f, 1 - vertex.bone_weight / 100.0f, 0, 0);
    v.isEdge = vertex.edge_flag == 0; //0の場合有効
    v.normal.normalize();
    return v;
  }
  Array<mmd::MeshVertex> CreateVertices(const pmd::Vertices &Vertices) {
    Array<mmd::MeshVertex> meshVertices;
    meshVertices.reserve(Vertices.size());
    for (auto& i : Vertices) {
      meshVertices.push_back(CreateVertex(i));
    }
    return meshVertices;
  }

  std::pair<Array<mmd::ModelNode>, Array<mmd::ModelNode>> CreateNode(const PMDReader &loader, const FilePath &m_filepath) {
    const auto &pmdFaces = loader.getFaces();
    const auto &pmdMaterials = loader.getMaterials();
    const auto &pmdVertices = loader.getVertices();
    auto meshVertices = CreateVertices(pmdVertices);
    auto pmdFacesIter = pmdFaces.begin();
    Array<mmd::ModelNode> nodes, edgeNodes;
    nodes.reserve(pmdMaterials.size());
    IndexMap indexMap, edgeIndex;
    for (auto& item : pmdMaterials) {
      const int face_vertex_len = item.face_vert_count;
      const mmd::Material material = CreateMaterial(item, m_filepath);
      indexMap.reset();
      edgeIndex.reset();
      MeshData mesh, edgeMesh;
      Array<uint32_t> indices(face_vertex_len), edgeIndices;
      Array<mmd::MeshVertex> vertices, edgeVertices;
      vertices.reserve(face_vertex_len);
      const bool isEdge = item.edge_flag != 0;
      if (isEdge) edgeIndices.resize(face_vertex_len);
      for (auto& i : step(face_vertex_len)) {
        const int faceIndex = *pmdFacesIter;
        ++pmdFacesIter;
        const auto iter = indexMap.insert(faceIndex);
        const mmd::MeshVertex &meshVertex = meshVertices[faceIndex];
        if (iter.second) //新規に挿入した時のみ頂点を追加
          vertices.push_back(meshVertex);
        indices[i] = iter.first;

        if (!isEdge) continue;
        const auto edgeIter = edgeIndex.insert(faceIndex);
        if (edgeIter.second) {
          if (pmdVertices[faceIndex].edge_flag == 0) {
            edgeVertices.push_back(meshVertex);
          } else {
            edgeVertices.push_back({ meshVertex.position, Float3(0, 0, 0), meshVertex.texcoord });
          }
        }
        edgeIndices[i] = edgeIter.first;
      }
      vertices.shrink_to_fit();
      nodes.push_back({ { vertices, indices }, material });
      if (isEdge)
        edgeNodes.push_back({ {edgeVertices, edgeIndices},material });
    }
    return{ std::move(nodes), std::move(edgeNodes) };
  }
  Array<mmd::Ik> CreateIkData(const pmd::IkData & ikData) {
    Array<mmd::Ik> newIkData(ikData.size());
    for (auto& i : step(static_cast<int>(ikData.size()))) {
      newIkData[i].control_weight = ikData[i].control_weight;
      newIkData[i].ik_bone_index = ikData[i].ik_bone_index;
      newIkData[i].ik_child_bone_index.reserve(ikData[i].ik_child_bone_length);
      for (auto& j : ikData[i].ik_child_bone_index) {
        newIkData[i].ik_child_bone_index.push_back(j);
      }
      newIkData[i].ik_target_bone_index = ikData[i].ik_target_bone_index;
      newIkData[i].iterations = ikData[i].iterations;
    }
    return newIkData;
  }
  mmd::Bones CreateBones(const Array<pmd::Bone> &pmdBones, const pmd::IkData &ikData) {
    mmd::Bones bones(CreateIkData(ikData));
    const int size = static_cast<int>(pmdBones.size());
    bones.resize(size);
    for (int i : step(size)) {
      const auto &item = pmdBones[i];
      mmd::Bone &bone = bones[i];
      const uint16 parentBoneIndex = item.parent_bone_index;
      //自分と同じ親で自分よりあとのボーンが兄弟になる
      for (int j = i + 1; j < size; ++j) if (parentBoneIndex == pmdBones[j].parent_bone_index) {
        bone.sibling = j;
        break;
      }
      //自分が親担っていて一番早く現れるボーンが子になる
      for (int j : step(size)) if (i == pmdBones[j].parent_bone_index) {
        bone.firstChild = j;
        break;
      }

      if (parentBoneIndex != 0xFFFF)
        bone.parent = parentBoneIndex;

      char boneName[21] = { 0 };	// ボーン名が20byteのときのために最後に0を追加
      memcpy(boneName, item.bone_name, 20);
      bone.name = boneName;
      bone.id = i;
      bone.type = item.bone_type;
      const Mat4x4 modelLocalInitMat = Mat4x4::Translate(item.bone_head_pos[0], item.bone_head_pos[1], item.bone_head_pos[2]);
      bone.initMatML = bone.boneMatML = bone.initMat = modelLocalInitMat;	// モデルローカル座標系
      bone.offsetMat = XMMatrixInverse(0, modelLocalInitMat);
    }
    bones.InitMatCalc();
    return bones;
  }

  class MMDModel::Pimpl {
  public:
    Pimpl(const FilePath& path) {
      PMDReader loader(path);
      std::tie(m_nodes, m_edges) = CreateNode(loader, path);
      m_bones = std::make_shared<mmd::Bones>();
      *m_bones = CreateBones(loader.getBones(), loader.getIkData());
      m_modelName = loader.getModelName();
      m_comment = loader.getComment();
      m_rigidBodies = loader.getRigidBodies();
      m_joints = loader.getJoints();
      m_handle = createHandle();
    }
    Pimpl() :m_handle(NullHandleID) {
    }

    void release() {
      m_handle = NullHandleID;
      m_edges = m_nodes = Array<mmd::ModelNode>();
      m_bones = std::make_shared<mmd::Bones>();
      m_modelName.clear();
      m_comment.clear();
    }
    Array<mmd::ModelNode> m_nodes;
    Array<mmd::ModelNode> m_edges;
    std::shared_ptr<mmd::Bones> m_bones;

    pmd::RigidBodies m_rigidBodies;
    pmd::Joints m_joints;
    String m_modelName;
    String m_comment;
    HandleIDType m_handle;
    Texture vertexTexture;
  };

  MMDModel::MMDModel() {
  }

  MMDModel::MMDModel(const FilePath & path) {
    m_handle = std::make_shared<Pimpl>(path);
  }

  MMDModel::~MMDModel() {
    m_handle = std::make_shared<Pimpl>();
  }

  void MMDModel::release() {
    m_handle->release();
  }

  HandleIDType MMDModel::id() const {
    return m_handle->m_handle;
  }

  bool MMDModel::isEmpty() const {
    return !m_handle;
  }

  bool MMDModel::operator==(const MMDModel & model) const {
    return m_handle == model.m_handle;
  }

  bool MMDModel::operator!=(const MMDModel & model) const {
    return !(*this == model);
  }

  /// <summary>
  /// モデルノードの一覧を取得します。
  /// </summary>
  /// <returns>
  /// モデルノードの一覧
  /// </returns>
  Array<mmd::ModelNode>& MMDModel::nodes() const {
    return m_handle->m_nodes;
  }

  Array<mmd::ModelNode>& MMDModel::edgeNodes() const {
    return m_handle->m_edges;
  }
  std::shared_ptr<mmd::Bones> MMDModel::bones() const {
    return m_handle->m_bones;
  }

  const pmd::RigidBodies & MMDModel::rigidBodies() const {
    return m_handle->m_rigidBodies;
  }

  const pmd::Joints & MMDModel::joints() const {
    return  m_handle->m_joints;
  }

  const String& MMDModel::name() const {
    return m_handle->m_modelName;
  }
  const String& MMDModel::comment() const {
    return m_handle->m_comment;
  }
}
