#include "../include/MMD.h"
#ifdef USE_BULLET_PHYSICS
#include "MmdPhysics.h"
#include "../BulletPhysics/detail/Siv3DBulletConverter.h"
#include "../include/BulletBox.h"
#include "../include/BulletSphere.h"
#include "../include/BulletCapsule.h"
#endif
#include <DirectXMath.h>
namespace s3d_mmd {

#ifdef USE_BULLET_PHYSICS
  //#pragma unmanaged
  MmdPhysics::MmdPhysics(s3d_bullet::BulletPhysics bulletPhysics) :m_bulletPhysics(bulletPhysics) {
    //this->joint_mesh = NULL;
  }

  MmdPhysics::~MmdPhysics() {
    //int max=rigidbody_mesh.size();
    //int cnt=0;
    /*for(auto &it : rigidbody_mesh){
      printf("max=%d now=%d %p\n",max,cnt++,it);
      SAFE_RELEASE(it);
      }*/
      //SAFE_RELEASE(joint_mesh);
    Destroy();
  }

  void MmdPhysics::Create(std::shared_ptr<mmd::Bones> bones,
    const std::vector<pmd::RigidBody>& pmdRigidBodies,
    const std::vector<pmd::Joint>& pmdJoints) {
    SetBones(bones);
    CreateRigid(pmdRigidBodies);
    CreateJoint(pmdJoints);
  }

  void MmdPhysics::Destroy() {
    /*for (auto &it : m_rigidBodies) {
      it.body_.RemoveRigidBody();
      it.body_.release();
    }*/
    m_bones = nullptr;
    m_rigidBodies.clear();
    m_rigidbodyRelatedBoneIndex.clear();
    m_rigidbodyType.clear();
    m_rigidbodyInit.clear();
    m_rigidbodyOffset.clear();
    m_jointRelatedRigidIndex.clear();
    m_jointMatrix.clear();
    m_rigidMat.clear();
    m_initOffsetMat.clear();
  }
  void MmdPhysics::SetBones(std::shared_ptr<mmd::Bones> bones) {
    this->m_bones = bones;
  }

  /// 剛体を作成
  /// @param pmdRigidBodies Pmd剛体配列
  /// @param pmdBones Pmdボーン配列

  inline void MmdPhysics::CreateRigid(const std::vector<s3d_mmd::pmd::RigidBody>& pmdRigidBodies) {
    const int pmdRigidBodiesSize = static_cast<int>(pmdRigidBodies.size());
    m_rigidbodyRelatedBoneIndex.resize(pmdRigidBodiesSize);
    m_rigidbodyType.resize(pmdRigidBodiesSize);
    m_rigidbodyInit.resize(pmdRigidBodiesSize);
    m_rigidbodyOffset.resize(pmdRigidBodiesSize);
    m_rigidBodies.resize(pmdRigidBodiesSize);
    //rigidbody_mesh.resize(pmdRigidBodiessize,0);
    //const MmdStruct::PmdRigidBody *pmdRigidBodies = &pmdRigidBodie[0];
    //#pragma loop(hint_parallel() )

    //for(auto it=pmdRigidBodie.begin(); it!=itend; ++it){

    for (int i = 0; i < pmdRigidBodiesSize; ++i) {
      using namespace DirectX;
      m_rigidbodyRelatedBoneIndex[i] = (pmdRigidBodies[i].rigidbody_rel_bone_index);
      m_rigidbodyType[i] = (pmdRigidBodies[i].rigidbody_type);
      XMMATRIX world, world_inv;
      world = CreateRigidMatrix(pmdRigidBodies[i].pos_pos, pmdRigidBodies[i].pos_rot, pmdRigidBodies[i].rigidbody_rel_bone_index);
      world_inv = XMMatrixInverse(nullptr, world);
      m_rigidbodyInit[i] = world;
      m_rigidbodyOffset[i] = world_inv;
      s3d_bullet::bullet::RigidBody rigidBody;
      const bool isKinematic = pmdRigidBodies[i].rigidbody_type == 0;
      if (pmdRigidBodies[i].shape_type == 0) {		// 球
        const float radius = pmdRigidBodies[i].shape_w;

        s3d_bullet::bullet::Sphere sphere(radius);
        if (!isKinematic) sphere.setMass(pmdRigidBodies[i].rigidbody_weight);
        rigidBody = std::move(s3d_bullet::bullet::RigidBody{ sphere, m_rigidbodyInit[i] });

      } else if (pmdRigidBodies[i].shape_type == 1) {	// 箱
        const float width = 2 * pmdRigidBodies[i].shape_w;
        const float height = 2 * pmdRigidBodies[i].shape_h;
        const float depth = 2 * pmdRigidBodies[i].shape_d;

        s3d_bullet::bullet::Box box(width, height, depth);
        if (!isKinematic) box.setMass(pmdRigidBodies[i].rigidbody_weight);
        rigidBody = { box, m_rigidbodyInit[i] };

      } else if (pmdRigidBodies[i].shape_type == 2) {	// カプセル
        const float radius = pmdRigidBodies[i].shape_w, height = pmdRigidBodies[i].shape_h;

        s3d_bullet::bullet::Capsule capsule(radius, height);
        if (!isKinematic) capsule.setMass(pmdRigidBodies[i].rigidbody_weight);
        rigidBody = { capsule, m_rigidbodyInit[i] };
      }

      rigidBody.setDamping(pmdRigidBodies[i].rigidbody_pos_dim, pmdRigidBodies[i].rigidbody_rot_dim)
        .setFriction(pmdRigidBodies[i].rigidbody_friction)
        .setKinematic(isKinematic)
        .setRestitution(pmdRigidBodies[i].rigidbody_recoil)
        .addWorld(1 << pmdRigidBodies[i].rigidbody_group_index, pmdRigidBodies[i].rigidbody_group_target);

      m_rigidBodies[i] = std::move(rigidBody);
    }
  }

  template<class T, size_t n> std::array<float, 3> ConvertArray(T(&val)[n]) {
    static_assert(n == 3 || n == 4, "");
    return std::array<float, 3>({ val[0], val[1], val[2] });
  }

  template<class T, size_t n> Float3 ConvertFloat(T(&val)[n]) {
    static_assert(n == 3 || n == 4, "");
    return Float3(val[0], val[1], val[2]);
  }

  void MmdPhysics::CreateJoint(const Array<s3d_mmd::pmd::Joint> &pmdJoints) {
    const size_t pmdJointsSize = pmdJoints.size();
    m_jointRelatedRigidIndex.reserve(pmdJointsSize);
    m_jointMatrix.reserve(pmdJointsSize);

    for (auto &joint : pmdJoints) {
      using namespace DirectX;
      std::array<float, 3> c_p1(ConvertArray(joint.constrain_pos_1));
      std::array<float, 3> c_p2(ConvertArray(joint.constrain_pos_2));
      std::array<float, 3> c_r1(ConvertArray(joint.constrain_rot_1));
      std::array<float, 3> c_r2(ConvertArray(joint.constrain_rot_2));
      Float3 s_p(ConvertFloat(joint.spring_pos));
      Float3 s_r(ConvertFloat(joint.spring_rot));
      Float3 p(ConvertFloat(joint.joint_pos));
      Float3 r(ConvertFloat(joint.joint_rot));
      const Matrix rotation = XMMatrixRotationRollPitchYaw(r.x, r.y, r.z);
      const Matrix world = XMMatrixTranslation(p.x, p.y, p.z) * rotation; // ジョイントの行列（モデルローカル座標系）
      auto &rigidbody_a = m_rigidBodies[joint.joint_rigidbody_a];
      auto &rigidbody_b = m_rigidBodies[joint.joint_rigidbody_b];

      // ジョイントの行列（剛体ローカル座標系）
      const Matrix aMat = rigidbody_a.getWorld();
      const Matrix aInv = XMMatrixInverse(nullptr, aMat);
      const Matrix frameInA = world * aInv;

      const Matrix bMat = rigidbody_b.getWorld();
      const Matrix bInv = XMMatrixInverse(nullptr, bMat);
      const Matrix frameInB = world * bInv;

      m_bulletPhysics.Add6DofSpringConstraint(rigidbody_a, rigidbody_b,
        frameInA,
        frameInB,
        c_p1, c_p2, c_r1, c_r2, s_p, s_r);

      m_jointRelatedRigidIndex.push_back(joint.joint_rigidbody_a);
      m_jointMatrix.push_back(frameInA);
    }
    //const float length = 0.3f;
    //D3DXCreateBox(pDevice, length, length, length, &joint_mesh, 0);
    const std::uint_fast32_t rigidBodiesSize = static_cast<std::uint_fast32_t>(m_rigidBodies.size());
    for (std::uint_fast32_t i = 0; i < rigidBodiesSize; ++i) {
      if (m_rigidbodyRelatedBoneIndex[i] == 0xFFFF)
        continue;
      const mmd::Bone &bone = (*m_bones)[m_rigidbodyRelatedBoneIndex[i]];
      const Matrix m = XMMatrixMultiply(m_rigidbodyInit[i], bone.offsetMat);
      m_rigidMat.push_back(m);
      const Matrix tempInitOffsetMat = XMMatrixMultiply(bone.initMatML, m_rigidbodyOffset[i]);
      m_initOffsetMat.push_back(tempInitOffsetMat);
    }

  }

  void MmdPhysics::BoneUpdate(const Matrix &mat, Array<Mat4x4> &boneWorld) {
    //if (physicsEnabled) ;	// 物理シミュレーション
    const std::uint_fast32_t rigidBodiessize = static_cast<std::uint_fast32_t>(m_rigidBodies.size());
    //const MATRIX *rigidbodyoffset=&rigidbody_offset[0];
    int count = 0;
    for (std::uint_fast32_t i = 0; i < rigidBodiessize; ++i) {
      if (m_rigidbodyRelatedBoneIndex[i] == 0xFFFF) {
        count++;
        continue;
      }
      mmd::Bone& bone = (*m_bones)[m_rigidbodyRelatedBoneIndex[i]];
      auto &rigidBodie = m_rigidBodies[i];
      using namespace DirectX;
      switch (m_rigidbodyType[i]) {
      case 0: //bone追従
      {
        // ボーン追従タイプの剛体にボーン行列を設定
        // ボーンの移動量を剛体の初期姿勢に適用したものが剛体の現在の姿勢
        const XMMATRIX mxm = XMMatrixMultiply(m_rigidMat[i - count], m_bones->CalcBoneMatML(m_rigidbodyRelatedBoneIndex[i]));
        rigidBodie.MoveRigidBody(mxm);
        bone.extraBoneControl = false;
        break;
      }
      case 1: //物理演算
      {
        Matrix m = rigidBodie.getWorld();
        const XMMATRIX xmm = XMMatrixInverse(nullptr, mat);
        m = XMMatrixMultiply(m, xmm);
        bone.boneMatML = XMMatrixMultiply(m_initOffsetMat[i - count], m);
        break;
      }
      //物理演算(bone合わせ)
      case 2:
      {
        //うまく動いてるか不明
        bone.extraBoneControl = false;
        Matrix rigidWorld = m_initOffsetMat[i - count] * rigidBodie.getWorld();
        // ボーン位置あわせタイプの剛体の位置移動量にボーンの位置移動量を設定
        Matrix boneParentMat = m_bones->CalcParentBoneMat(m_rigidbodyRelatedBoneIndex[i]);
        const Matrix boneMat = bone.boneMat * boneParentMat;
        rigidWorld.r[3] = boneMat.r[3];
        Matrix inv = XMMatrixInverse(nullptr, boneParentMat);
        rigidWorld = rigidWorld * inv;
        bone.boneMat = rigidWorld;

        //剛体の位置更新
        rigidWorld = rigidBodie.getWorld();
        //rigidWorld.r[3] = boneMat.r[3];
        rigidBodie.MoveRigidBody(rigidWorld);
        break;
      }

      }
    }
    m_bones->CalcWorld(mat, boneWorld);
  }

  // private : 剛体のワールド変換行列を作成
  // @param pos		剛体の位置		：MmdStruct::PmdRigidBody.pos_pos[3]
  // @param rot		剛体の回転		：MmdStruct::PmdRigidBody.pos_rot[3]
  // @param i			関連ボーン番号
  // @return			ワールド変換行列
  DirectX::XMMATRIX MmdPhysics::CreateRigidMatrix(const float* pos, const float* rot, int i) {
    Float3 p = Float3(pos[0], pos[1], pos[2]);
    if (i == 0xffff) {
      i = 0; //関連ボーンがない場合は0のセンターボーンが基準
    }
    DirectX::XMFLOAT3 f;
    DirectX::XMStoreFloat3(&f, (*m_bones)[i].initMatML.r[3]);
    // 関連ボーンがある場合は、ボーン相対座標からモデルローカル座標に変換。MmdStruct::PmdRigidBody.pos_posを参照
    p.x += f.x;
    p.y += f.y;
    p.z += f.z;
    (*m_bones)[i].extraBoneControl = true;
    const DirectX::XMMATRIX trans = DirectX::XMMatrixTranslation(p.x, p.y, p.z);
    const DirectX::XMMATRIX rotation = DirectX::XMMatrixRotationRollPitchYaw(rot[0], rot[1], rot[2]);
    return rotation * trans;
  }

#endif // USE_BULLET_PHYSICS

}
