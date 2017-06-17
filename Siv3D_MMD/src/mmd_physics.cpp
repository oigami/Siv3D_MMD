#include <MMD/MMD.h>
#include "mmd_physics.h"

namespace s3d_mmd
{
  MmdPhysics::MmdPhysics(physics3d::Physics3DWorld world) : m_world(world) { }

  MmdPhysics::~MmdPhysics()
  {
    Destroy();
  }

  void MmdPhysics::create(std::shared_ptr<mmd::Bones> bones,
                          const std::vector<pmd_struct::RigidBody>& pmdRigidBodies,
                          const std::vector<pmd_struct::Joint>& pmdJoints)
  {
    SetBones(bones);
    CreateRigid(pmdRigidBodies);
    CreateJoint(pmdJoints);
  }

  void MmdPhysics::Destroy()
  {
    m_6DofSpringConstraint.clear();
    m_bones = nullptr;
    m_rigidBodies.clear();
    m_rigidbodyRelatedBoneIndex.clear();
    m_rigidbodyType.clear();
    m_rigidbodyInit.clear();
    m_rigidbodyInvInit.clear();
    m_jointRelatedRigidIndex.clear();
    m_jointMatrix.clear();
    m_rigidMat.clear();
    m_initOffsetMat.clear();
  }

  void MmdPhysics::SetBones(std::shared_ptr<mmd::Bones> bones)
  {
    this->m_bones = bones;
  }

  void MmdPhysics::CreateRigid(const std::vector<s3d_mmd::pmd_struct::RigidBody>& pmdRigidBodies)
  {
    const int pmdRigidBodiesSize = static_cast<int>(pmdRigidBodies.size());
    m_rigidbodyRelatedBoneIndex.resize(pmdRigidBodiesSize);
    m_rigidbodyType.resize(pmdRigidBodiesSize);
    m_rigidbodyInit.resize(pmdRigidBodiesSize);
    m_rigidbodyInvInit.resize(pmdRigidBodiesSize);
    m_rigidBodies.resize(pmdRigidBodiesSize);

    for ( int i = 0; i < pmdRigidBodiesSize; ++i )
    {
      auto&& item = pmdRigidBodies[i];
      m_rigidbodyRelatedBoneIndex[i] = item.rigidbody_rel_bone_index;
      m_rigidbodyType[i] = item.rigidbody_type;
      const auto matrix = CreateRigidMatrix(item.pos_pos, item.pos_rot, item.rigidbody_rel_bone_index);
      const auto center = ToVec3(matrix.first);
      const Mat4x4 world = matrix.second.toMatrix() * DirectX::XMMatrixTranslationFromVector(matrix.first);
      const Mat4x4 world_inv = world.inverse();
      m_rigidbodyInit[i] = world;
      m_rigidbodyInvInit[i] = world_inv;
      physics3d::Physics3DBody rigidBody;

      const auto bodyType = item.rigidbody_type == 0 ? physics3d::Physics3DBodyType::Kinematic : physics3d::Physics3DBodyType::Dynamic;
      const physics3d::Physics3DFilter filter(1 << item.rigidbody_group_index, item.rigidbody_group_target);
      const physics3d::Physics3DMaterial material(item.rigidbody_weight, item.rigidbody_recoil, item.rigidbody_friction, item.rigidbody_pos_dim, item.rigidbody_rot_dim);

      if ( item.shape_type == 0 )
      { // 球
        const float radius = item.shape_w;
        rigidBody = m_world.createSphere(center, Sphere(radius), material, filter, bodyType);
      }
      else if ( item.shape_type == 1 )
      { // 箱
        const float width = 2 * item.shape_w;
        const float height = 2 * item.shape_h;
        const float depth = 2 * item.shape_d;

        rigidBody = m_world.createBox(center, Box({ 0, 0, 0 }, width, height, depth), material, filter, bodyType);
      }
      else if ( item.shape_type == 2 )
      { // カプセル
        const float radius = item.shape_w, height = item.shape_h;

        rigidBody = m_world.createCapsule(center, s3d::Cylinder(radius, height), material, filter, bodyType);
      }

      rigidBody.setTransform(world);
      m_rigidBodies[i] = std::move(rigidBody);
    }
  }

  void MmdPhysics::CreateJoint(const Array<s3d_mmd::pmd_struct::Joint>& pmdJoints)
  {
    const size_t pmdJointsSize = pmdJoints.size();
    m_jointRelatedRigidIndex.reserve(pmdJointsSize);
    m_jointMatrix.reserve(pmdJointsSize);
    m_6DofSpringConstraint.reserve(pmdJointsSize);

    for ( auto& joint : pmdJoints )
    {
      const Float3& pos = joint.joint_pos;
      const Float3& r = joint.joint_rot;

      const Quaternion rotation = Quaternion::RollPitchYaw(r.z, r.x, r.y);
      const Mat4x4 world = rotation.toMatrix() * Mat4x4::Translate(pos.x, pos.y, pos.z);
      //const Mat4x4 world = Mat4x4::AffineTransform(1.0, rotation, joint.joint_pos);
      auto& rigidbodyA = m_rigidBodies[joint.joint_rigidbody_a];
      auto& rigidbodyB = m_rigidBodies[joint.joint_rigidbody_b];

      // ジョイントの行列（剛体ローカル座標系）
      physics3d::Physics3D6DofSpringConstraintState state;

      const Mat4x4 aInv = rigidbodyA.getTransform().inverse();
      state.frameInA = world * aInv;

      const Mat4x4 bInv = rigidbodyB.getTransform().inverse();
      state.frameInB = world * bInv;

      state.linearLowerLimit = joint.constrain_lower_pos;
      state.linearUpperLimit = joint.constrain_upper_pos;

      state.angularLowerLimit = joint.constrain_lower_rot;
      state.angularUpperLimit = joint.constrain_upper_rot;

      state.stiffnessPos = joint.spring_pos;
      state.stiffnessRot = joint.spring_rot;

      m_6DofSpringConstraint.push_back(m_world.create6DofSpringConstraint(rigidbodyA, rigidbodyB, state));

      m_jointRelatedRigidIndex.push_back(joint.joint_rigidbody_a);
      m_jointMatrix.push_back(state.frameInA);
    }

    const std::uint_fast32_t rigidBodiesSize = static_cast<std::uint_fast32_t>(m_rigidBodies.size());
    for ( std::uint_fast32_t i = 0; i < rigidBodiesSize; ++i )
    {
      if ( m_rigidbodyRelatedBoneIndex[i] == 0xFFFF ) continue;
      const mmd::Bone& bone = (*m_bones)[m_rigidbodyRelatedBoneIndex[i]];
      Matrix m = m_rigidbodyInit[i];
      m.r[3] = m.r[3] + bone.offsetMat;
      m_rigidMat.push_back(m);
      const Matrix tempInitOffsetMat = math::Mul(bone.initMatML, m_rigidbodyInvInit[i]);
      m_initOffsetMat.push_back(tempInitOffsetMat);
    }
  }

  void MmdPhysics::boneUpdate(const Mat4x4& world, Array<Mat4x4>& boneWorld)
  {
    if ( !m_bones )
    {
      return;
    }
    const std::uint_fast32_t rigidBodiessize = static_cast<std::uint_fast32_t>(m_rigidBodies.size());

    int count = 0;
    for ( std::uint_fast32_t i = 0; i < rigidBodiessize; ++i )
    {
      if ( m_rigidbodyRelatedBoneIndex[i] == 0xFFFF )
      {
        count++;
        continue;
      }
      mmd::Bone& bone = (*m_bones)[m_rigidbodyRelatedBoneIndex[i]];
      auto& rigidBodie = m_rigidBodies[i];
      const Mat4x4 worldInv = world.inverse();
      switch ( m_rigidbodyType[i] )
      {
      case 0: //bone追従
      {
        // ボーン追従タイプの剛体にボーン行列を設定
        // ボーンの移動量を剛体の初期姿勢に適用したものが剛体の現在の姿勢
        // rigidBody_init * bone_init_inv * bone_now
        const Mat4x4 mxm = m_rigidMat[i - count] * m_bones->calcBoneMatML(m_rigidbodyRelatedBoneIndex[i]) * world;
        rigidBodie.setTransform(mxm);
        bone.extraBoneControl = false;
        break;
      }
      case 1: //物理演算
      {
        Mat4x4 rigidTransform = rigidBodie.getTransform();
        // bone_init * rigidBody_inv * rigidBody_now * world_inv
        bone.boneMatML = m_initOffsetMat[i - count] * rigidTransform;
        break;
      }

        //物理演算(bone合わせ)
      case 2:
      {
        //うまく動いてるか不明
        // TODO: http://mikudan.blog120.fc2.com/blog-entry-318.html を参考に書き換える
        bone.extraBoneControl = false;
        // bone_init * rigidBody_inv * rigidBody_now * world_inv
        Mat4x4 rigidWorld = m_initOffsetMat[i - count] * rigidBodie.getTransform();

        // ボーン位置あわせタイプの剛体の位置移動量にボーンの位置移動量を設定
        Mat4x4 boneParentMat = m_bones->calcParentBoneMat(m_rigidbodyRelatedBoneIndex[i]);

        rigidWorld *= boneParentMat.inverse();
        const Mat4x4 boneMat = bone.boneMat * boneParentMat;
        rigidWorld.r[3] = bone.boneMat.r[3];
        bone.boneMat = rigidWorld;

        //剛体の位置更新
        rigidWorld = rigidBodie.getTransform();
        rigidWorld.r[3] = boneMat.r[3];
        rigidBodie.setTransform(rigidWorld);
        break;
      }
      }
    }
    m_bones->calcWorld(world, boneWorld);
  }

  std::pair<Vector, Quaternion> MmdPhysics::CreateRigidMatrix(const s3d::Float3& pos, const s3d::Float3& rot, int i)
  {
    if ( i == 0xffff )
    {
      i = 0; //関連ボーンがない場合は0のセンターボーンが基準
    }
    // 関連ボーンがある場合は、ボーン相対座標からモデルローカル座標に変換。MmdStruct::PmdRigidBody.pos_posを参照
    auto p = ToVector(pos.x, pos.y, pos.z, 0.0f) + (*m_bones)[i].initMatML;

    (*m_bones)[i].extraBoneControl = true;
    const Quaternion rotation = Quaternion::RollPitchYaw(rot.z, rot.x, rot.y);
    return { p, rotation };
  }
}
