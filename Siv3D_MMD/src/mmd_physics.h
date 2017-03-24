#ifndef MMDPHYSICS_H
#define MMDPHYSICS_H
#include <MMD/physics3d.h>
#include <MMD/mmd_bone.h>
#include <MMD/pmd_struct.h>
#include <MMD/immd_physics.h>
#include<vector>
#include<memory>

namespace s3d_mmd
{
  class MmdPhysics : public IMMDPhysics
  {
    void SetBones(std::shared_ptr<mmd::Bones> bones);

    void CreateRigid(const std::vector<s3d_mmd::pmd_struct::RigidBody>& pmdRigidBodies);

    void CreateJoint(const std::vector<s3d_mmd::pmd_struct::Joint>& pmdJoints);

  public:

    MmdPhysics(physics3d::Physics3DWorld world = physics3d::Physics3DWorld());
    ~MmdPhysics();

    void create(std::shared_ptr<mmd::Bones> bones,
                const std::vector<pmd_struct::RigidBody>& pmdRigidBodies,
                const std::vector<pmd_struct::Joint>& pmdJoints) override;

    void boneUpdate(const Mat4x4& mat, Array<Mat4x4>& boneWorld) override;

    void Destroy();

  private:

    std::pair<Vector, Quaternion> CreateRigidMatrix(const s3d::Float3& pos, const s3d::Float3& rot, int i);

  private:
    physics3d::Physics3DWorld m_world;
    std::shared_ptr<mmd::Bones> m_bones; // 全ボーン配列へのポインタ
    Array<physics3d::Physics3DBody> m_rigidBodies; // 1メッシュに対する剛体配列
    Array<int> m_rigidbodyRelatedBoneIndex; // 各剛体に関連するボーンのインデックス
    Array<int> m_rigidbodyType; // 各剛体のタイプ
    Array<Matrix> m_rigidbodyInit; // 各剛体の初期姿勢行列
    Array<Matrix> m_rigidbodyInvInit; // 各剛体のオフセット行列

    //vector<ID3DXMesh*> rigidbody_mesh;    // 各剛体のメッシュ
    //ID3DXMesh* joint_mesh;                // ジョイントのメッシュ
    Array<int> m_jointRelatedRigidIndex; // 各ジョイントの関連剛体
    Array<Matrix> m_jointMatrix; // 各ジョイントの姿勢行列(剛体Aローカル座標系)
    Array<Matrix> m_rigidMat; // rigidbody_init * bones.offsetMatML
    Array<Matrix> m_initOffsetMat; // bones.initMatML * rigidbodyInvInit
    Array<physics3d::Physics3D6DofSpringConstraint> m_6DofSpringConstraint;
  };

  class MMDPhysicsFactory : public IMMDPhysicsFactory
  {
    bool m_isShared = true;
    physics3d::Physics3DWorld m_world;
  public:
    MMDPhysicsFactory(physics3d::Physics3DWorld world): m_world(world) { }

    MMDPhysicsFactory(): m_isShared(false) {}

    std::shared_ptr<IMMDPhysics> create() override
    {
      return std::make_shared<MmdPhysics>(m_isShared ? m_world : physics3d::Physics3DWorld());
    }
  };
}
#endif
