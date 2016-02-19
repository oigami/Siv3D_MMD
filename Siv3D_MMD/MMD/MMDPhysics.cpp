#include "../include/MMD.h"
#ifdef USE_BULLET_PHYSICS
#include "MmdPhysics.h"
#include "../BulletPhysics/BulletPhysics.h"
#endif
#include <DirectXMath.h>
using namespace std;
namespace s3d_mmd {

#ifdef USE_BULLET_PHYSICS
  using namespace s3d_bullet;
  //#pragma unmanaged
  MmdPhysics::MmdPhysics(BulletPhysicsPtr bulletPhysics) {
    this->m_bulletPhysics = bulletPhysics;
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
    for (auto &it : m_rigidBodies) {
      it.body_->RemoveRigidBody();
      it.body_ = nullptr;
    }
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

  /// ���̂��쐬
  /// @param pmdRigidBodies Pmd���̔z��
  /// @param pmdBones Pmd�{�[���z��

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
      if (pmdRigidBodies[i].shape_type == 0) {		// ��
        const float radius = pmdRigidBodies[i].shape_w;
        auto body = m_bulletPhysics->CreateSphere(
          radius, m_rigidbodyInit[i], pmdRigidBodies[i].rigidbody_weight, pmdRigidBodies[i].rigidbody_recoil, pmdRigidBodies[i].rigidbody_friction, pmdRigidBodies[i].rigidbody_pos_dim,
          pmdRigidBodies[i].rigidbody_rot_dim, pmdRigidBodies[i].rigidbody_type == 0, 1 << pmdRigidBodies[i].rigidbody_group_index, pmdRigidBodies[i].rigidbody_group_target);
        m_rigidBodies[i] = mmd::Body(body, 1 << pmdRigidBodies[i].rigidbody_group_index, pmdRigidBodies[i].rigidbody_group_target);
        const std::uint32_t slices = 10, stacks = 5;

      } else if (pmdRigidBodies[i].shape_type == 1) {	// ��
        const float width = 2 * pmdRigidBodies[i].shape_w, height = 2 * pmdRigidBodies[i].shape_h, depth = 2 * pmdRigidBodies[i].shape_d;
        auto body = m_bulletPhysics->CreateBox(
          width, height, depth, m_rigidbodyInit[i], pmdRigidBodies[i].rigidbody_weight, pmdRigidBodies[i].rigidbody_recoil, pmdRigidBodies[i].rigidbody_friction, pmdRigidBodies[i].rigidbody_pos_dim,
          pmdRigidBodies[i].rigidbody_rot_dim, pmdRigidBodies[i].rigidbody_type == 0, 1 << pmdRigidBodies[i].rigidbody_group_index, pmdRigidBodies[i].rigidbody_group_target);
        m_rigidBodies[i] = mmd::Body(body, 1 << pmdRigidBodies[i].rigidbody_group_index, pmdRigidBodies[i].rigidbody_group_target);

      } else if (pmdRigidBodies[i].shape_type == 2) {	// �J�v�Z��
        const float radius = pmdRigidBodies[i].shape_w, height = pmdRigidBodies[i].shape_h;
        auto body = m_bulletPhysics->CreateCapsule(
          radius, height, m_rigidbodyInit[i], pmdRigidBodies[i].rigidbody_weight, pmdRigidBodies[i].rigidbody_recoil, pmdRigidBodies[i].rigidbody_friction, pmdRigidBodies[i].rigidbody_pos_dim,
          pmdRigidBodies[i].rigidbody_rot_dim, (pmdRigidBodies[i].rigidbody_type == 0), 0x1 << (pmdRigidBodies[i].rigidbody_group_index), pmdRigidBodies[i].rigidbody_group_target);
        m_rigidBodies[i] = mmd::Body(body, 1 << pmdRigidBodies[i].rigidbody_group_index, pmdRigidBodies[i].rigidbody_group_target);
        //const UINT slices = 10, stacks = 5;
      }
    }
  }

  template<class T, size_t n> Float3 ConvertFloat3(T(&val)[n]) {
    static_assert(n == 3 || n == 4, "");
    return Float3(val[0], val[1], val[2]);
  }

  void MmdPhysics::CreateJoint(const vector<s3d_mmd::pmd::Joint> &pmdJoints) {
    const size_t pmdJointsSize = pmdJoints.size();
    m_jointRelatedRigidIndex.reserve(pmdJointsSize);
    m_jointMatrix.reserve(pmdJointsSize);

    for (auto &joint : pmdJoints) {
      using namespace DirectX;
      Float3 c_p1(ConvertFloat3(joint.constrain_pos_1));
      Float3 c_p2(ConvertFloat3(joint.constrain_pos_2));
      Float3 c_r1(ConvertFloat3(joint.constrain_rot_1));
      Float3 c_r2(ConvertFloat3(joint.constrain_rot_2));
      Float3 s_p(ConvertFloat3(joint.spring_pos));
      Float3 s_r(ConvertFloat3(joint.spring_rot));
      Float3 p(ConvertFloat3(joint.joint_pos));
      Float3 r(ConvertFloat3(joint.joint_rot));
      const Matrix rotation = XMMatrixRotationRollPitchYaw(r.x, r.y, r.z);
      const Matrix world = XMMatrixTranslation(p.x, p.y, p.z) * rotation; // �W���C���g�̍s��i���f�����[�J�����W�n�j
      const auto &rigidbody_a = m_rigidBodies[joint.joint_rigidbody_a].body_;
      const auto &rigidbody_b = m_rigidBodies[joint.joint_rigidbody_b].body_;

      // �W���C���g�̍s��i���̃��[�J�����W�n�j
      const Matrix aMat = bullet::ConvertMatrixBtToDx(rigidbody_a->GetWorld());
      const Matrix aInv = XMMatrixInverse(nullptr, aMat);
      const Matrix frameInA = world * aInv;

      const Matrix bMat = bullet::ConvertMatrixBtToDx(rigidbody_b->GetWorld());
      const Matrix bInv = XMMatrixInverse(nullptr, bMat);
      const Matrix frameInB = world * bInv;

      m_bulletPhysics->Add6DofSpringConstraint(rigidbody_a, rigidbody_b,
        bullet::ConvertMatrixDxToBt(frameInA),
        bullet::ConvertMatrixDxToBt(frameInB),
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

  void MmdPhysics::BoneUpdate(const Matrix &mat) {
    m_bulletPhysics->StepSimulation();
    //if (physicsEnabled) ;	// �����V�~�����[�V����
    const std::uint_fast32_t rigidBodiessize = static_cast<std::uint_fast32_t>(m_rigidBodies.size());
    //const MATRIX *rigidbodyoffset=&rigidbody_offset[0];
    int count = 0;
    for (std::uint_fast32_t i = 0; i < rigidBodiessize; ++i) {
      if (m_rigidbodyRelatedBoneIndex[i] == 0xFFFF) {
        count++;
        continue;
      }
      Matrix m, mm;
      mmd::Bone& bone = (*m_bones)[m_rigidbodyRelatedBoneIndex[i]];
      auto &rigidBodie = m_rigidBodies[i].body_;
      using namespace DirectX;
      switch (m_rigidbodyType[i]) {
      case 2:case 0: //bone�Ǐ]
      {
        // �{�[���Ǐ]�^�C�v�̍��̂Ƀ{�[���s���ݒ�
        // �{�[���̈ړ��ʂ����̂̏����p���ɓK�p�������̂����̂̌��݂̎p��
        const XMMATRIX mxm = XMMatrixMultiply(m_rigidMat[i - count], m_bones->CalcBoneMatML(m_rigidbodyRelatedBoneIndex[i]));
        m = mxm;
        mm = XMMatrixMultiply(mxm, mat);
        rigidBodie->MoveRigidBody(m);
        break;
      }
       //�������Z(bone���킹)
      {
        // �{�[���ʒu���킹�^�C�v�̍��̂̈ʒu�ړ��ʂɃ{�[���̈ʒu�ړ��ʂ�ݒ�
        const Vector m1 = m_bones->CalcBoneMatML(m_rigidbodyRelatedBoneIndex[i]).r[3];
        const Vector m2 = bone.initMatML.r[3];
        const Vector initMat = m_rigidbodyInit[i].r[3];
        m = m_rigidbodyInit[i];
#ifndef _XM_NO_INTRINSICS_
        const float w = DirectX::XMVectorGetW(m.r[3]);
        const Vector v = initMat + m1 - m2;
        m.r[3] = DirectX::XMVectorSetW(v, w);
#else
        m._41 = initm[0] + m1[0] - m2[0];
        m._42 = initm[1] + m1[1] - m2[1];
        m._43 = initm[2] + m1[2] - m2[2];
#endif // !_XM_NO_INTRINSICS_

        mm = XMMatrixMultiply(m, mat);
        rigidBodie->MoveRigidBody(m);
        //m_bulletPhysics->MoveRigidBody(rigidBodie, mm);
        //XMMatrixMultiply(&m_rigidMat,&bone.initMatML,&m_rigidbodyOffset[i]);
        break;
      }
      case 1: //�������Z
      {
        m = bullet::ConvertMatrixBtToDx(rigidBodie->GetWorld());
        ///*body���������Ƃ��Ɏg�������܂������Ȃ�
        const XMMATRIX xmm = XMMatrixInverse(nullptr, mat);
        m = XMMatrixMultiply(m, xmm);

        mm = m_rigidMat[i - count];
        break;
      }
      }
      bone.boneMatML = XMMatrixMultiply(m_initOffsetMat[i - count], m);
    }
  }

  /// ���̃��b�V����`�悷��

  inline void MmdPhysics::DrawRigidMesh(const Matrix & world) {
    StartWireframe(world);
    for (unsigned int i = 0; i < m_rigidBodies.size(); ++i) {
      Matrix w = bullet::ConvertMatrixBtToDx(m_rigidBodies[i].body_->GetWorld());
      using namespace DirectX;
      const XMMATRIX xmm = XMMatrixMultiply(w, world);

      if (m_rigidBodies[i].body_->body->getCollisionShape()->getShapeType() == CAPSULE_SHAPE_PROXYTYPE) {
        XMMATRIX r = XMMatrixRotationX(XM_PI / 2);
        w = r * xmm;	// PMD, bullet�̃J�v�Z����Y�����S�ADirectX�̉~���v���~�e�B�u��Z�����S�Ȃ̂�X���𒆐S��90����]������
      }
      //pDevice->SetTransform(D3DTS_WORLD, &w);
      //rigidbody_mesh[i]->DrawSubset(0);
    }
    EndWireframe();
  }

  void MmdPhysics::DrawJointMesh(const Matrix &world) {
    StartWireframe(world);
    for (unsigned int i = 0; i < m_jointRelatedRigidIndex.size(); ++i) {
      Matrix w = m_jointMatrix[i], m;
      ///w*=*bulletPhysics->GetWorld(&m, rigidBodies[joint_relatedRigidIndex[i]].body);
      //w*=(*world);
      //pDevice->SetTransform(D3DTS_WORLD, &w);
      //joint_mesh->DrawSubset(0);
    }
    EndWireframe();
  }

  // private : ���̂̃��[���h�ϊ��s����쐬
  // @param pos		���̂̈ʒu		�FMmdStruct::PmdRigidBody.pos_pos[3]
  // @param rot		���̂̉�]		�FMmdStruct::PmdRigidBody.pos_rot[3]
  // @param i			�֘A�{�[���ԍ�
  // @return			���[���h�ϊ��s��
  DirectX::XMMATRIX MmdPhysics::CreateRigidMatrix(const float* pos, const float* rot, int i) {
    Float3 p = Float3(pos[0], pos[1], pos[2]);
    if (i != 0xFFFF) {
      DirectX::XMFLOAT3 f;
      DirectX::XMStoreFloat3(&f, (*m_bones)[i].initMatML.r[3]);
      // �֘A�{�[��������ꍇ�́A�{�[�����΍��W���烂�f�����[�J�����W�ɕϊ��BMmdStruct::PmdRigidBody.pos_pos���Q��
      p.x += f.x;
      p.y += f.y;
      p.z += f.z;
      (*m_bones)[i].extraBoneControl = true;
    }
    const DirectX::XMMATRIX trans = DirectX::XMMatrixTranslation(p.x, p.y, p.z);
    const DirectX::XMMATRIX rotation = DirectX::XMMatrixRotationRollPitchYaw(rot[0], rot[1], rot[2]);
    return rotation * trans;
  }

  // private: ���C���[�t���[���̕`����J�n����
  void MmdPhysics::StartWireframe(const Matrix &world) {
    //D3DMATERIAL9 material = {{0, 0, 0, 1.0f}, color};
    Matrix view, projection;
    //camera->GetMatrix(&view, &projection);
    /*pDevice->SetLight(0, light);
    pDevice->LightEnable(0, TRUE) ;
    pDevice->SetRenderState(D3DRS_LIGHTING, TRUE);
    pDevice->SetMaterial(&material);
    pDevice->SetRenderState(D3DRS_ZENABLE, FALSE);
    pDevice->SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
    pDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    pDevice->SetTransform(D3DTS_VIEW, &view);
    pDevice->SetTransform(D3DTS_PROJECTION, &projection);*/
  }

  // private: ���C���[�t���[���̕`����I������
  void MmdPhysics::EndWireframe() {
    //pDevice->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
    //pDevice->SetRenderState(D3DRS_ZENABLE, TRUE);
  }
#endif // USE_BULLET_PHYSICS

}
