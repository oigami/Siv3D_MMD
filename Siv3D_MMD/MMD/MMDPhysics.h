#ifndef MMDPHYSICS_H
#define MMDPHYSICS_H
//#pragma unmanaged
#include"../BulletPhysics/BulletPhysics.h"
#include<vector>
#include<memory>
#include "../include/PMDStruct.h"
#include "../include/MMDBone.h"
//#include"VmdMotionController.h"
namespace s3d_mmd {
  namespace mmd {

    struct Body {

      Body(std::shared_ptr<s3d_bullet::bullet::Data> body, short group, short mask) {
        group_ = group;
        body_ = body;
        mask_ = mask;
      }

      Body() {};

      short group_;
      short mask_;
      std::shared_ptr<s3d_bullet::bullet::Data> body_;

    };

  }

  class MmdPhysics {
    typedef std::shared_ptr<s3d_bullet::BulletPhysics> BulletPhysicsPtr;
    
    /// �S�{�[���z��ւ̃|�C���^���Z�b�g
    /// @param bones �S�{�[���z��ւ̃|�C���^
    void SetBones(std::shared_ptr<mmd::Bones> bones);
    
    /// ���̂��쐬
    /// @param pmdRigidBodies Pmd���̔z��
    /// @param pmdBones Pmd�{�[���z��
    void CreateRigid(const std::vector<s3d_mmd::pmd::RigidBody> &pmdRigidBodies);

    /// �W���C���g���쐬
    /// @param pmdBones Pmd�W���C���g�z��
    void CreateJoint(const std::vector<s3d_mmd::pmd::Joint> &pmdJoints);

  public:

    MmdPhysics(BulletPhysicsPtr bulletPhysics);
    ~MmdPhysics();

    void Create(std::shared_ptr<mmd::Bones> bones,
      const std::vector<pmd::RigidBody> &pmdRigidBodies,
      const std::vector<pmd::Joint> &pmdJoints);

    void Destroy();

   
   
    /// �{�[���s����X�V
    /// @param �������Z�g�p�\
    void BoneUpdate(const Matrix &mat);

    /// ���̃��b�V����`�悷��
    void DrawRigidMesh(const Matrix &world);

    /// �W���C���g���b�V����`�悷��
    void DrawJointMesh(const Matrix &world);

    /*void SetCollisionDetection(bool flag,const MATRIX &mat){
      if(flag){
      btTransform btrans;
      MATRIX m;int i=0;
      for(auto it=rigidBodies.begin(); it!=rigidBodies.end(); it++,i++){
      XMStoreFloat4x4(&m,XMMatrixMultiply(XMLoadFloat4x4(&rigidbody_init[i]),XMLoadFloat4x4(&mat)));
      bulletPhysics->ConvertMatrixDxToBt(&btrans,m);
      bulletPhysics->GetbtRigidBody(it->body)->setWorldTransform(btrans);
      bulletPhysics->AddRigidBody(it->body,it->group,it->mask);
      }
      }
      else{
      for(auto it=rigidBodies.begin(); it!=rigidBodies.end(); it++){
      bulletPhysics->RemoveRigidBody(it->body);
      }
      }
      }*/
      // �⏕�֐�
  private:

    DirectX::XMMATRIX CreateRigidMatrix(const float* pos, const float* rot, int i);
    void StartWireframe(const Matrix &world);
    void EndWireframe();

  private:
    BulletPhysicsPtr m_bulletPhysics;
    std::shared_ptr<mmd::Bones> m_bones;    // �S�{�[���z��ւ̃|�C���^
    Array<mmd::Body> m_rigidBodies;         // 1���b�V���ɑ΂��鍄�̔z��
    Array<int> m_rigidbodyRelatedBoneIndex; // �e���̂Ɋ֘A����{�[���̃C���f�b�N�X
    Array<int> m_rigidbodyType;             // �e���̂̃^�C�v
    Array<Matrix> m_rigidbodyInit;          // �e���̂̏����p���s��
    Array<Matrix> m_rigidbodyOffset;        // �e���̂̃I�t�Z�b�g�s��
    //vector<ID3DXMesh*> rigidbody_mesh;    // �e���̂̃��b�V��
    //ID3DXMesh* joint_mesh;                // �W���C���g�̃��b�V��
    Array<int> m_jointRelatedRigidIndex;    // �e�W���C���g�̊֘A����
    Array<Matrix> m_jointMatrix;            // �e�W���C���g�̎p���s��(����A���[�J�����W�n)
    Array<Matrix> m_rigidMat;               //rigidbody_init*bones.offsetMatML
    Array<Matrix> m_initOffsetMat;          // bones.initMatML*rigidbody_offset

  };;

}
#endif
