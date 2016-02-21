#ifndef MMDPHYSICS_H
#define MMDPHYSICS_H
//#pragma unmanaged
#include"../include/BulletPhysics.h"
#include<vector>
#include<memory>
#include "../include/PMDStruct.h"
#include "../include/MMDBone.h"
//#include"VmdMotionController.h"
namespace s3d_mmd {
  namespace mmd {

    struct Body {

      Body(s3d_bullet::bullet::Data body, short group, short mask) :body_(body){
        group_ = group;
        mask_ = mask;
      }

      Body() :body_(){};

      short group_;
      short mask_;
      s3d_bullet::bullet::Data body_;

    };

  }

  class MmdPhysics {
    
    /// 全ボーン配列へのポインタをセット
    /// @param bones 全ボーン配列へのポインタ
    void SetBones(std::shared_ptr<mmd::Bones> bones);
    
    /// 剛体を作成
    /// @param pmdRigidBodies Pmd剛体配列
    /// @param pmdBones Pmdボーン配列
    void CreateRigid(const std::vector<s3d_mmd::pmd::RigidBody> &pmdRigidBodies);

    /// ジョイントを作成
    /// @param pmdBones Pmdジョイント配列
    void CreateJoint(const std::vector<s3d_mmd::pmd::Joint> &pmdJoints);

  public:

    MmdPhysics(s3d_bullet::BulletPhysics bulletPhysics);
    ~MmdPhysics();

    void Create(std::shared_ptr<mmd::Bones> bones,
      const std::vector<pmd::RigidBody> &pmdRigidBodies,
      const std::vector<pmd::Joint> &pmdJoints);

    void Destroy();

   
   
    /// ボーン行列を更新
    /// @param 物理演算使用可能
    void BoneUpdate(const Matrix &mat);

    /// 剛体メッシュを描画する
    void DrawRigidMesh(const Matrix &world);

    /// ジョイントメッシュを描画する
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
      // 補助関数
  private:

    DirectX::XMMATRIX CreateRigidMatrix(const float* pos, const float* rot, int i);
    void StartWireframe(const Matrix &world);
    void EndWireframe();

  private:
    s3d_bullet::BulletPhysics m_bulletPhysics;
    std::shared_ptr<mmd::Bones> m_bones;    // 全ボーン配列へのポインタ
    Array<mmd::Body> m_rigidBodies;         // 1メッシュに対する剛体配列
    Array<int> m_rigidbodyRelatedBoneIndex; // 各剛体に関連するボーンのインデックス
    Array<int> m_rigidbodyType;             // 各剛体のタイプ
    Array<Matrix> m_rigidbodyInit;          // 各剛体の初期姿勢行列
    Array<Matrix> m_rigidbodyOffset;        // 各剛体のオフセット行列
    //vector<ID3DXMesh*> rigidbody_mesh;    // 各剛体のメッシュ
    //ID3DXMesh* joint_mesh;                // ジョイントのメッシュ
    Array<int> m_jointRelatedRigidIndex;    // 各ジョイントの関連剛体
    Array<Matrix> m_jointMatrix;            // 各ジョイントの姿勢行列(剛体Aローカル座標系)
    Array<Matrix> m_rigidMat;               //rigidbody_init*bones.offsetMatML
    Array<Matrix> m_initOffsetMat;          // bones.initMatML*rigidbody_offset

  };;

}
#endif
