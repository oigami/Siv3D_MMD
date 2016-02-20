#pragma once
#ifndef INCLUDE_BULLET_PHYSICS_H
#define INCLUDE_BULLET_PHYSICS_H
#include <Siv3D.hpp>

//DirectXMathとの衝突を回避
#define BT_NO_SIMD_OPERATOR_OVERLOADS

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include "LinearMath/btDefaultMotionState.h"

#include "BulletDebug.h"

#ifdef _DEBUG

#pragma comment(lib, "BulletCollision_Debug.lib")
#pragma comment(lib, "BulletDynamics_Debug.lib")
#pragma comment(lib, "BulletSoftBody_Debug.lib")
#pragma comment(lib, "LinearMath_Debug.lib")

#else

#pragma comment(lib, "BulletCollision_vs2010.lib")
#pragma comment(lib, "BulletDynamics_vs2010.lib")
#pragma comment(lib, "BulletSoftBody_vs2010.lib")
#pragma comment(lib, "LinearMath_vs2010.lib")

#endif

namespace s3d_bullet {

  namespace bullet {

    /// <summary>
    /// Bullet形式とSiv3D形式の変換 
    /// </summary>
    /// <param name="v"> 変換するVECTOR3 </param>
    /// <returns> pOutのポインタ </returns>
    inline btVector3 ConvertVectorDxToBt(const Float3 &v) {
      const float x = static_cast<float>(v.x);
      const float y = static_cast<float>(v.y);
      const float z = static_cast<float>(-v.z);
      return btVector3(x, y, z);
    }

    /// <summary>
    /// Bullet形式とDirectX形式の変換 
    /// </summary>
    /// <param name="v"> 変換するbtVector3 </param>
    inline Float3 ConvertFloat3(const btVector3 &v) {
      return Float3(v.x(), v.y(), -v.z());
    }

    inline ColorF ConvertColor(const btVector3 &btColor) {
      return ColorF(btColor.x(), btColor.y(), btColor.z(), 1.f);
    }

    inline Quaternion ConvertQuaternion(const btQuaternion &q) {
      return Quaternion(-q.x(), -q.y(), q.z(), q.w());
    }

    /// <summary>
    /// Bullet形式とDirectX形式の変換 
    /// </summary>
    /// <param name="v"> 変換するbtVector3 </param>
    /// <returns> pOutのポインタ </returns>
    inline btTransform ConvertMatrix(const Matrix &m) {
      btTransform ret;
      // 鏡像変換＋転置 
#ifndef _XM_NO_INTRINSICS_
      DirectX::XMFLOAT4 r[4];
      for (auto& i : step(4))
        DirectX::XMStoreFloat4(r + i, m.r[i]);
      const btMatrix3x3 basis(
        r[0].x, r[1].x, -r[2].x,
        r[0].y, r[1].y, -r[2].y,
        -r[0].z, -r[1].z, r[2].z);
      ret.setOrigin(btVector3(r[3].x, r[3].y, -r[3].z));
#else
      const btMatrix3x3 basis(m._11, m._21, -m._31,
        m._12, m._22, -m._32,
        -m._13, -m._23, m._33);
      ret.setOrigin(btVector3(m._41, m._42, -m._43));
#endif // !_XM_NO_INTRINSICS_

      //btTransform bm;

      ret.setBasis(basis);
      return ret;
    }

    /// <summary>
    /// Bullet形式とDirectX形式の変換 
    /// </summary>
    /// <param name="v"> 変換するbtTransform </param>
    /// <returns> pOutのポインタ </returns>
    inline Matrix ConvertMatrix(const btTransform &t) {
      Matrix ret;
      const btMatrix3x3 basis = t.getBasis();
      const btVector3 R = basis.getColumn(0);
      const btVector3 U = basis.getColumn(1);
      const btVector3 L = basis.getColumn(2);
      const btVector3 P = t.getOrigin();
      // 鏡像変換＋転置 
#ifndef _XM_NO_INTRINSICS_
      ret.r[0] = DirectX::XMVectorSet(R.x(), R.y(), -R.z(), 0.f);
      ret.r[1] = DirectX::XMVectorSet(U.x(), U.y(), -U.z(), 0.f);
      ret.r[2] = DirectX::XMVectorSet(-L.x(), -L.y(), L.z(), 0.f);
      ret.r[3] = DirectX::XMVectorSet(P.x(), P.y(), -P.z(), 1.f);
#else
      ret._11 = R.x(), ret._12 = R.y(), ret._13 = -R.z(), ret._14 = 0.f;
      ret._21 = U.x(), ret._22 = U.y(), ret._23 = -U.z(), ret._24 = 0.f;
      ret._31 = -L.x(), ret._32 = -L.y(), ret._33 = L.z(), ret._34 = 0.f;
      ret._41 = P.x(), ret._42 = P.y(), ret._43 = -P.z(), ret._44 = 1.f;
#endif // !_XM_NO_INTRINSICS_

      return ret;
    }

    struct Data {

      Data(std::shared_ptr<btDynamicsWorld> world) : m_dynamicsWorld(world) {
      }

      ~Data() {
        //どちらかが消えた場合はあっても意味が無いので直ぐに削除
        for (auto &it : Constraintnum) {
          m_dynamicsWorld->removeConstraint(it.get());
          it.reset();
        }
        RemoveRigidBody();
      }

      void RemoveRigidBody() {
        if (body) m_dynamicsWorld->removeRigidBody(body.get());
      }

      void AddRigidBody(std::uint16_t group, std::uint16_t mask) {
        if (body) m_dynamicsWorld->addRigidBody(body.get(), group, mask);
      }

      void MoveRigidBody(const Matrix &world) {
        const btTransform trans = bullet::ConvertMatrix(world);
        body->getMotionState()->setWorldTransform(trans);
      }

      void SetMatrixRigidBody(const Matrix &world) {
        const btTransform trans = bullet::ConvertMatrix(world);
        motionState->setWorldTransform(trans);
      }

      btTransform GetWorld() {
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        return trans;
      }

      /// <summary>
      /// ジョイント 
      /// </summary>
      std::vector<std::shared_ptr<btTypedConstraint>> Constraintnum;
      std::unique_ptr<btRigidBody> body;
      std::unique_ptr<btCollisionShape> shape;
      std::unique_ptr<btDefaultMotionState> motionState;
      std::unique_ptr<btCollisionShape> heroshape;

    private:

      std::shared_ptr<btDynamicsWorld> m_dynamicsWorld;

    };

    struct Callback {
      int my_handlehandle;
      void *pointar;
    };

    struct Constraint {
      int bodyA;
      int bodyB;
      btGeneric6DofSpringConstraint* constraint;
    };

  }

  class BulletPhysics final {

    class Pimpl;

    std::shared_ptr<Pimpl> m_pimpl;

  public:

    BulletPhysics(const Vec3& gravity);
    ~BulletPhysics();

    std::shared_ptr<bullet::Data> CreateCompoundShape(std::unique_ptr<btCollisionShape> box, const Matrix &world,
      float mass, float restitution, float friction, float linear_damp, float angular_damp, bool kinematic, unsigned short group, unsigned short mask, const btVector3 &coord);

    std::shared_ptr<bullet::Data> CreateShape(std::unique_ptr<btCollisionShape> shape, const Matrix &world,
      float mass, float restitution, float friction, float linear_damp,
      float angular_damp, bool kinematic, unsigned short group, unsigned short mask);

    /// <summary>
    /// 剛体オブジェクト生成
    /// </summary>
    /// <param name="width"></param>
    /// <param name="height"></param>
    /// <param name="depth"></param>
    /// <param name="world"></param>
    /// <param name="mass"></param>
    /// <param name="restitution"></param>
    /// <param name="friction"></param>
    /// <param name="linear_damp"></param>
    /// <param name="angular_damp"></param>
    /// <param name="kinematic"></param>
    /// <param name="group"></param>
    /// <param name="mask"></param>
    /// <param name="coord"></param>
    /// <remarks>
    /// 質量0, kinematicをfalseにすると、動かないstatic剛体になる。
    /// 質量0, kinematicをtrueにすると、手動で動かせるが、物理演算の影響を受けないKinematic剛体になる 
    /// </remarks>
    /// <returns></returns>
    std::shared_ptr<bullet::Data> CreateBox(float width, float height, float depth, const Matrix &world,
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF,
      const btVector3 &coord = btVector3(0.0f, 0.0f, 0.0f));

    std::shared_ptr<bullet::Data> CreateSphere(float radius, const Matrix &world,
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF);

    std::shared_ptr<bullet::Data> CreateCylinder(float radius, float length, const Matrix &world, // 中心軸はZ軸
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF);

    std::shared_ptr<bullet::Data> CreateCapsule(float radius, float height, const Matrix &world, // 中心軸はZ軸. heightは球の中心間の距離
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF,
      const btVector3 &coord = btVector3(0.0f, 0.0f, 0.0f));

    // 拘束条件追加 
    void AddPointToPointConstraint(std::shared_ptr<bullet::Data> body, const Vec3& pivot);

    void AddPointToPointConstraint(std::shared_ptr<bullet::Data> bodyA,
      std::shared_ptr<bullet::Data> bodyB,
      const Vec3& pivotInA, const Vec3& pivotInB);

    /// <summary>
    /// 6軸ジョイントを追加 
    /// </summary>
    /// <param name="bodyA"> 剛体A </param>
    /// <param name="bodyB"> 剛体B </param>
    /// <param name="frameInA"> ジョイントのワールド変換行列(剛体Aローカル座標系) </param>
    /// <param name="frameInB"> ジョイントのワールド変換行列(剛体Bローカル座標系) </param>
    /// <param name="c_p1"> 移動制限1 </param>
    /// <param name="c_p2"> 移動制限2 </param>
    /// <param name="c_r1"> 回転制限1 </param>
    /// <param name="c_r2"> 回転制限2 </param>
    /// <param name="stiffnessPos"> バネ剛性(平行移動3要素) </param>
    /// <param name="stiffnessRot"> バネ剛性(回転移動3要素) </param>
    void Add6DofSpringConstraint(std::shared_ptr<bullet::Data> bodyA, std::shared_ptr<bullet::Data> bodyB,
      const btTransform& frameInA, const btTransform& frameInB,
      const Float3& c_p1, const Float3& c_p2,
      const Float3& c_r1, const Float3& c_r2,
      const Float3 &stiffnessPos, const Float3 &stiffnessRot);

    // 剛体を移動 
    void MoveRigidBody(int num, const Matrix &world);

    void SetMatrixRigidBody(int num, const Matrix &world);

    /// <summary>
    /// 物理演算の世界の時間を進める 
    /// </summary>
    void StepSimulation();

    void DebugDraw();

  };
}

#endif
