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
  namespace {
    /// <summary>null時にget、*で参照するとエラーが出るスマポ</summary>
    template<class _Ty, class _Dx = std::default_delete<_Ty>> class null_err_unique_ptr;
    /// <summary>null時にget、*で参照するとエラーが出るスマポ</summary>
    template<class _Ty, class  _Dx> class null_err_unique_ptr :public std::unique_ptr<_Ty> {
    public:
      typedef std::unique_ptr<_Ty> Base;
      null_err_unique_ptr(_Ty *t) : Base(t) {}
      template<class _Ty2, class _Dx2>
      null_err_unique_ptr(null_err_unique_ptr<_Ty2, _Dx2>&& _Right) {	// construct shared_ptr object that aliases _Right
        Base::reset(_Right.release());
      }
      null_err_unique_ptr() {}
#define _L(x)  __L(x)
#define __L(x)  L##x

#define MY_ASSERT(_Expression,msg) if(!(_Expression)){ _CrtDbgReportW(_CRT_WARN, _L(__FILE__), __LINE__, NULL, msg); _CrtDbgBreak();}
      _Ty &operator *() { return *this->get(); }
      _Ty *get() const {
        _Ty *t = std::unique_ptr<_Ty>::get();
        MY_ASSERT(t, L"null pointer have get\n");
        return t;
      }

    };
  }
  namespace bullet {

    /// <summary>Bullet形式とSiv3D形式の変換</summary>
    /// <param name="v">変換するVECTOR3</param>
    /// <returns>pOutのポインタ</returns>
    inline btVector3 ConvertVectorDxToBt(const Float3 &v) {
      const float x = static_cast<float>(v.x);
      const float y = static_cast<float>(v.y);
      const float z = static_cast<float>(-v.z);
      return btVector3(x, y, z);
    }

    /// /// <summary>Bullet形式とDirectX形式の変換</summary>
    /// <param name="v">変換するbtVector3</param>
    inline Float3 ConvertVectorBtToDx(const btVector3 &v) {
      return Float3(v.x(), v.y(), -v.z());
    }

    ///  Bullet形式とDirectX形式の変換
    /// <param name="v">変換するbtVector3</param>
    /// <returns>pOutのポインタ</returns>
    ///  Bullet形式とDirectX形式の変換
    inline btTransform ConvertMatrixDxToBt(const Matrix &m) {
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

    /// <summary>Bullet形式とDirectX形式の変換</summary>
    /// <param name="v">変換するbtTransform</param>
    /// <returns>pOutのポインタ</returns>
    inline Matrix ConvertMatrixBtToDx(const btTransform &t) {
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

      Data() {
      }

      ~Data() {
        //どちらかが消えた場合はあっても意味が無いので直ぐに削除
        for (auto &it : Constraintnum) {
          dynamics_world_->removeConstraint(it.get());
          it.reset();
        }
        RemoveRigidBody();
      }

      void RemoveRigidBody() {
        if (body) dynamics_world_->removeRigidBody(body.get());
      }

      void AddRigidBody(std::uint16_t group, std::uint16_t mask) {
        if (body) dynamics_world_->addRigidBody(body.get(), group, mask);
      }

      void MoveRigidBody(const Matrix &world) {
        btTransform trans = bullet::ConvertMatrixDxToBt(world);
        body->getMotionState()->setWorldTransform(trans);
        //m_bulletdata[num]->body->setWorldTransform(trans);
      }

      void SetMatrixRigidBody(const Matrix &world) {
        btTransform trans = bullet::ConvertMatrixDxToBt(world);
        motionState->setWorldTransform(trans);
        //m_bulletdata[num]->body->setWorldTransform(trans);
      }

      btTransform GetWorld() {
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        return trans;
      }

      /// <summary>ジョイント</summary>
      std::vector<std::shared_ptr<btTypedConstraint>> Constraintnum;
      std::unique_ptr<btRigidBody> body;
      std::unique_ptr<btCollisionShape> shape;
      std::unique_ptr<btDefaultMotionState> motionState;
      std::unique_ptr<btCollisionShape> heroshape;

    private:

      std::unique_ptr<btDynamicsWorld> dynamics_world_;

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

  /// /// 物理演算Bulletのラッパークラス
  class BulletPhysics final {

    std::shared_ptr<btDefaultCollisionConfiguration> m_collisionConfiguration;
    std::shared_ptr<btCollisionDispatcher> m_dispatcher;
    std::shared_ptr<btDbvtBroadphase> m_overlappingPairCache;
    std::shared_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::vector<std::shared_ptr<btTypedConstraint>> m_constraint;

  public:

    std::shared_ptr<btDiscreteDynamicsWorld> m_dynamicsWorld;

    std::shared_ptr<bullet::Data> CreateCompoundShape(null_err_unique_ptr<btCollisionShape> box, const Matrix &world,
      float mass, float restitution, float friction, float linear_damp, float angular_damp, bool kinematic, unsigned short group, unsigned short mask, const btVector3 &coord);

    std::shared_ptr<bullet::Data> CreateShape(null_err_unique_ptr<btCollisionShape> shape, const Matrix &world,
      float mass, float restitution, float friction, float linear_damp,
      float angular_damp, bool kinematic, unsigned short group, unsigned short mask);

    typedef bool(*ProcessedCallBack)(btManifoldPoint& p, void* a, void* b);

    BulletPhysics(const Vec3& gravity, ProcessedCallBack callback = nullptr);
    ~BulletPhysics();

    /// 剛体オブジェクト生成
    /// 質量0, kinematicをfalseにすると、動かないstatic剛体になる。
    /// 質量0, kinematicをtrueにすると、手動で動かせるが、物理演算の影響を受けないKinematic剛体になる
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

    /// 6軸ジョイントを追加
    /// @param bodyA 剛体A
    /// @param bodyB 剛体B
    /// @param frameInA ジョイントのワールド変換行列(剛体Aローカル座標系)
    /// @param frameInB ジョイントのワールド変換行列(剛体Bローカル座標系)
    /// @param c_p1 移動制限1
    /// @param c_p2 移動制限2
    /// @param c_r1 回転制限1
    /// @param c_r2 回転制限2
    /// @param stiffness バネ剛性(平行移動x, y, z, 回転移動x, y, zの順の6要素)
    /// 6軸ジョイントを追加
    /// @param bodyA 剛体A
    /// @param bodyB 剛体B
    /// @param frameInA ジョイントのワールド変換行列(剛体Aローカル座標系)
    /// @param frameInB ジョイントのワールド変換行列(剛体Bローカル座標系)
    /// @param c_p1 移動制限1
    /// @param c_p2 移動制限2
    /// @param c_r1 回転制限1
    /// @param c_r2 回転制限2
    /// @param stiffnessPos バネ剛性(平行移動x, y, z の3要素)
    /// @param stiffnessRot バネ剛性(回転移動x, y, z の3要素)
    void Add6DofSpringConstraint(std::shared_ptr<bullet::Data> bodyA, std::shared_ptr<bullet::Data> bodyB,
      const btTransform& frameInA, const btTransform& frameInB,
      const Float3& c_p1, const Float3& c_p2,
      const Float3& c_r1, const Float3& c_r2,
      const Float3 &stiffnessPos, const Float3 &stiffnessRot);

    // 剛体を移動
    void MoveRigidBody(int num, const Matrix &world);
    void SetMatrixRigidBody(int num, const Matrix &world);
    // 物理演算の世界の時間を1/60秒進める
    void StepSimulation();
    void DebugDraw();

  };
}

#endif

