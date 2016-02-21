#pragma once
#ifndef INCLUDE_DETAIL_BULLET_PHYSICS_H
#define INCLUDE_DETAIL_BULLET_PHYSICS_H

#include <vector>
#include <memory>
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
    namespace detail {

      class Data {
      public:
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

        void MoveRigidBody(const btTransform &trans) {
          body->getMotionState()->setWorldTransform(trans);
        }

        void SetMatrixRigidBody(const btTransform &trans) {
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
  }
  namespace detail {
    class BulletPhysicsDetail final {

      class Pimpl;

      std::shared_ptr<Pimpl> m_pimpl;

    public:

      BulletPhysicsDetail(const btVector3& gravity);
      ~BulletPhysicsDetail();
      std::shared_ptr<bullet::detail::Data> CreateCompoundShape(std::unique_ptr<btCollisionShape> box, const btTransform &world,
        float mass, float restitution, float friction, float linear_damp, float angular_damp, bool kinematic, unsigned short group, unsigned short mask, const btVector3 &coord);

      std::shared_ptr<bullet::detail::Data> CreateShape(std::unique_ptr<btCollisionShape> shape, const btTransform &world,
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
      std::shared_ptr<bullet::detail::Data> CreateBox(float width, float height, float depth, const btTransform &world,
        float mass = 0, float restitution = 0, float friction = 0.5f,
        float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
        unsigned short group = 1, unsigned short mask = 0xFFFF,
        const btVector3 &coord = btVector3(0.0f, 0.0f, 0.0f));

      std::shared_ptr<bullet::detail::Data> CreateSphere(float radius, const btTransform &world,
        float mass = 0, float restitution = 0, float friction = 0.5f,
        float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
        unsigned short group = 1, unsigned short mask = 0xFFFF);

      std::shared_ptr<bullet::detail::Data> CreateCylinder(float radius, float length, const btTransform &world, // 中心軸はZ軸
        float mass = 0, float restitution = 0, float friction = 0.5f,
        float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
        unsigned short group = 1, unsigned short mask = 0xFFFF);

      std::shared_ptr<bullet::detail::Data> CreateCapsule(float radius, float height, const btTransform &world, // 中心軸はZ軸. heightは球の中心間の距離
        float mass = 0, float restitution = 0, float friction = 0.5f,
        float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
        unsigned short group = 1, unsigned short mask = 0xFFFF,
        const btVector3 &coord = btVector3(0.0f, 0.0f, 0.0f));

      // 拘束条件追加 
      void AddPointToPointConstraint(std::shared_ptr<bullet::detail::Data> body, const btVector3& pivot);

      void AddPointToPointConstraint(std::shared_ptr<bullet::detail::Data> bodyA,
        std::shared_ptr<bullet::detail::Data> bodyB,
        const btVector3& pivotInA, const btVector3& pivotInB);

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
      void Add6DofSpringConstraint(std::shared_ptr<bullet::detail::Data> bodyA, std::shared_ptr<bullet::detail::Data> bodyB,
        const btTransform& frameInA, const btTransform& frameInB,
        const std::array<float, 3>& c_p1, const std::array<float, 3>& c_p2,
        const std::array<float, 3>& c_r1, const std::array<float, 3>& c_r2,
        const btVector3 &stiffnessPos, const btVector3 &stiffnessRot);

      // 剛体を移動 
      void MoveRigidBody(int num, const btTransform &world);

      void SetMatrixRigidBody(int num, const btTransform &world);

      /// <summary>
      /// 物理演算の世界の時間を進める 
      /// </summary>
      void StepSimulation();

      void DebugDraw();

    };
  }
}

#endif
