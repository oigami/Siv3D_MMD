#pragma once
#ifndef INCLUDE_BULLET_PHYSICS_H
#define INCLUDE_BULLET_PHYSICS_H

#include <Siv3D.hpp>

#include "../BulletPhysics/BulletRigidBody.h"
namespace s3d_bullet {
  class BulletPhysics;
  namespace bullet {
    namespace detail {
      class Data;
    }
    struct Data {

      class Pimpl;
      std::shared_ptr<Pimpl> m_pimpl;

      Data(std::shared_ptr<detail::Data> data);
      Data() = default;
      ~Data();

      void RemoveRigidBody();

      void AddRigidBody(std::uint16_t group, std::uint16_t mask);

      void MoveRigidBody(const Mat4x4 &trans);

      void SetMatrixRigidBody(const Mat4x4 &trans);

      Mat4x4 GetWorld() const;

      void release() {}
    };

  }

  class BulletPhysics final {

    class Pimpl;

    std::shared_ptr<Pimpl> m_pimpl;

  public:

    BulletPhysics(const Float3& gravity);
    ~BulletPhysics() = default;

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
    bullet::Data CreateBox(float width, float height, float depth, const Mat4x4 &world,
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF,
      const Float3 &coord = Float3(0.0f, 0.0f, 0.0f));

    bullet::Data CreateSphere(float radius, const Mat4x4 &world,
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF);

    bullet::Data CreateCylinder(float radius, float length, const Mat4x4 &world, // 中心軸はZ軸
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF);

    bullet::Data CreateCapsule(float radius, float height, const Mat4x4 &world, // 中心軸はZ軸. heightは球の中心間の距離
      float mass = 0, float restitution = 0, float friction = 0.5f,
      float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
      unsigned short group = 1, unsigned short mask = 0xFFFF,
      const Float3 &coord = Float3(0.0f, 0.0f, 0.0f));

    // 拘束条件追加 
    void AddPointToPointConstraint(bullet::Data body, const Vec3& pivot);

    void AddPointToPointConstraint(bullet::Data bodyA,
      bullet::Data bodyB,
      const Vec3& pivotInA, const Vec3& pivotInB);

    /// <summary>
    /// 6軸ジョイントを追加 
    /// </summary>
    /// <param name="bodyA"> 剛体A </param>
    /// <param name="bodyB"> 剛体B </param>
    /// <param name="frameInA"> ジョイントのワールド変換行列(剛体Aローカル座標系) </param>
    /// <param name="frameInB"> ジョイントのワールド変換行列(剛体Bローカル座標系) </param>
    /// <param name="c_p1"> 移動制限1(x,y,z) </param>
    /// <param name="c_p2"> 移動制限2(x,y,z) </param>
    /// <param name="c_r1"> 回転制限1(x,y,z) </param>
    /// <param name="c_r2"> 回転制限2(x,y,z) </param>
    /// <param name="stiffnessPos"> バネ剛性(平行移動) </param>
    /// <param name="stiffnessRot"> バネ剛性(回転移動) </param>
    void Add6DofSpringConstraint(bullet::Data bodyA, bullet::Data bodyB,
      const Mat4x4& frameInA, const Mat4x4& frameInB,
      const std::array<float, 3>& c_p1, const std::array<float, 3>& c_p2,
      const std::array<float, 3>& c_r1, const std::array<float, 3>& c_r2,
      const Float3 &stiffnessPos, const Float3 &stiffnessRot);

    // 剛体を移動 
    void MoveRigidBody(int num, const Matrix &world);

    void SetMatrixRigidBody(int num, const Matrix &world);

    /// <summary>
    /// 物理演算の世界の時間を進める 
    /// </summary>
    void StepSimulation();

    void DebugDraw();

    void addRigidBody(const bullet::RigidBody& body, std::uint_fast16_t group, std::uint_fast16_t mask);

    void removeRigidBody();

  };
  BulletPhysics getBulletPhysics();

}

#endif
