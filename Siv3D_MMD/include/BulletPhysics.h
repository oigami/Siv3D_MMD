#pragma once
#ifndef INCLUDE_BULLET_PHYSICS_H
#define INCLUDE_BULLET_PHYSICS_H

#include <Siv3D.hpp>

namespace s3d_bullet
{
  class BulletPhysics;

  namespace bullet
  {
    class RigidBody;
    namespace detail
    {
      class Data;
    }

  }

  class BulletPhysics final
  {

    class Pimpl;

    std::shared_ptr<Pimpl> m_pimpl;

  public:

    BulletPhysics(const Float3& gravity);
    BulletPhysics() = default;
    ~BulletPhysics() = default;


    //bullet::Data CreateCylinder(float radius, float length, const Mat4x4 &world, // 中心軸はZ軸
    //  float mass = 0, float restitution = 0, float friction = 0.5f,
    //  float linear_damp = 0, float angular_damp = 0, bool kinematic = false,
    //  unsigned short group = 1, unsigned short mask = 0xFFFF);

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
    void Add6DofSpringConstraint(bullet::RigidBody& bodyA, bullet::RigidBody& bodyB,
                                 const Mat4x4& frameInA, const Mat4x4& frameInB,
                                 const std::array<float, 3>& c_p1, const std::array<float, 3>& c_p2,
                                 const std::array<float, 3>& c_r1, const std::array<float, 3>& c_r2,
                                 const Float3 &stiffnessPos, const Float3 &stiffnessRot);

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
