#pragma once
#ifndef PHYSICS3D_INCLUDE_GUARD
#define PHYSICS3D_INCLUDE_GUARD

#include <Siv3D.hpp>
#include <HamFramework.hpp>

# ifdef _WIN64
#  ifdef _DEBUG
#   pragma comment(lib, "x64/bullet/BulletCollision_d")
#   pragma comment(lib, "x64/bullet/BulletDynamics_d")

//  #pragma comment(lib, "x64/bullet/BulletSoftBody_d")
#   pragma comment(lib, "x64/bullet/LinearMath_d")
#  else
#   pragma comment(lib, "x64/bullet/BulletCollision")
#   pragma comment(lib, "x64/bullet/BulletDynamics")

//  #pragma comment(lib, "x64/bullet/BulletSoftBody.lib")
#   pragma comment(lib, "x64/bullet/LinearMath")
#  endif // _DEBUG
# else
#  ifdef _DEBUG
#   pragma comment(lib, "x86/bullet/BulletCollision_d")
#   pragma comment(lib, "x86/bullet/BulletDynamics_d")

//  #pragma comment(lib, "x64/bullet/BulletSoftBody_d")
#   pragma comment(lib, "x86/bullet/LinearMath_d")
#  else
#   pragma comment(lib, "x86/bullet/BulletCollision")
#   pragma comment(lib, "x86/bullet/BulletDynamics")

//  #pragma comment(lib, "x64/bullet/BulletSoftBody.lib")
#   pragma comment(lib, "x86/bullet/LinearMath")
#  endif // _DEBUG
# endif // _WIN64

#include <DirectXMath.h>

//#define BT_NO_SIMD_OPERATOR_OVERLOADS
#ifdef _XM_SSE_INTRINSICS_
#define BT_USE_SSE_IN_API
#endif // _XM_SSE_INTRINSICS_

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>


#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>

#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btIDebugDraw.h>


#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>

namespace physics3d
{
  class DebugDraw : public btIDebugDraw
  {
    int m_debugMode;

  public:

    DebugDraw();

    /// <summary>
    /// 線の描画
    /// </summary>
    /// <param name="from"> 開始点 </param>
    /// <param name="to"> 終了点 </param>
    /// <param name="color"> 色 </param>
    virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);


    /// <summary>
    /// 球の描画
    /// </summary>
    /// <param name="radius"> 半径 </param>
    /// <param name="transform"> 座標 </param>
    /// <param name="color"> 色 </param>
    virtual void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color);

    /// <summary>
    /// 箱の描画
    /// </summary>
    /// <param name="bbMin"> 開始点 </param>
    /// <param name="bbMax"> 終了点 </param>
    /// <param name="trans"> ? </param>
    /// <param name="color"> 色 </param>
    //virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btTransform& trans, const btVector3& color);

    /// <summary>
    /// 箱の描画
    /// </summary>
    /// <param name="bbMin"> 開始点 </param>
    /// <param name="bbMax"> 終了点 </param>
    /// <param name="color"> 色 </param>
    virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btVector3& color);

    /// <summary>
    /// カプセルの描画
    /// </summary>
    /// <param name="radius"> 半径 </param>
    /// <param name="halfHeight"> 高さの半分 </param>
    /// <param name="upAxis"> 上の向き(1 or -1?) </param>
    /// <param name="transform"> 位置と回転 </param>
    /// <param name="color"> 色 </param>
    virtual void drawCapsule(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color);

    /// <summary>
    /// 円柱の描画
    /// </summary>
    /// <param name="radius"> 半径 </param>
    /// <param name="halfHeight"> 高さの半分 </param>
    /// <param name="upAxis"> 上の向き(1 or -1?) </param>
    /// <param name="transform"> 位置と回転 </param>
    /// <param name="color"> 色 </param>
    virtual void drawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color);

    /// <summary>
    /// 衝突点（方向付き）の描画
    /// </summary>
    /// <param name="PointOnB"> 衝突位置 </param>
    /// <param name="normalOnB"> 衝突方向 </param>
    /// <param name="distance"> 衝突の大きさ </param>
    /// <param name="lifeTime"> 表示時間？ </param>
    /// <param name="color"> 色 </param>
    virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);

    /// <summary>
    /// 警告、エラー文の出力
    /// </summary>
    /// <param name="warningString">表示する文字列</param>
    virtual void reportErrorWarning(const char* warningString);

    /// <summary>
    /// 3D文字の描画
    /// </summary>
    /// <param name="location"> 描画位置 </param>
    /// <param name="textString"> 表示する文字列 </param>
    virtual void draw3dText(const btVector3& location, const char* textString);

    /// <summary>
    /// デバッグモードの設定
    /// </summary>
    /// <param name="debugMode"> デバッグモード </param>
    virtual void setDebugMode(int debugMode);

    /// <summary>
    /// デバッグモードの取得
    /// </summary>
    /// <returns> デバッグモード </returns>
    virtual int getDebugMode() const;
  };

  class Physics3DBody;
  class Physics3DShape;

  struct Physics3DMaterial;

  struct Physics3DFilter;

  using Physics3DBodyType = PhysicsBodyType;

  enum class Physics3DShapeType
  {
    Box,
    Sphere,
    Capsule,
  };

  class Physics3D6DofSpringConstraint;
  struct Physics3D6DofSpringConstraintState;

  struct Physics3DMaterial
  {
    //質量
    double mass = 1.0;

    // 反発係数
    double restitution = 0.1;

    // 摩擦係数
    double friction = 0.2;

    // 減衰率(空気抵抗)
    double linearDamping = 0.2, angularDamping = 0.2;

    constexpr Physics3DMaterial(s3d::nullopt_t) {}

    constexpr Physics3DMaterial(double _mass = 1.0, double _restitution = 0.1, double _friction = 0.2, double _linearDamping = 0.2, double _angularDamping = 0.2)
      : mass(_mass)
        , restitution(_restitution)
        , friction(_friction)
        , linearDamping(_linearDamping)
        , angularDamping(_angularDamping) { }
  };

  struct Physics3DFilter
  {
    s3d::uint16 group;
    s3d::uint16 mask;

    Physics3DFilter(s3d::uint16 _group = 0b0000'0000'0000'0001, s3d::uint16 _mask = 0b1111'1111'1111'1111)
      : group(_group), mask(_mask) { }
  };

  class Physics3DWorld
  {
  private:

    class CPhysics3DWorld;

    std::shared_ptr<CPhysics3DWorld> pImpl;

  public:

    Physics3DWorld(const s3d::Vec3& gravity = s3d::Vec3(0.0, -9.8, 0.0));

    std::weak_ptr<btDiscreteDynamicsWorld> getWorldPtr() const;

    /*
    void enableSleep(bool enable);
    bool getSleepEnabled() const;
    */

    void setGravity(const s3d::Vec3& gravity);

    s3d::Vec3 getGravity() const;

    void update(double timeStep = 1.0 / 60) const;

    void debugDraw() const;

    void debugDrawConstraint() const;

    //void shiftOrigin(const s3d::Vec2& newOrigin);

    Physics3DBody createEmpty(const s3d::Vec3& center, const Physics3DMaterial& material = Physics3DMaterial(), const Physics3DFilter& filter = Physics3DFilter(), PhysicsBodyType bodyType = PhysicsBodyType::Dynamic) const;

    Physics3DBody createBox(const s3d::Vec3& center, const s3d::Box& box, const Physics3DMaterial& material = Physics3DMaterial(), const Physics3DFilter& filter = Physics3DFilter(), PhysicsBodyType bodyType = PhysicsBodyType::Dynamic) const;

    Physics3DBody createCapsule(const s3d::Vec3& center, const s3d::Cylinder& capsule, const Physics3DMaterial& material = Physics3DMaterial(), const Physics3DFilter& filter = Physics3DFilter(), PhysicsBodyType bodyType = PhysicsBodyType::Dynamic) const;

    Physics3DBody createSphere(const s3d::Vec3& center, const s3d::Sphere& sphere, const Physics3DMaterial& material = Physics3DMaterial(), const Physics3DFilter& filter = Physics3DFilter(), PhysicsBodyType bodyType = PhysicsBodyType::Dynamic) const;

    Physics3D6DofSpringConstraint create6DofSpringConstraint(const Physics3DBody& bodyA, const Physics3DBody& bodyB, const Physics3D6DofSpringConstraintState& state);
  };

  class Physics3DWorld::CPhysics3DWorld
  {
  private:
    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionConfiguration;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher;
    std::unique_ptr<btDbvtBroadphase> m_overlappingPairCache;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::vector<std::shared_ptr<btTypedConstraint>> m_constraint;
    std::shared_ptr<btDiscreteDynamicsWorld> m_world;

    std::unique_ptr<DebugDraw> m_debugDraw;
  public:

    CPhysics3DWorld(const s3d::Vec3& gravity);

    std::weak_ptr<btDiscreteDynamicsWorld> getWorldPtr() const;
  };

  class Physics3DBody
  {
  private:

    class CPhysics3DBody;

    std::shared_ptr<CPhysics3DBody> pImpl;

    friend class Physics3D6DofSpringConstraint;

  public:

    Physics3DBody();

    Physics3DBody(const Physics3DWorld& world, const s3d::Vec3& center = s3d::Vec3(0, 0, 0), const Physics3DMaterial& material = Physics3DMaterial(), const Physics3DFilter& filter = Physics3DFilter(), PhysicsBodyType bodyType = PhysicsBodyType::Dynamic);

    bool initialized() const;

    Physics3DBody& addBox(const s3d::Box& box);

    Physics3DBody& addCapsule(const s3d::Cylinder& capsule);

    Physics3DBody& addSphere(const s3d::Sphere& sphere);

    void setSleepingAllowed(bool allowed);

    bool isSleepingAllowed() const;

    void setAwake(bool awake);

    bool isAwake() const;

    void setPos(double x, double y, double z);

    void setPos(const s3d::Vec3& pos);

    void moveBy(double x, double y, double z);

    void moveBy(const s3d::Vec3& v);

    void setAngle(const s3d::Quaternion& angle);

    void rotateBy(const s3d::Quaternion& angle);

    void setTransform(double x, double y, double z, const s3d::Quaternion& angle);

    void setTransform(const s3d::Vec3& pos, const s3d::Quaternion& angle);

    void setTransform(const Mat4x4& transform);

    void applyForce(const s3d::Vec3& force);

    void applyForce(const s3d::Vec3& force, const s3d::Vec3& offset);

    void applyForceAt(const s3d::Vec3& force, const s3d::Vec3& pos);

    void applyLinearImpulse(const s3d::Vec3& force);

    void applyLinearImpulse(const s3d::Vec3& force, const s3d::Vec3& offset);

    void applyLinearImpulseAt(const s3d::Vec3& force, const s3d::Vec3& pos);

    void applyTorque(const s3d::Vec3& torque);

    void applyAngularImpulse(const s3d::Vec3& torque);

    s3d::Vec3 getPos() const;

    s3d::Quaternion getAngle() const;

    s3d::Mat4x4 getTransform() const;

    void setVelocity(const s3d::Vec3& v);

    s3d::Vec3 getVelocity() const;

    void setAngularVelocity(const s3d::Vec3& omega);

    s3d::Vec3 getAngularVelocity() const;

    void setDamping(double damping);

    double getDamping() const;

    void setAngularDamping(double damping);

    double getAngularDamping() const;

    //void setGravityScale(double scale);

    //double getGravityScale() const;

    double getMass() const;

    s3d::Vec3 getInertia() const;

    void setBodyType(Physics3DBodyType bodyType);

    Physics3DBodyType getBodyType() const;

    //void setFixedRotation(bool fixedRotation);

    //bool isFixedRotation() const;

    void draw(const s3d::Color& color = s3d::Palette::White) const;

    Physics3DShape& shape(size_t index);

    const Physics3DShape& shape(size_t index) const;

    template<class PShape, std::enable_if_t<std::is_base_of<PhysicsShape, PShape>::value>* = nullptr>
    std::shared_ptr<PShape> shapeAs(const size_t index) const;

    size_t num_shapes() const;
  };

  class Physics3DBody::CPhysics3DBody
  {
  private:
    s3d::Array<std::shared_ptr<Physics3DShape>> m_shapes;

    std::unique_ptr<btMotionState> m_motionState;
    std::unique_ptr<btRigidBody> m_body;
    std::unique_ptr<btCompoundShape> m_compoundShape;

    std::weak_ptr<btDiscreteDynamicsWorld> m_world;

  public:

    CPhysics3DBody(const Physics3DWorld& world, const s3d::Vec3& center, const Physics3DMaterial& material, const Physics3DFilter& filter, Physics3DBodyType bodyType);

    ~CPhysics3DBody();

    void addBox(const s3d::Box& box);

    void addCapsule(const s3d::Cylinder& capsule);

    void addSphere(const s3d::Sphere& sphere);

    btRigidBody* getBody();

    s3d::Array<std::shared_ptr<Physics3DShape>>& getShapes();
  };

  struct Fixture
  {
    btTransform trans;
    btRigidBody* body;
  };

  class Physics3DShape
  {
  protected:

    Fixture m_fixture;

    void updateInertia();

    s3d::Vec3 getPos() const;

    s3d::Quaternion getRot() const;

  public:

    Physics3DShape(btRigidBody* _body, const s3d::Vec3& center, const s3d::Quaternion& rot);

    virtual ~Physics3DShape() = default;

    virtual Physics3DShapeType getShapeType() const = 0;

    virtual void draw(const s3d::Color& color) const = 0;

    /*
    void setDensity(double density);
    double getDensity() const;
    void setFriction(double friction);
    double getFriction() const;
    void setRestitution(double restitution);
    double getRestitution() const;
    void setFilter(const Physics3DFilter& filter);
    Physics3DFilter getFilter() const;
    */
  };

  struct Physics3D6DofSpringConstraintState
  {
    // 結合するローカル座標を指定
    s3d::Mat4x4 frameInA, frameInB;

    // 移動の制限
    s3d::Vec3 linearLowerLimit, linearUpperLimit;

    // 回転の制限(ラジアン)
    s3d::Vec3 angularLowerLimit, angularUpperLimit;

    // ばねの強さ
    s3d::Vec3 stiffnessPos, stiffnessRot;
  };


  class Physics3D6DofSpringConstraint
  {
  private:

    class CPhysics3D6DofSpringConstraint;

    std::shared_ptr<CPhysics3D6DofSpringConstraint> pImpl;

  public:

    Physics3D6DofSpringConstraint();

    Physics3D6DofSpringConstraint(const Physics3DWorld& world, const Physics3DBody& bodyA, const Physics3DBody& bodyB, const Physics3D6DofSpringConstraintState& state);
  };

  class Physics3D6DofSpringConstraint::CPhysics3D6DofSpringConstraint
  {
  private:
    std::unique_ptr<btGeneric6DofSpringConstraint> m_joint;
    std::weak_ptr<btDiscreteDynamicsWorld> m_world;
    std::weak_ptr<Physics3DBody::CPhysics3DBody> m_bodyA, m_bodyB;
  public:

    CPhysics3D6DofSpringConstraint(const Physics3DWorld& world, const Physics3DBody& bodyA, const Physics3DBody& bodyB, const Physics3D6DofSpringConstraintState& state);

    ~CPhysics3D6DofSpringConstraint();
  };


  namespace detail
  {
#ifdef _XM_VECTORCALL_
    using FQuatenion = const s3d::Quaternion;
    using FVec3 = const s3d::Vec3;
#else
    using FQuatenion = const s3d::Quaternion&;
    using FVec3 = const s3d::Vec3&;
#endif // _XM_VECTORCALL_

    inline btQuaternion XM_CALLCONV toBTRot(FQuatenion rot)
    {
#ifdef BT_USE_SSE_IN_API
      constexpr DirectX::XMVECTOR negative{ -1, -1 , 1, 1 };
      return DirectX::XMVectorMultiply(negative, rot.component);
#else
      DirectX::XMFLOAT4A r;
      DirectX::XMStoreFloat4A(&r, rot.component);
      return{ -r.x, -r.y, r.z, r.w };
#endif // BT_USE_SSE_IN_API
    }

    inline s3d::Quaternion toRot(const btQuaternion& rot)
    {
#ifdef BT_USE_SSE_IN_API
      constexpr DirectX::XMVECTOR negative{ -1, -1 , 1, 1 };
      return DirectX::XMVectorMultiply(negative, rot.get128());
#else
      return{ -rot.x(), -rot.y(), rot.z(), rot.w() };
#endif // BT_USE_SSE_IN_API
    }

    /// <summary>
    /// Siv3D形式のベクトルをBullet形式に変換する
    /// <para> zを反転する </para>
    /// </summary>
    /// <param name="vec"></param>
    /// <returns></returns>
    inline btVector3 XM_CALLCONV toBTVec3(FVec3 vec)
    {
      return { static_cast<btScalar>(vec.x), static_cast<btScalar>(vec.y), static_cast<btScalar>(-vec.z) };
    }

    /// <summary>
    /// Bullet形式のベクトルをSiv3D形式に変換する
    /// <para> zを反転する </para>
    /// </summary>
    /// <param name="vec"></param>
    /// <returns></returns>
    inline s3d::Vec3 toVec3(const btVector3& vec)
    {
      return { vec.x(), vec.y(), -vec.z() };
    }

    inline s3d::Vector toVector(const btVector3& vec)
    {
      constexpr DirectX::XMVECTOR negative{ 1, 1 , -1, 1 };
      return DirectX::XMVectorMultiply(negative, vec.get128());
    }

    /// <summary>
    /// Bullet形式のスカラー3つをSiv3D形式に変換する
    /// <para> zを反転しない </para>
    /// </summary>
    /// <param name="scalar3"></param>
    /// <returns></returns>
    inline btVector3 XM_CALLCONV toBTScalar3(FVec3 scalar3)
    {
      return { static_cast<btScalar>(scalar3.x), static_cast<btScalar>(scalar3.y), static_cast<btScalar>(scalar3.z) };
    }

    /// <summary>
    /// Siv3D形式のスカラー3つをBullet形式に変換する
    /// <para> zを反転しない </para>
    /// </summary>
    /// <param name="scalar3"></param>
    /// <returns></returns>
    inline s3d::Vec3 toScalar3(const btVector3& scalar3)
    {
      return { scalar3.x(), scalar3.y(), scalar3.z() };
    }

    inline btTransform XM_CALLCONV toBTMatrix(DirectX::FXMMATRIX m)
    {
      btTransform ret;

#ifdef BT_USE_SSE_IN_API
      auto m2 = DirectX::XMMatrixTranspose(m);
      constexpr DirectX::XMVECTOR negative1{ 1, 1, -1, 0 }, negative2{ -1, -1, 1, 0 };
      const btMatrix3x3 basis(DirectX::XMVectorMultiply(negative1, m2.r[0]),
                              DirectX::XMVectorMultiply(negative1, m2.r[1]),
                              DirectX::XMVectorMultiply(negative2, m2.r[2]));
      ret.setOrigin(DirectX::XMVectorMultiply(negative1, m.r[3]));
#else
      DirectX::XMFLOAT4A r[4];
      for ( auto& i : step(4) )
        DirectX::XMStoreFloat4A(r + i, m.r[i]);
      const btMatrix3x3 basis(
        r[0].x, r[1].x, -r[2].x,
        r[0].y, r[1].y, -r[2].y,
        -r[0].z, -r[1].z, r[2].z);
      ret.setOrigin(btVector3(r[3].x, r[3].y, -r[3].z));
#endif // BT_USE_SSE_IN_API
      ret.setBasis(basis);
      return ret;
    }

    inline s3d::Mat4x4 toMatrix(const btTransform& t)
    {
      s3d::Mat4x4 ret;
      const btMatrix3x3 basis = t.getBasis();
      const btVector3 P = t.getOrigin();

#ifdef BT_USE_SSE_IN_API
      const btVector3 R = basis.getRow(0);
      const btVector3 U = basis.getRow(1);
      const btVector3 L = basis.getRow(2);
      constexpr DirectX::XMVECTOR negative1{ 1, 1, -1, 0 }, negative2{ -1, -1, 1, 0 };
      ret.r[0] = DirectX::XMVectorMultiply(negative1, R.get128());
      ret.r[1] = DirectX::XMVectorMultiply(negative1, U.get128());
      ret.r[2] = DirectX::XMVectorMultiply(negative2, L.get128());
      ret.r[3] = { 0, 0, 0, 1 };
      ret = DirectX::XMMatrixTranspose(ret);
      ret.r[3] = DirectX::XMVectorMultiplyAdd(negative1, P.get128(), ret.r[3]);
#else
      const btVector3 R = basis.getColumn(0);
      const btVector3 U = basis.getColumn(1);
      const btVector3 L = basis.getColumn(2);
      ret.r[0] = DirectX::XMVectorSet(R.x(), R.y(), -R.z(), 0.f);
      ret.r[1] = DirectX::XMVectorSet(U.x(), U.y(), -U.z(), 0.f);
      ret.r[2] = DirectX::XMVectorSet(-L.x(), -L.y(), L.z(), 0.f);
      ret.r[3] = DirectX::XMVectorSet(P.x(), P.y(), -P.z(), 1.f);

#endif // BT_USE_SSE_IN_API


      return ret;
    }

    inline s3d::ColorF toColor(const btVector3& btColor)
    {
      return s3d::ColorF(btColor.x(), btColor.y(), btColor.z());
    }
  }

  class BTBox : public Physics3DShape
  {
    btBoxShape m_shape;
  public:
    BTBox(btRigidBody* body, btCompoundShape* compound, const s3d::Box& box)
      : m_shape(detail::toBTScalar3(box.size) / 2.0), Physics3DShape(body, box.center, box.rotation)
    {
      compound->addChildShape(m_fixture.trans, &m_shape);
      compound->recalculateLocalAabb();
      body->setCollisionShape(compound);

      updateInertia();
    }

    Physics3DShapeType getShapeType() const override
    {
      return Physics3DShapeType::Box;
    }

    void draw(const s3d::Color& color = s3d::Palette::White) const override
    {
      getBox().draw(color);
    }

    s3d::Box getBox() const
    {
      const s3d::Vec3 size = detail::toScalar3(m_shape.getHalfExtentsWithoutMargin() * 2);
      return { getPos(), size, getRot() };
    }
  };

  class BTCapsule : public Physics3DShape
  {
    btCapsuleShape m_shape;

  public:

    BTCapsule(btRigidBody* body, btCompoundShape* compound, const s3d::Cylinder& capsule)
      : m_shape(static_cast<btScalar>(capsule.r), static_cast<btScalar>(capsule.h)), Physics3DShape(body, capsule.center, capsule.rotation)
    {
      compound->addChildShape(m_fixture.trans, &m_shape);
      body->setCollisionShape(compound);
      updateInertia();
    }

    Physics3DShapeType getShapeType() const override
    {
      return Physics3DShapeType::Capsule;
    }

    void draw(const s3d::Color& color = s3d::Palette::White) const override
    {
      getCapsule().draw(color);
    }

    // TODO: カプセルがMeshDataからしか作れないのでとりあえずs3d::Cylinderを使う
    s3d::Cylinder getCapsule() const
    {
      auto pos = getPos();
      return s3d::Cylinder(pos.x, pos.y, pos.z, m_shape.getRadius(), m_shape.getHalfHeight() * 2, getRot());
    }
  };


  class BTSphere : public Physics3DShape
  {
    btSphereShape m_shape;

  public:

    BTSphere(btRigidBody* body, btCompoundShape* compound, const s3d::Sphere& sphere)
      : m_shape(static_cast<btScalar>(sphere.r)), Physics3DShape(body, sphere.center, sphere.rotation)
    {
      compound->addChildShape(m_fixture.trans, &m_shape);
      body->setCollisionShape(compound);
      updateInertia();
    }

    Physics3DShapeType getShapeType() const override
    {
      return Physics3DShapeType::Sphere;
    }

    void draw(const s3d::Color& color = s3d::Palette::White) const override
    {
      getSphere().draw(color);
    }

    s3d::Sphere getSphere() const
    {
      const float r = m_shape.getRadius();
      return { getPos(), r, getRot() };
    }
  };


  //-----------------------------------------------
  //
  //  PhysicsWorld
  //

  inline Physics3DWorld::Physics3DWorld(const s3d::Vec3& gravity)
    : pImpl(std::make_shared<CPhysics3DWorld>(gravity)) { }

  inline std::weak_ptr<btDiscreteDynamicsWorld> Physics3DWorld::getWorldPtr() const
  {
    return pImpl->getWorldPtr();
  }

  inline void Physics3DWorld::setGravity(const s3d::Vec3& gravity)
  {
    getWorldPtr().lock()->setGravity(detail::toBTVec3(gravity));
  }

  inline s3d::Vec3 Physics3DWorld::getGravity() const
  {
    return detail::toVec3(getWorldPtr().lock()->getGravity());
  }

  inline void Physics3DWorld::update(double timeStep) const
  {
    getWorldPtr().lock()->stepSimulation(static_cast<btScalar>(timeStep), 10);
  }

  inline void Physics3DWorld::debugDraw() const
  {
    getWorldPtr().lock()->debugDrawWorld();
  }

  inline void Physics3DWorld::debugDrawConstraint() const
  {
    auto world = getWorldPtr().lock();
    for ( auto& i : step(world->getNumConstraints()) )
    {
      world->debugDrawConstraint(world->getConstraint(i));
    };
  }

  inline Physics3DBody Physics3DWorld::createEmpty(const s3d::Vec3& center, const Physics3DMaterial& material, const Physics3DFilter& filter, PhysicsBodyType bodyType) const
  {
    return Physics3DBody(*this, center, material, filter, bodyType);
  }

  inline Physics3DBody Physics3DWorld::createBox(const s3d::Vec3& center, const s3d::Box& box, const Physics3DMaterial& material, const Physics3DFilter& filter, PhysicsBodyType bodyType) const
  {
    Physics3DBody body(*this, center, material, filter, bodyType);
    body.addBox(box);
    return body;
  }

  inline Physics3DBody Physics3DWorld::createCapsule(const s3d::Vec3& center, const s3d::Cylinder& capsule, const Physics3DMaterial& material, const Physics3DFilter& filter, PhysicsBodyType bodyType) const
  {
    Physics3DBody body(*this, center, material, filter, bodyType);
    body.addCapsule(capsule);
    return body;
  }

  inline Physics3DBody Physics3DWorld::createSphere(const s3d::Vec3& center, const s3d::Sphere& sphere, const Physics3DMaterial& material, const Physics3DFilter& filter, PhysicsBodyType bodyType) const
  {
    Physics3DBody body(*this, center, material, filter, bodyType);
    body.addSphere(sphere);
    return body;
  }

  inline Physics3D6DofSpringConstraint Physics3DWorld::create6DofSpringConstraint(const Physics3DBody& bodyA, const Physics3DBody& bodyB, const Physics3D6DofSpringConstraintState& state)
  {
    return Physics3D6DofSpringConstraint(*this, bodyA, bodyB, state);
  }


  //-----------------------------------------------
  //
  //  CPhysicsWorld
  //

  inline Physics3DWorld::CPhysics3DWorld::CPhysics3DWorld(const s3d::Vec3& gravity)
    : m_collisionConfiguration(new btDefaultCollisionConfiguration()),
      m_dispatcher(new btCollisionDispatcher(m_collisionConfiguration.get())),
      m_overlappingPairCache(new btDbvtBroadphase()),
      m_solver(new btSequentialImpulseConstraintSolver()),
      m_world(new btDiscreteDynamicsWorld(m_dispatcher.get(), m_overlappingPairCache.get(),
                                          m_solver.get(), m_collisionConfiguration.get())),
      m_debugDraw(new DebugDraw())
  {
    m_world->setGravity(detail::toBTVec3(gravity));
    m_world->setDebugDrawer(m_debugDraw.get());
  }

  inline std::weak_ptr<btDiscreteDynamicsWorld> Physics3DWorld::CPhysics3DWorld::getWorldPtr() const
  {
    return m_world;
  }

  //-----------------------------------------------
  //
  //  Physics3DBody
  //

  inline Physics3DBody::Physics3DBody() {}

  inline Physics3DBody::Physics3DBody(const Physics3DWorld& world, const s3d::Vec3& center, const Physics3DMaterial& material, const Physics3DFilter& filter, PhysicsBodyType bodyType)
    : pImpl(std::make_shared<CPhysics3DBody>(world, center, material, filter, bodyType)) { }

  inline bool Physics3DBody::initialized() const
  {
    return !!pImpl;
  }

  inline Physics3DBody& Physics3DBody::addBox(const s3d::Box& box)
  {
    if ( !pImpl )
    {
      return *this;
    }

    pImpl->addBox(box);

    return *this;
  }

  inline Physics3DBody& Physics3DBody::addCapsule(const s3d::Cylinder& capsule)
  {
    if ( !pImpl )
    {
      return *this;
    }

    pImpl->addCapsule(capsule);

    return *this;
  }

  inline Physics3DBody& Physics3DBody::addSphere(const s3d::Sphere& sphere)
  {
    if ( !pImpl )
    {
      return *this;
    }

    pImpl->addSphere(sphere);

    return *this;
  }

  inline void Physics3DBody::setSleepingAllowed(bool allowed)
  {
    if ( !pImpl )
    {
      return;
    }

    if ( allowed )
    {
      pImpl->getBody()->setSleepingThresholds(0.8f, 1.0f);
    }
    else
    {
      pImpl->getBody()->setSleepingThresholds(0.0f, 0.0f);
    }
  }

  inline bool Physics3DBody::isSleepingAllowed() const
  {
    if ( !pImpl )
    {
      return true;
    }

    auto body = pImpl->getBody();
    return body->getLinearSleepingThreshold() != 0.0f || body->getAngularSleepingThreshold() != 0.0f;
  }

  inline void Physics3DBody::setAwake(bool awake)
  {
    if ( !pImpl )
    {
      return;
    }

    if ( awake )
    {
      pImpl->getBody()->activate();
    }
    else
    {
      pImpl->getBody()->wantsSleeping();
    }
  }

  inline bool Physics3DBody::isAwake() const
  {
    if ( !pImpl )
    {
      return true;
    }

    return pImpl->getBody()->isActive();
  }

  inline void Physics3DBody::setPos(double x, double y, double z)
  {
    setPos({ x,y,z });
  }

  inline void Physics3DBody::setPos(const s3d::Vec3& pos)
  {
    if ( !pImpl )
    {
      return;
    }

    auto body = pImpl->getBody();

    body->setWorldTransform(btTransform(body->getWorldTransform().getRotation(), detail::toBTVec3(pos)));
  }

  inline void Physics3DBody::moveBy(double x, double y, double z)
  {
    moveBy({ x,y,z, });
  }

  inline void Physics3DBody::moveBy(const s3d::Vec3& v)
  {
    setPos(getPos() + v);
  }

  inline void Physics3DBody::setAngle(const s3d::Quaternion& angle)
  {
    if ( !pImpl )
    {
      return;
    }

    auto body = pImpl->getBody();

    body->setWorldTransform(btTransform(detail::toBTRot(angle), body->getWorldTransform().getOrigin()));
  }

  inline void Physics3DBody::rotateBy(const s3d::Quaternion& angle)
  {
    setAngle(getAngle() * angle);
  }

  inline void Physics3DBody::setTransform(double x, double y, double z, const s3d::Quaternion& angle)
  {
    setTransform({ x,y,z }, angle);
  }

  inline void Physics3DBody::setTransform(const s3d::Vec3& pos, const s3d::Quaternion& angle)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->setWorldTransform(btTransform(detail::toBTRot(angle), detail::toBTVec3(pos)));
  }

  inline void Physics3DBody::setTransform(const Mat4x4& transform)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->getMotionState()->setWorldTransform(detail::toBTMatrix(transform));
  }

  inline void Physics3DBody::applyForce(const s3d::Vec3& force)
  {
    if ( !pImpl )
    {
      return;
    }

    setAwake(true);
    pImpl->getBody()->applyCentralForce(detail::toBTVec3(force));
  }

  inline void Physics3DBody::applyForce(const s3d::Vec3& force, const s3d::Vec3& offset)
  {
    if ( !pImpl )
    {
      return;
    }

    setAwake(true);
    pImpl->getBody()->applyForce(detail::toBTVec3(force), detail::toBTVec3(offset));
  }

  inline void Physics3DBody::applyForceAt(const s3d::Vec3& force, const s3d::Vec3& pos)
  {
    applyForce(force, pos - getPos());
  }

  inline void Physics3DBody::applyLinearImpulse(const s3d::Vec3& force)
  {
    applyLinearImpulse(force, s3d::Vec3::Zero);
  }

  inline void Physics3DBody::applyLinearImpulse(const s3d::Vec3& force, const s3d::Vec3& offset)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->applyImpulse(detail::toBTVec3(force), detail::toBTVec3(offset));
  }

  inline void Physics3DBody::applyLinearImpulseAt(const s3d::Vec3& force, const s3d::Vec3& pos)
  {
    applyLinearImpulse(force, pos - getPos());
  }

  inline void Physics3DBody::applyTorque(const s3d::Vec3& torque)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->applyTorque(detail::toBTVec3(torque));
  }

  inline void Physics3DBody::applyAngularImpulse(const s3d::Vec3& torque)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->applyTorqueImpulse(detail::toBTVec3(torque));
  }

  inline s3d::Vec3 Physics3DBody::getPos() const
  {
    if ( !pImpl )
    {
      return s3d::Vec3::Zero;
    }

    return detail::toVec3(pImpl->getBody()->getWorldTransform().getOrigin());
  }

  inline s3d::Quaternion Physics3DBody::getAngle() const
  {
    if ( !pImpl )
    {
      return s3d::Quaternion::Identity();
    }

    return detail::toRot(pImpl->getBody()->getWorldTransform().getRotation());
  }

  inline s3d::Mat4x4 Physics3DBody::getTransform() const
  {
    if ( !pImpl )
    {
      return s3d::Mat4x4::Identity();
    }

    btTransform transform;
    pImpl->getBody()->getMotionState()->getWorldTransform(transform);
    return detail::toMatrix(transform);
  }

  inline void Physics3DBody::setVelocity(const s3d::Vec3& v)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->setLinearVelocity(detail::toBTVec3(v));
  }

  inline s3d::Vec3 Physics3DBody::getVelocity() const
  {
    if ( !pImpl )
    {
      return s3d::Vec3::Zero;
    }

    return detail::toVec3(pImpl->getBody()->getLinearVelocity());
  }

  inline void Physics3DBody::setAngularVelocity(const s3d::Vec3& omega)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->setAngularVelocity(detail::toBTVec3(omega));
  }

  inline s3d::Vec3 Physics3DBody::getAngularVelocity() const
  {
    if ( !pImpl )
    {
      return s3d::Vec3::Zero;
    }
    return detail::toVec3(pImpl->getBody()->getAngularVelocity());
  }

  inline void Physics3DBody::setDamping(double damping)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->setDamping(static_cast<btScalar>(damping), static_cast<btScalar>(getAngularDamping()));
  }

  inline double Physics3DBody::getDamping() const
  {
    if ( !pImpl )
    {
      return 0.0;
    }

    return pImpl->getBody()->getLinearDamping();
  }

  inline void Physics3DBody::setAngularDamping(double damping)
  {
    if ( !pImpl )
    {
      return;
    }

    pImpl->getBody()->setDamping(static_cast<btScalar>(getDamping()), static_cast<btScalar>(damping));
  }

  inline double Physics3DBody::getAngularDamping() const
  {
    if ( !pImpl )
    {
      return 0.0;
    }

    return pImpl->getBody()->getAngularDamping();
  }

  inline double Physics3DBody::getMass() const
  {
    if ( !pImpl )
    {
      return 0.0;
    }

    auto res = pImpl->getBody()->getInvMass();

    if ( res == 0.0 ) return res;

    return 1 / res;
  }

  inline s3d::Vec3 Physics3DBody::getInertia() const
  {
    if ( !pImpl )
    {
      return s3d::Vec3::Zero;
    }

    return detail::toVec3(pImpl->getBody()->getLocalInertia());
  }

  inline void Physics3DBody::setBodyType(Physics3DBodyType bodyType)
  {
    if ( !pImpl )
    {
      return;
    }

    auto&& body = pImpl->getBody();
    int flag = body->getCollisionFlags();

    switch ( bodyType )
    {
    case Physics3DBodyType::Dynamic:
      flag &= ~(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_STATIC_OBJECT);
      break;

    case Physics3DBodyType::Static:
      flag &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
      flag |= btCollisionObject::CF_STATIC_OBJECT;

      break;

    case Physics3DBodyType::Kinematic:
      flag &= ~btCollisionObject::CF_STATIC_OBJECT;
      flag |= btCollisionObject::CF_KINEMATIC_OBJECT;
      break;
    }
    body->setCollisionFlags(flag);
  }

  inline Physics3DBodyType Physics3DBody::getBodyType() const
  {
    if ( !pImpl )
    {
      return Physics3DBodyType::Static;
    }

    auto&& body = pImpl->getBody();

    if ( pImpl->getBody()->isKinematicObject() )
    {
      return Physics3DBodyType::Kinematic;
    }

    if ( body->isStaticObject() )
    {
      return Physics3DBodyType::Static;
    }

    return Physics3DBodyType::Dynamic;
  }

  inline void Physics3DBody::draw(const s3d::Color& color) const
  {
    if ( !pImpl )
    {
      return;
    }

    for ( auto& shape : pImpl->getShapes() )
    {
      shape->draw(color);
    }
  }

  inline Physics3DShape& Physics3DBody::shape(size_t index)
  {
    if ( !pImpl )
    {
      throw std::out_of_range("Physics3DBody::shape() Physics3DBody is empty.");
    }

    return *pImpl->getShapes()[index];
  }

  inline const Physics3DShape& Physics3DBody::shape(size_t index) const
  {
    if ( !pImpl )
    {
      throw std::out_of_range("Physics3DBody::shape() Physics3DBody is empty.");
    }

    return *pImpl->getShapes()[index];
  }

  template<class PShape, std::enable_if_t<std::is_base_of<PhysicsShape, PShape>::value>*>
  inline std::shared_ptr<PShape> Physics3DBody::shapeAs(const size_t index) const
  {
    if ( !pImpl )
    {
      throw std::out_of_range("Physics3DBody::shapeAs() Physics3DBody is empty.");
    }

    return std::dynamic_pointer_cast<PShape>(pImpl->getShapes()[index]);
  }

  inline size_t Physics3DBody::num_shapes() const
  {
    if ( !pImpl )
    {
      return 0;
    }

    return pImpl->getShapes().size();
  }

  //-----------------------------------------------
  //
  //  CPhysicsBody
  //

  inline Physics3DBody::CPhysics3DBody::CPhysics3DBody(const Physics3DWorld& world, const s3d::Vec3& center, const Physics3DMaterial& material, const Physics3DFilter& filter, Physics3DBodyType bodyType)
  {
    m_world = world.getWorldPtr();
    m_compoundShape.reset(new btCompoundShape());

    btScalar mass = static_cast<btScalar>(0.0);
    btVector3 localInertia = btVector3(0, 0, 0);
    if ( bodyType == Physics3DBodyType::Dynamic )
    {
      mass = static_cast<btScalar>(material.mass);
      m_compoundShape->calculateLocalInertia(mass, localInertia);
    }

    btTransform transform = btTransform::getIdentity();
    transform.setOrigin(detail::toBTVec3(center));
    m_motionState.reset(new btDefaultMotionState(transform));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, m_motionState.get(),
                                                    m_compoundShape.get(), localInertia);
    m_body.reset(new btRigidBody(rbInfo));
    if ( bodyType == Physics3DBodyType::Kinematic )
    {
      m_body->setCollisionFlags(m_body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
      m_body->setActivationState(DISABLE_DEACTIVATION);
    }
    else if ( bodyType == Physics3DBodyType::Static )
    {
      m_body->setCollisionFlags(m_body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    }
    m_body->setFriction(static_cast<btScalar>(material.friction));
    m_body->setRestitution(static_cast<btScalar>(material.restitution));
    m_body->setDamping(static_cast<btScalar>(material.linearDamping), static_cast<btScalar>(material.angularDamping));
    world.getWorldPtr().lock()->addRigidBody(m_body.get(), filter.group, filter.mask);
  }

  inline Physics3DBody::CPhysics3DBody::~CPhysics3DBody()
  {
    if ( auto world = m_world.lock() )
    {
      world->removeRigidBody(m_body.get());
      for ( auto& i : step_backward(m_body->getNumConstraintRefs()) )
      {
        world->removeConstraint(m_body->getConstraintRef(i));
      };
    }
  }

  inline void Physics3DBody::CPhysics3DBody::addBox(const s3d::Box& box)
  {
    m_shapes.push_back(std::make_shared<BTBox>(m_body.get(), m_compoundShape.get(), box));
  }

  inline void Physics3DBody::CPhysics3DBody::addCapsule(const s3d::Cylinder& capsule)
  {
    m_shapes.push_back(std::make_shared<BTCapsule>(m_body.get(), m_compoundShape.get(), capsule));
  }

  inline void Physics3DBody::CPhysics3DBody::addSphere(const s3d::Sphere& sphere)
  {
    m_shapes.push_back(std::make_shared<BTSphere>(m_body.get(), m_compoundShape.get(), sphere));
  }

  inline btRigidBody* Physics3DBody::CPhysics3DBody::getBody()
  {
    return m_body.get();
  }

  inline s3d::Array<std::shared_ptr<Physics3DShape>>& Physics3DBody::CPhysics3DBody::getShapes()
  {
    return m_shapes;
  }

  //-----------------------------------------------
  //
  //	Physics3DShape
  //

  inline void Physics3DShape::updateInertia()
  {
    btVector3 inertia = btVector3(0, 0, 0);
    auto mass = 1 / m_fixture.body->getInvMass();
    m_fixture.body->getCollisionShape()->calculateLocalInertia(mass, inertia);
    m_fixture.body->setMassProps(mass, inertia);
    m_fixture.body->updateInertiaTensor();
  }

  inline s3d::Vec3 Physics3DShape::getPos() const
  {
    const btTransform& transform = m_fixture.body->getWorldTransform();
    return s3d::ToVec3(DirectX::XMVector3Transform(m_fixture.trans.getOrigin().get128(), detail::toMatrix(transform)));
  }

  inline s3d::Quaternion Physics3DShape::getRot() const
  {
    const btTransform& transform = m_fixture.body->getWorldTransform();
    return detail::toRot(transform.getRotation() * m_fixture.trans.getRotation());
  }

  inline Physics3DShape::Physics3DShape(btRigidBody* _body, const s3d::Vec3& center, const s3d::Quaternion& rot)
  {
    m_fixture.body = _body;
    m_fixture.trans.setOrigin(detail::toBTVec3(center));
    m_fixture.trans.setRotation(detail::toBTRot(rot));
  }

  //-----------------------------------------------
  //
  //	Physics3D6DofSpringConstraint
  //

  inline Physics3D6DofSpringConstraint::Physics3D6DofSpringConstraint() { }

  inline Physics3D6DofSpringConstraint::Physics3D6DofSpringConstraint(const Physics3DWorld& world, const Physics3DBody& bodyA, const Physics3DBody& bodyB, const Physics3D6DofSpringConstraintState& state)
    : pImpl(std::make_shared<CPhysics3D6DofSpringConstraint>(world, bodyA, bodyB, state)) { }

  inline Physics3D6DofSpringConstraint::CPhysics3D6DofSpringConstraint::CPhysics3D6DofSpringConstraint(const Physics3DWorld& world, const Physics3DBody& bodyA, const Physics3DBody& bodyB, const Physics3D6DofSpringConstraintState& state)
  {
    m_world = world.getWorldPtr();

    m_bodyA = bodyA.pImpl;
    m_bodyB = bodyB.pImpl;
    m_joint.reset(new btGeneric6DofSpringConstraint(*bodyA.pImpl->getBody(), *bodyB.pImpl->getBody(),
                                                    detail::toBTMatrix(state.frameInA), detail::toBTMatrix(state.frameInB),
                                                    true));

    m_joint->setLinearLowerLimit(detail::toBTScalar3(state.linearLowerLimit));
    m_joint->setLinearUpperLimit(detail::toBTScalar3(state.linearUpperLimit));
    m_joint->setAngularLowerLimit(detail::toBTScalar3(state.angularLowerLimit));
    m_joint->setAngularUpperLimit(detail::toBTScalar3(state.angularUpperLimit));

    auto set = [&](double val, int i)
      {
        if ( val != 0.0f )
        {
          m_joint->enableSpring(i, true);
          m_joint->setStiffness(i, static_cast<btScalar>(val));
        }
      };
    set(state.stiffnessPos.x, 0);
    set(state.stiffnessPos.y, 1);
    set(state.stiffnessPos.z, 2);
    set(state.stiffnessRot.x, 3);
    set(state.stiffnessRot.y, 4);
    set(state.stiffnessRot.z, 5);
    m_world.lock()->addConstraint(m_joint.get(), true);
  }

  inline Physics3D6DofSpringConstraint::CPhysics3D6DofSpringConstraint::~CPhysics3D6DofSpringConstraint()
  {
    // bodyA も bodyB も生きている時だけここで消す
    // それ以外は bodyA か bodyB が消えるタイミングでCPhysics3DBodyが消す
    if ( auto a = m_bodyA.lock() )
      if ( auto b = m_bodyB.lock() )
        if ( auto world = m_world.lock() )
        {
          world->removeConstraint(m_joint.get());
        }
  }

  template<class F>
  inline void SetRasterizerState(s3d::RasterizerState state, F f)
  {
    const s3d::RasterizerState rasterizerState = s3d::Graphics3D::GetRasterizerState();
    s3d::Graphics3D::SetRasterizerState(state);
    f();
    s3d::Graphics3D::SetRasterizerState(rasterizerState);
  }

  inline DebugDraw::DebugDraw()
    : m_debugMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawConstraints) { }

  inline void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& btColor)
  {
    const s3d::Vec3 vec1 = detail::toVec3(from);
    const s3d::Vec3 vec2 = detail::toVec3(to);
    s3d::ColorF color = detail::toColor(btColor);
    s3d::Line3D(vec1, vec2).drawForward(color);
  }

  inline void DebugDraw::drawSphere(btScalar radius, const btTransform& transform, const btVector3& color)
  {
    const s3d::Vec3 pos = detail::toVec3(transform.getOrigin());
    SetRasterizerState(s3d::RasterizerState::WireframeCullBack, [&]()
                       {
                         s3d::Sphere(pos, radius).draw(detail::toColor(color));
                       });
  }

  inline void DebugDraw::drawBox(const btVector3& bbMin, const btVector3& bbMax, const btVector3& color)
  {
    const s3d::Vec3 from = detail::toVec3(bbMin);
    const s3d::Vec3 to = detail::toVec3(bbMax);
    SetRasterizerState(s3d::RasterizerState::WireframeCullBack, [&]()
                       {
                         s3d::Box(from, to).draw(detail::toColor(color));
                       });
  }

  //void DebugDraw::drawBox(const btVector3 & bbMin, const btVector3 & bbMax, const btTransform& trans, const btVector3 & color) {
  //  const Vec3 from = bullet::ConvertVector(trans.getOrigin() - bbMin);
  //  const Vec3 to = bullet::ConvertVector(bbMax - bbMin);
  //  const auto rot = bullet::ConvertQuaternion(trans.getRotation());
  //  SetRasterizerState(RasterizerState::WireframeCullBack, [&]() {
  //    Box(from, to, rot).draw(bullet::ConvertColor(color));
  //  });
  //}

  inline void DebugDraw::drawCapsule(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color)
  {
    const s3d::Vec3 vec = detail::toVec3(transform.getOrigin());
    const s3d::Quaternion rot = detail::toRot(transform.getRotation() * static_cast<btScalar>(upAxis));
    SetRasterizerState(s3d::RasterizerState::WireframeCullBack, [&]()
                       {
                         //TODO: Capsuleがない
                         s3d::Cylinder(vec, radius, halfHeight * 2, rot).draw(detail::toColor(color));
                       });
  }

  inline void DebugDraw::drawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color)
  {
    const s3d::Vec3 vec = detail::toVec3(transform.getOrigin());
    const s3d::Quaternion rot = detail::toRot(transform.getRotation() * static_cast<btScalar>(upAxis));
    SetRasterizerState(s3d::RasterizerState::WireframeCullBack, [&]()
                       {
                         s3d::Cylinder(vec, radius, halfHeight * 2, rot).draw(detail::toColor(color));
                       });
  }

  inline void DebugDraw::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int, const btVector3& color)
  {
    btVector3 to = distance * (PointOnB + normalOnB);
    drawLine(PointOnB, to, color);
  }

  inline void DebugDraw::reportErrorWarning(const char* warningString)
  {
    s3d::Println(s3d::CharacterSet::Widen(warningString));
  }

  inline void DebugDraw::draw3dText(const btVector3&, const char* textString)
  {
    s3d::Println(s3d::CharacterSet::Widen(textString));
  }

  inline void DebugDraw::setDebugMode(int debugMode)
  {
    m_debugMode = debugMode;
  }

  inline int DebugDraw::getDebugMode() const
  {
    return m_debugMode;
  }
}
#endif // !PHYSICS3D_INCLUDE_GUARD
