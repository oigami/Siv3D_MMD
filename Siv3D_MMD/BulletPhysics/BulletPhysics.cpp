/*
bullet�֌W�̃������m�ێ��ɃA���C�����g����������c�����Ă���
�G���[���e���̂̓������A�N�Z�X�ᔽ�Ō������肪���ɂ����̂Œ��ӂ���
//New�̓A���C�����g�ɑΉ����ĂȂ��̂ŃN���b�V������
//�X�^�b�N�ɐςޏꍇ�̃A���C�����g�̂�������킩��Ȃ��̂ŗv����
//make_shared�̓A���C�����g�ɑΉ����Ă�݂��������O�̂��ߎg�킸��
//���ꂼ��̃N���X�ŃI�[�o�[���[�h���ꂽoperator new���g��
*/

#include "BulletPhysics.h"

#include "LinearMath/btDefaultMotionState.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
//#include"bullet/src/btBulletDynamicsCommon.h"
namespace s3d_bullet {
  using namespace bullet;
  BulletPhysics::BulletPhysics(const Vec3& gravity_DX,
    ProcessedCallBack callback)
    :m_collisionConfiguration(new btDefaultCollisionConfiguration()),
    m_dispatcher(new btCollisionDispatcher(m_collisionConfiguration.get())),
    m_overlappingPairCache(new btDbvtBroadphase()),
    m_solver(new btSequentialImpulseConstraintSolver()),
    m_dynamicsWorld(
      new btDiscreteDynamicsWorld(m_dispatcher.get(), m_overlappingPairCache.get(),
        m_solver.get(), m_collisionConfiguration.get())) {
    btVector3 gravity = ConvertVectorDxToBt(gravity_DX);
    //collisionConfiguration = new btDefaultCollisionConfiguration();
    //dispatcher = new btCollisionDispatcher(collisionConfiguration);
    //overlappingPairCache = new btDbvtBroadphase();
    //solver = new btSequentialImpulseConstraintSolver;
    //dynamicsWorld = new btDiscreteDynamicsWorld(&dispatcher,&overlappingPairCache,&solver,&collisionConfiguration);
    m_dynamicsWorld->setGravity(gravity);
    //debugdraw=new DebugDraw();
    //dynamics_world_->setDebugDrawer(&debug_draw_);
    gContactProcessedCallback = callback;
    //mutex_ = CreateMutex(0, FALSE, 0);
  }

  BulletPhysics::~BulletPhysics() {
    for (int i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; --i) {
      btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
      m_dynamicsWorld->removeConstraint(constraint);
      delete constraint;
    }

    for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
      btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState()) delete body->getMotionState();
      m_dynamicsWorld->removeCollisionObject(obj);
      delete obj;
    }

    /*delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete debugdraw;*/

  }

  std::shared_ptr<bullet::Data> BulletPhysics::CreateCompoundShape(
    null_err_unique_ptr<btCollisionShape> box, const Matrix & world, float mass,
    float restitution, float friction, float linear_damp, float angular_damp,
    bool kinematic, unsigned short group, unsigned short mask, const btVector3 & coord) {

    null_err_unique_ptr<btCompoundShape> heroShape(new btCompoundShape());

    btTransform shift;
    shift.setIdentity();
    shift.setOrigin(coord);
    heroShape->addChildShape(shift, box.get());
    //collisionShapes.push_back(box);
    auto bulletdata = CreateShape(move(heroShape), world, mass, restitution, friction, linear_damp, angular_damp, kinematic, group, mask);
    bulletdata->heroshape = move(box);
    return bulletdata;
  }

  std::shared_ptr<bullet::Data>
    BulletPhysics::CreateShape(null_err_unique_ptr<btCollisionShape> shape,
      const Matrix & world, float mass, float restitution,
      float friction, float linear_damp, float angular_damp,
      bool kinematic, unsigned short group, unsigned short mask) {
    auto bulletdata = std::make_shared<bullet::Data>();
    //collisionShapes.push_back(shape);
    btVector3 local_inertia(0, 0, 0);
    if (mass != 0) shape->calculateLocalInertia(mass, local_inertia);
    btTransform btworld = ConvertMatrixDxToBt(world);
    bulletdata->motionState.reset(new btDefaultMotionState(btworld));

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, bulletdata->motionState.get(), shape.get(), local_inertia);
    //m_motionstate.push_back(motionState);
    bulletdata->body.reset(new btRigidBody(rbInfo));
    //btRigidBody* btbody = new btRigidBody(rbInfo);

    bulletdata->body->setRestitution(restitution);
    bulletdata->body->setFriction(friction);
    bulletdata->body->setDamping(linear_damp, angular_damp);
    //float linearDamp = body->getLinearDamping();
    //float angularDamp = body->getAngularDamping();
    if (kinematic) {
      bulletdata->body->setCollisionFlags(bulletdata->body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
      bulletdata->body->setActivationState(DISABLE_DEACTIVATION);
    }
    bulletdata->shape = move(shape);
    /*
    group : 0x8000 �ǂȂǂ̂����蔻��p�̔ԍ�
    */
    //WaitForSingleObject(mutex_, INFINITE);
    m_dynamicsWorld->addRigidBody(bulletdata->body.get(), group, mask);
    //ReleaseMutex(mutex_);

    //unique_ptr<btRigidBody> bodyunique(btbody);
    return bulletdata;
  }





  /// ���̃I�u�W�F�N�g����
  /// ����0, kinematic��false�ɂ���ƁA�����Ȃ�static���̂ɂȂ�B
  /// ����0, kinematic��true�ɂ���ƁA�蓮�œ������邪�A�������Z�̉e�����󂯂Ȃ�Kinematic���̂ɂȂ�

  std::shared_ptr<bullet::Data> BulletPhysics::CreateBox(float width, float height, float depth, const Matrix & world,
    float mass, float restitution, float friction,
    float linear_damp, float angular_damp, bool kinematic,
    unsigned short group, unsigned short mask,
    const btVector3 & coord) {
    btVector3 halfExtents(width / 2, height / 2, depth / 2);
    //btCollisionShape* shape = new btBoxShape(halfExtents);
    if (kinematic) mass = 0;
    null_err_unique_ptr<btBoxShape> temp_box(new btBoxShape(halfExtents));
    return CreateCompoundShape(move(temp_box), world, mass, restitution, friction, linear_damp, angular_damp, kinematic, group, mask, coord);
  }
  std::shared_ptr<bullet::Data> BulletPhysics::CreateSphere(float radius, const Matrix & world,
    float mass, float restitution, float friction, float linear_damp,
    float angular_damp, bool kinematic, unsigned short group, unsigned short mask) {
    if (kinematic) mass = 0;
    null_err_unique_ptr<btSphereShape> shape(new btSphereShape(radius));
    return CreateShape(move(shape), world, mass, restitution, friction, linear_damp, angular_damp, kinematic, group, mask);
  }

  std::shared_ptr<bullet::Data> BulletPhysics::CreateCylinder(float radius, float length, const Matrix & world,
    float mass, float restitution, float friction, float linear_damp,
    float angular_damp, bool kinematic, unsigned short group, unsigned short mask) {
    btVector3 halfExtents(radius, radius, length / 2);
    if (kinematic) mass = 0;
    null_err_unique_ptr<btCylinderShape> shape(new btCylinderShape(halfExtents));
    return CreateShape(move(shape), world, mass, restitution, friction, linear_damp, angular_damp, kinematic, group, mask);
  }

  std::shared_ptr<bullet::Data> BulletPhysics::CreateCapsule(float radius, float height, const Matrix & world,
    float mass, float restitution, float friction, float linear_damp,
    float angular_damp, bool kinematic, unsigned short group, unsigned short mask, const btVector3 & coord) {
    if (kinematic) mass = 0;
    null_err_unique_ptr<btCapsuleShape> shape(new btCapsuleShape(radius, height));
    return CreateCompoundShape(move(shape), world, mass, restitution, friction, linear_damp, angular_damp, kinematic, group, mask, coord);
  }

  void BulletPhysics::AddPointToPointConstraint(std::shared_ptr<bullet::Data>  body, const Vec3& pivotDX) {
    btVector3 pivot = ConvertVectorDxToBt(pivotDX);
    std::shared_ptr<btPoint2PointConstraint> constraint(new btPoint2PointConstraint(*body->body, pivot));
    body->Constraintnum.push_back(constraint);
    m_constraint.push_back(constraint);
    m_dynamicsWorld->addConstraint(constraint.get());
  }

  void BulletPhysics::AddPointToPointConstraint(std::shared_ptr<bullet::Data> bodyA,
    std::shared_ptr<bullet::Data> bodyB,
    const Vec3& pivotInADX, const Vec3& pivotInBDX) {
    btVector3 pivotInA = ConvertVectorDxToBt(pivotInADX);
    btVector3 pivotInB = ConvertVectorDxToBt(pivotInBDX);
    std::shared_ptr<btPoint2PointConstraint> constraint(
      new btPoint2PointConstraint(*bodyA->body, *bodyB->body, pivotInA, pivotInB));

    bodyA->Constraintnum.push_back(constraint);
    bodyB->Constraintnum.push_back(constraint);

    m_constraint.push_back(constraint);
    m_dynamicsWorld->addConstraint(constraint.get());
  }






  /// 6���W���C���g��ǉ�
  /// @param bodyA ����A
  /// @param bodyB ����B
  /// @param frameInA �W���C���g�̃��[���h�ϊ��s��(����A���[�J�����W�n)
  /// @param frameInB �W���C���g�̃��[���h�ϊ��s��(����B���[�J�����W�n)
  /// @param c_p1 �ړ�����1
  /// @param c_p2 �ړ�����2
  /// @param c_r1 ��]����1
  /// @param c_r2 ��]����2
  /// @param stiffness �o�l����(���s�ړ�x, y, z, ��]�ړ�x, y, z�̏���6�v�f)
  /// 6���W���C���g��ǉ�
  /// @param bodyA ����A
  /// @param bodyB ����B
  /// @param frameInA �W���C���g�̃��[���h�ϊ��s��(����A���[�J�����W�n)
  /// @param frameInB �W���C���g�̃��[���h�ϊ��s��(����B���[�J�����W�n)
  /// @param c_p1 �ړ�����1
  /// @param c_p2 �ړ�����2
  /// @param c_r1 ��]����1
  /// @param c_r2 ��]����2
  /// @param stiffnessPos �o�l����(���s�ړ�x, y, z ��3�v�f)
  /// @param stiffnessRot �o�l����(��]�ړ�x, y, z ��3�v�f)

  void BulletPhysics::Add6DofSpringConstraint(std::shared_ptr<bullet::Data> bodyA, std::shared_ptr<bullet::Data> bodyB,
    const btTransform & frameInA, const btTransform & frameInB,
    const Float3 & c_p1, const Float3 & c_p2, const Float3 & c_r1, const Float3 & c_r2,
    const Float3 & stiffnessPos, const Float3 & stiffnessRot) {
    // ��܈����̌��ʂ͓�B�ǂ���ł������l�Ɍ�����c�c�B
    //auto constraint = make_shared<btGeneric6DofSpringConstraint>(*m_bulletdata[bodyA]->body, *m_bulletdata[bodyB]->body, frameInA, frameInB, false);
    std::shared_ptr<btGeneric6DofSpringConstraint> constraint(
      new btGeneric6DofSpringConstraint(*bodyA->body, *bodyB->body, frameInA, frameInB, false));
    bodyA->Constraintnum.push_back(constraint);
    bodyB->Constraintnum.push_back(constraint);
    m_constraint.push_back(constraint);
    constraint->setLinearLowerLimit(btVector3(c_p1.x, c_p1.y, c_p1.z));	// �^�̓x�N�g�������x�N�g���ʂł͂Ȃ��̂�Z�͔��]���Ȃ��B
    constraint->setLinearUpperLimit(btVector3(c_p2.x, c_p2.y, c_p2.z));
    constraint->setAngularLowerLimit(btVector3(c_r1.x, c_r1.y, c_r1.z));
    constraint->setAngularUpperLimit(btVector3(c_r2.x, c_r2.y, c_r2.z));
    auto set = [&](float val, int i) {
      if (val != 0.0f) {
        constraint->enableSpring(i, true);
        constraint->setStiffness(i, val);
      }
    };
    set(stiffnessPos.x, 0);
    set(stiffnessPos.y, 1);
    set(stiffnessPos.z, 2);
    set(stiffnessRot.x, 3);
    set(stiffnessRot.y, 4);
    set(stiffnessRot.z, 5);
    m_dynamicsWorld->addConstraint(constraint.get());
  }

  void BulletPhysics::StepSimulation() {
    m_dynamicsWorld->stepSimulation(1.0f / Profiler::FPS(), 100);
  }
  void BulletPhysics::DebugDraw() {
    m_dynamicsWorld->debugDrawWorld();
  }

}
