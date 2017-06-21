#pragma once
#ifndef INCLUDE_IMMD_PHYCSIS_H
#define INCLUDE_IMMD_PHYCSIS_H

#include <MMD/mmd_bone.h>
#include <MMD/pmd_struct.h>

namespace s3d_mmd
{
  class IMMDPhysics
  {
  public:

    virtual void create(std::shared_ptr<mmd::Bones> bones,
                        const std::vector<pmd_struct::RigidBody>& pmdRigidBodies,
                        const std::vector<pmd_struct::Joint>& pmdJoints) = 0;
    virtual void boneUpdate(const Mat4x4& world) = 0;

  protected:
    virtual ~IMMDPhysics() = default;
  };

  class IMMDPhysicsWorld
  {
  public:

    virtual std::shared_ptr<IMMDPhysics> create() = 0;

    virtual void update() = 0;

  protected:
    virtual ~IMMDPhysicsWorld() = default;
  };

}
#endif // !INCLUDE_IMMD_PHYCSIS_H
