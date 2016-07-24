#pragma once

//物理演算を使用する場合に定義する
//#define USE_BULLET_PHYSICS
#ifndef MAKE_SIV3D_MMD_EXE
#ifdef NDEBUG
#pragma comment(lib, "Siv3d_MMD")
#else
#pragma comment(lib, "Siv3d_MMD_d")
#endif // NDEBUG
#endif // MAKE_SIV3D_MMD_EXE

#include <memory>
#include "MMDModel.h"
#include "MMDBone.h"
#include "VMDController.h"
#include "BulletPhysics.h"
namespace s3d_mmd {

  class MMD {
    class Pimpl;
    std::shared_ptr<Pimpl> m_handle;

  public:

    MMD() = default;
    MMD(const MMDModel &model);
    MMD(const FilePath &filename) :MMD(MMDModel(filename)) {}
    ~MMD();


    void draw() const;
    void draw(const VMD &vmd) const;
    void draw(double edgeSize) const;
    void drawEdge(double edgeSize) const;

    std::shared_ptr<mmd::Bones> bones() const;
    const String &name() const;
    const String &comment() const;
    const Texture &vertexTexture() const;

    bool isOpen() const;
  private:

    void PhysicsUpdate(Array<Mat4x4> &boneWorld) const;

  };

  class DrawableVMD {
    friend class MMD;
    const MMD m_mmd;
    const VMD m_vmd;

  public:

    DrawableVMD(const MMD &mmd, const VMD &vmd);
    void draw() const;

  };

}
