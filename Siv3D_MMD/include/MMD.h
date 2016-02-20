#pragma once
//物理演算を使用する場合に定義する
#define USE_BULLET_PHYSICS
#include <memory>
#include "MMDModel.h"
#include "MMDBone.h"
#include "VMDController.h"
#include "../BulletPhysics/BulletPhysics.h"
namespace s3d_mmd {

#ifdef USE_BULLET_PHYSICS

  std::shared_ptr<s3d_bullet::BulletPhysics> getBulletInstance();
#endif // USE_BULLET_PHYSICS


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
