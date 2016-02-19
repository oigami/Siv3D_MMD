#pragma once
#include <memory>
#include "MMDModel.h"
#include "MMDBone.h"
#include "VMDController.h"
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
