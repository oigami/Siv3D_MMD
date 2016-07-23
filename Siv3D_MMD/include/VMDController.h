#pragma once
#include <Siv3D.hpp>
#include "MMDBone.h"
#include "VMDReader.h"
namespace s3d_mmd {

  class VMD {
    class Pimpl;
    std::shared_ptr<Pimpl> m_handle;

  public:

    VMD(const FilePath &filename);
    VMD();
    ~VMD();

    void UpdateBone(mmd::Bones &bones) const;
    void UpdateTime() const;
    void setTime(int frameCount) const;

    void IsLoop(bool loop, int startTime)const;
  };
}
