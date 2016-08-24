#pragma once
#include <MMD/vmd_struct.h>
namespace s3d_mmd
{
  namespace mmd
  {
    class VMDWriter
    {
      BinaryWriter writer_;

    public:

      String modelName;
      Array<vmd_struct::Bone> boneFrame;
      Array<vmd_struct::Morph> morphFrame;
      Array<vmd_struct::Camera> cameraFrame;
      Array<vmd_struct::Light> lightFrame;
      Array<vmd_struct::SelfShadow> selfShadowFrame;
      Array<vmd_struct::ShowIk> showIkFrame;


      VMDWriter() = default;
      VMDWriter(const FilePath& outFileName);

      ~VMDWriter();

      bool isOpened();

      void close();
    };
  }
}
