#pragma once
namespace s3d_mmd
{
  namespace mmd
  {
    struct FaceMorph
    {
      std::unordered_map<String, int> faceNum;
      Array<float> weight;
    };
  }
}
