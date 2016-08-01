#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd
{
  namespace mmd
  {
    struct FaceMorph
    {

    public:

      FaceMorph() = default;

      FaceMorph(std::unordered_map<String, int>&& faceIndex);

      Optional<int> getFaceIndex(const String& faceName);

      void setWeight(int index, float weight);

      bool setWeight(const String& faceName, float weight);

      const Array<float>& weights()const;

      const std::unordered_map<String, int> faceNameIndex() const;

    private:

      std::unordered_map<String, int> m_faceNum;

      Array<float> m_weight;

    };
  }
}
