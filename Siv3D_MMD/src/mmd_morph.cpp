#include "stdafx/stdafx.h"
#include <MMD/mmd_morph.h>

namespace s3d_mmd
{
  namespace mmd
  {
    FaceMorph::FaceMorph(std::unordered_map<String, int>&& faceIndex)
      : m_faceNum(std::move(faceIndex)), m_weight(m_faceNum.size()) { }

    Optional<int> FaceMorph::getFaceIndex(const String& faceName)
    {
      auto it = m_faceNum.find(faceName);
      if ( it == m_faceNum.end() ) return none;
      return it->second;
    }

    void FaceMorph::setWeight(int index, float weight)
    {
      m_weight[index] = weight;
    }

    bool FaceMorph::setWeight(const String& faceName, float weight)
    {
      if ( auto i = getFaceIndex(faceName) )
      {
        m_weight[*i] = weight;
        return true;
      }
      return false;
    }

    const Array<float>& FaceMorph::weights() const
    {
      return m_weight;
    }

    const std::unordered_map<String, int> FaceMorph::faceNameIndex() const
    {
      return m_faceNum;
    }
  }
}
