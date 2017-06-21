#include <Siv3D.hpp>
#include <MMD/mmd_bone.h>
#include "MMD/math_util.h"

namespace s3d_mmd
{
  namespace mmd
  {
    Bone::Bone() : extraBoneControl(false), type(), id(), parent(-1), firstChild(-1), sibling(-1)
    {
      initMatBoneLocal = DirectX::XMVectorZero();
      offsetMat = DirectX::XMVectorZero();
      bonePosition = DirectX::XMVectorZero();
      boneMatML = Mat4x4::Identity();
    }

    void Bones::initMatCalc()
    {
      initMatCalc(m_bones.data(), DirectX::XMVectorZero());
      for ( auto& i : m_bones ) i.bonePosition = i.initMatBoneLocal;
    }

    void Bones::initMatCalc(Bone* me, Vector parentoffsetMat)
    {
      if ( me->firstChild != -1 ) initMatCalc(&m_bones[me->firstChild], me->offsetMat);
      if ( me->sibling != -1 ) initMatCalc(&m_bones[me->sibling], parentoffsetMat);
      using namespace DirectX;
      me->initMatBoneLocal = me->initMatML + parentoffsetMat;
    }


    void Bones::calcWorld(const Bone& me, const Mat4x4& parentWorldMat, Array<Mat4x4>& worlds) const
    {
      auto boneMat = me.boneRotation.toMatrix();
      boneMat.r[3] = me.bonePosition;
      const Mat4x4 m = boneMat * parentWorldMat;
      worlds[me.id] = math::Mul(me.offsetMat, m);
      if ( me.firstChild != -1 ) calcWorld(m_bones[me.firstChild], m, worlds);
      if ( me.sibling != -1 ) calcWorld(m_bones[me.sibling], parentWorldMat, worlds);
    }

    Bones::Bones(Array<Bone> bones, Array<mmd::Ik> ikData) : m_bones(std::move(bones)), m_ikData(std::move(ikData))
    {
      for ( auto& i : step(static_cast<int>(m_bones.size())) )
      {
        m_boneNameIndex[m_bones[i].name] = i;
      }
      initMatCalc();
    }

    Array<Mat4x4>& Bones::calcWorld(const Mat4x4& world)
    {
      const std::uint_fast32_t bonsSize = static_cast<std::uint_fast32_t>(m_bones.size());
      m_lastUpdatedWorlds.resize(bonsSize);
      calcWorld(m_bones[0], world, m_lastUpdatedWorlds);
      for ( int i : step(bonsSize) )
      {
        if ( m_bones[i].extraBoneControl )
        {
          m_lastUpdatedWorlds[i] = math::Mul(m_bones[i].offsetMat, m_bones[i].boneMatML) * world;
        }
      }
      return m_lastUpdatedWorlds;
    }

    // モデルローカル座標系でのボーン行列を計算
    Mat4x4 Bones::calcBoneMatML(int index) const
    {
      Quaternion q = m_bones[index].boneRotation;
      Vector pos = m_bones[index].bonePosition;
      for ( int parent = index; parent = m_bones[parent].parent, parent != -1; )
      {
        q = q * m_bones[parent].boneRotation;
        using DirectX::operator+;
        pos = DirectX::XMVector3Rotate(pos, m_bones[parent].boneRotation.component) + m_bones[parent].bonePosition;
      }
      auto tmp = q.toMatrix();
      tmp.r[3] = pos;
      return tmp;
    }

    Vector Bones::calcBonePositionML(int index) const
    {
      Vector ret = m_bones[index].bonePosition;

      for ( int parent = index; parent = m_bones[parent].parent, parent != -1; )
      {
        using DirectX::operator+=;
        ret = DirectX::XMVector3Rotate(ret, m_bones[parent].boneRotation.component);
        ret += m_bones[parent].bonePosition;
      }
      return ret;
    }

    Optional<Mat4x4> Bones::calcBoneMatML(const String& boneName) const
    {
      if ( auto index = getBoneIndex(boneName) )
      {
        return calcBoneMatML(*index);
      }
      return none;;
    }

    Mat4x4 Bones::calcParentBoneMat(int index) const
    {
      if ( m_bones[index].parent == -1 ) return Mat4x4::Identity();
      return calcBoneMatML(m_bones[index].parent);
    }

    Optional<Mat4x4> Bones::calcParentBoneMat(const String& boneName) const
    {
      if ( auto index = getBoneIndex(boneName) )
      {
        return calcParentBoneMat(*index);
      }
      return none;
    }

    Optional<int> Bones::getBoneIndex(const String& boneName) const
    {
      auto it = m_boneNameIndex.find(boneName);
      if ( it != m_boneNameIndex.end() )
      {
        return it->second;
      }
      return none;
    }
  }
}
