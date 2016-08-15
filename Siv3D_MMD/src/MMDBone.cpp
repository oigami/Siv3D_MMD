#include <Siv3D.hpp>
#include <MMD/MMDBone.h>
namespace s3d_mmd
{
  namespace mmd
  {
    Bone::Bone() : extraBoneControl(false), type(), id(), parent(-1), firstChild(-1), sibling(-1)
    {
      initMat = Mat4x4::Identity();
      offsetMat = Mat4x4::Identity();
      boneMat = Mat4x4::Identity();
      boneMatML = Mat4x4::Identity();
    }

    void Bones::initMatCalc()
    {
      initMatCalc(m_bones.data(), Mat4x4::Identity());
      for ( auto &i : m_bones ) i.boneMat = i.initMat;
    }

    void Bones::initMatCalc(Bone * me, const Matrix & parentoffsetMat)
    {
      if ( me->firstChild != -1 ) initMatCalc(&m_bones[me->firstChild], me->offsetMat);
      if ( me->sibling != -1 ) initMatCalc(&m_bones[me->sibling], parentoffsetMat);
      me->initMat = me->initMatML * parentoffsetMat;
    }

    void Bones::calcWorld(const Bone &me, const Mat4x4 & parentWorldMat, Array<Mat4x4>& worlds) const
    {
      const Mat4x4 m = me.boneMat * parentWorldMat;
      worlds[me.id] = me.offsetMat * m;
      if ( me.firstChild != -1 ) calcWorld(m_bones[me.firstChild], m, worlds);
      if ( me.sibling != -1 ) calcWorld(m_bones[me.sibling], parentWorldMat, worlds);
    }

    Bones::Bones(Array<Bone> bones, Array<mmd::Ik> ikData) :m_bones(std::move(bones)), m_ikData(std::move(ikData))
    {
      for ( auto& i : step(static_cast<int>(m_bones.size())) )
      {
        m_boneNameIndex[m_bones[i].name] = i;
      }
      initMatCalc();
    }

    void Bones::calcWorld(const Mat4x4 &world, Array<Mat4x4> &worlds) const
    {
      const std::uint_fast32_t bonsSize = static_cast<std::uint_fast32_t>(m_bones.size());
      worlds.resize(bonsSize);
      calcWorld(m_bones[0], world, worlds);
      for ( int i : step(bonsSize) )
      {
        if ( m_bones[i].extraBoneControl )
        {
          worlds[i] = m_bones[i].offsetMat * m_bones[i].boneMatML * world;
        }
      }
    }

    // モデルローカル座標系でのボーン行列を計算
    Mat4x4 Bones::calcBoneMatML(int index) const
    {
      Mat4x4 ret = m_bones[index].boneMat;

      for ( int parent = index; parent = m_bones[parent].parent, parent != -1;)
      {
        ret = XMMatrixMultiply(ret, m_bones[parent].boneMat);
      }
      return ret;
      /*auto &bone = m_bones[index];
      if (bone.parent != -1) {
        return XMMatrixMultiply(bone.boneMat, calcBoneMatML(bone.parent));
      } else return bone.boneMat;*/
    }

    Optional<Mat4x4> Bones::calcBoneMatML(const String & boneName) const
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

    Optional<Mat4x4> Bones::calcParentBoneMat(const String & boneName) const
    {
      if ( auto index = getBoneIndex(boneName) )
      {
        return calcParentBoneMat(*index);
      }
      return none;
    }

    Optional<int> Bones::getBoneIndex(const String & boneName) const
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
