#include <Siv3D.hpp>
#include "../include/MMDBone.h"
namespace s3d_mmd {
  namespace mmd {
    Bone::Bone() : extraBoneControl(false), type(), id(), parent(-1), firstChild(-1), sibling(-1) {
      initMat = Mat4x4::Identity();
      offsetMat = Mat4x4::Identity();
      boneMat = Mat4x4::Identity();
      boneMatML = Mat4x4::Identity();
    }

    void Bones::InitMatCalc() {
      InitMatCalc(m_bones.data(), Mat4x4::Identity());
      for (auto &i : m_bones) i.boneMat = i.initMat;
    }

    void Bones::InitMatCalc(Bone * me, const Matrix & parentoffsetMat) {
      if (me->firstChild != -1) InitMatCalc(&m_bones[me->firstChild], me->offsetMat);
      if (me->sibling != -1) InitMatCalc(&m_bones[me->sibling], parentoffsetMat);
      me->initMat = XMMatrixMultiply(me->initMatML, parentoffsetMat);
    }

    void Bones::CalcWorld(const Bone &me, const Mat4x4 & parentWorldMat, Array<Mat4x4>& worlds) const {
      const DirectX::XMMATRIX m = XMMatrixMultiply(me.boneMat, parentWorldMat);
      worlds[me.id] = XMMatrixMultiply(me.offsetMat, m);
      if (me.firstChild != -1) CalcWorld(m_bones[me.firstChild], m, worlds);
      if (me.sibling != -1) CalcWorld(m_bones[me.sibling], parentWorldMat, worlds);
    }

    void Bones::CalcWorld(const Mat4x4 &world, Array<Mat4x4> &worlds) const {
      const std::uint_fast32_t bonsSize = static_cast<std::uint_fast32_t>(m_bones.size());
      worlds.resize(bonsSize);
      CalcWorld(m_bones[0], world, worlds);
      for (int i : step(bonsSize)) {
        if (m_bones[i].extraBoneControl) {
          const DirectX::XMMATRIX temp = XMMatrixMultiply(m_bones[i].offsetMat, m_bones[i].boneMatML);
          worlds[i] = XMMatrixMultiply(temp, world);
        }
      }
    }

    // モデルローカル座標系でのボーン行列を計算
    Mat4x4 Bones::CalcBoneMatML(int index) const {
      Mat4x4 ret = m_bones[index].boneMat;
      for (int parent = index; parent = m_bones[parent].parent, parent != -1;) {
        ret *= m_bones[parent].boneMat;
      }
      return ret;
      /*auto &bone = m_bones[index];
      if (bone.parent != -1) {
        return XMMatrixMultiply(bone.boneMat, CalcBoneMatML(bone.parent));
      } else return bone.boneMat;*/
    }

  }

}
