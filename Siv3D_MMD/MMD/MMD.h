#pragma once

//物理演算を使用する場合に定義する
//#define USE_BULLET_PHYSICS
#ifndef MAKE_SIV3D_MMD_EXE
#ifdef NDEBUG
#pragma comment(lib, "MMD/lib/Siv3d_MMD")
#else
#pragma comment(lib, "MMD/lib/Siv3d_MMD_d")
#endif // NDEBUG
#endif // MAKE_SIV3D_MMD_EXE

#include <memory>
#include <MMD/mmd_model.h>
#include <MMD/mmd_bone.h>
#include <MMD/mmd_morph.h>
#include <MMD/vmd.h>
#include <MMD/mmd_motion.h>
#include <MMD/physics3d.h>

namespace s3d_mmd
{
  class MMD
  {
    class Pimpl;
    std::shared_ptr<Pimpl> m_handle;

  public:

    MMD() = default;
    MMD(const MMDModel& model, const physics3d::Physics3DWorld& world = physics3d::Physics3DWorld());

    MMD(const FilePath& filename, const physics3d::Physics3DWorld& world = physics3d::Physics3DWorld()) : MMD(MMDModel(filename), world) {}

    ~MMD();


    /// <summary>
    /// ボーンを更新する
    /// <para> 1フレームに一回呼び出す </para>
    /// </summary>
    /// <returns></returns>
    const MMD& update() const;

    /// <summary>
    /// モデルを描画する
    /// </summary>
    /// <param name="worldMat">
    /// 表示するときの座標を指定する。
    /// </param>
    void drawForward(const Mat4x4& worldMat = Mat4x4::Identity()) const;

    void drawForward(double edgeSize, const Mat4x4& worldMat = Mat4x4::Identity()) const;

    void drawEdge(double edgeSize, const Mat4x4& worldmat = Mat4x4::Identity()) const;

    std::shared_ptr<mmd::Bones> bones() const;
    mmd::FaceMorph& morphs() const;
    const String& name() const;
    const String& comment() const;
    const Texture& vertexTexture() const;

    /// <summary>
    /// VMDをセットする
    /// <para> 再生の一時停止等の制御はVMDで行う </para>
    /// </summary>
    /// <param name="vmd"></param>
    void attach(const VMD& vmd) const;

    /// <summary>
    /// VMDを外す
    /// <para> ただし姿勢は元のまま維持される </para>
    /// </summary>
    void dettach() const;

    void release();

    bool isOpened() const;

    explicit operator bool() const
    {
      return isOpened();
    }

  private:

    void PhysicsUpdate() const;
  };
}
