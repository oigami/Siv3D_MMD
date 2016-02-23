﻿# include <Siv3D.hpp>
#include "include/MMD.h"
#include "include/BulletPhysics.h"
using namespace s3d_mmd;
#include <iostream>

void Main() {
  Window::SetStyle(WindowStyle::Sizeable);
  Graphics::SetBackground(Color(80, 160, 230));
  const MMD model(L"Data/初音ミク/初音ミクVer2.pmd");
  Println(model.name());
  Println(model.comment());
  using namespace s3d_bullet;
  const VMD vmd(L"Data/きしめん.vmd");
  const Mesh meshGround(MeshData::Plane({ 40, 40 }, { 6, 6 }));
  //Bone &bone10 = model.bones()->get(10);
  //bone10.extraBoneControl = true;
  const Font font(30);
  auto bulletPtr = getBulletPhysics();
  while (System::Update()) {
    font(Profiler::FPS(), L"fps").draw();
    vmd.UpdateTime();
    Graphics3D::FreeCamera();
    meshGround.draw();
    //bone10.extraBoneMat *= Quaternion(10_deg, 0, 0, 1).toMatrix();
    model.draw(vmd);
    bulletPtr.StepSimulation();

    bulletPtr.DebugDraw();
  }
}
