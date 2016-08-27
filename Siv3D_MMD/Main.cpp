#ifndef LIB_COMPILE
# include <Siv3D.hpp>
#include "MMD/MMD.h"
#include "MMD/BulletPhysics.h"
using namespace s3d_mmd;
#include <iostream>
#include <MMD/mmd_motion.h>

void Main()
{
  Window::SetStyle(WindowStyle::Sizeable);
  Graphics::SetBackground(Color(80, 160, 230));
  Graphics3D::SetAmbientLight(ColorF(0.3));
  const MMD model(L"Data/初音ミク/初音ミクVer2.pmd");
  Println(model.name());
  Println(model.comment());
  using namespace s3d_bullet;
  const Mesh meshGround(MeshData::Plane({ 40, 40 }, { 6, 6 }));

  mmd::MMDMotion motion(VMDReader(L"Data/きしめん.vmd"));
  motion.saveVMD(L"Data/きしめん.vmd.sav");
  const VMD vmd(L"Data/きしめん.vmd.sav");
  vmd.play();
  model.attach(vmd);
  //Bone &bone10 = model.bones()->get(10);
  //bone10.extraBoneControl = true;
  const Font font(30);
  auto bulletPtr = getBulletPhysics();
  /*GUI gui(GUIStyle::Default);
  gui.setTitle(L"タイトル");

  gui.add(L"frame", GUISlider::Create(0, 1000, 0));*/
  Camera camera;
  while ( System::Update() )
  {
    font(Profiler::FPS(), L"fps").draw();

    //vmd.setTime(gui.slider(L"frame").valueInt);
    //if (Input::KeyN.pressed)
    meshGround.draw();

    //bone10.extraBoneMat *= Quaternion(10_deg, 0, 0, 1).toMatrix();
    model.update().draw();
    auto mat = *model.bones()->calcBoneMatML(L"頭");
    camera.lookat = mat.transform(Vec3(0, 0, 0));
    camera.pos = mat.transform(Vec3(0, 0, -10));

    Graphics3D::SetCamera(camera);
    //if (Input::KeyB.pressed)
    bulletPtr.StepSimulation();

    //bulletPtr.DebugDraw();
  }
}
#endif
