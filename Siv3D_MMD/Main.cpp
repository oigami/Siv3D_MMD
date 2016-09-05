#ifndef LIB_COMPILE
# include <Siv3D.hpp>
#include "MMD/MMD.h"
using namespace s3d_mmd;
#include <iostream>
#include <MMD/mmd_motion.h>

#include <MMD/mouse_camera3d.h>
void Main()
{
  Window::SetStyle(WindowStyle::Sizeable);
  Graphics::SetBackground(Color(80, 160, 230));
  Graphics3D::SetAmbientLight(ColorF(0.3));
  physics3d::Physics3DWorld world;
  const MMD model(L"Data/初音ミク/初音ミクVer2.pmd", world);
  Println(model.name());
  Println(model.comment());
  const Mesh meshGround(MeshData::Plane({ 40, 40 }, { 6, 6 }));

  mmd::MMDMotion motion(VMDReader(L"Data/きしめん.vmd"));
  motion.saveVMD(L"Data/きしめん.vmd.sav");
  const VMD vmd(L"Data/gokuraku.vmd");

  vmd.play();
  model.attach(vmd);
  //Bone &bone10 = model.bones()->get(10);
  //bone10.extraBoneControl = true;
  const Font font(30);
  /*GUI gui(GUIStyle::Default);
  gui.setTitle(L"タイトル");

  gui.add(L"frame", GUISlider::Create(0, 1000, 0));*/
  MouseCamera3D camera;
  while ( System::Update() )
  {
    world.update();
    //world.debugDraw();
    font(Profiler::FPS(), L"fps").draw();
    //vmd.setTime(gui.slider(L"frame").valueInt);
    //if (Input::KeyN.pressed)
    meshGround.draw();

    //bone10.extraBoneMat *= Quaternion(10_deg, 0, 0, 1).toMatrix();
    model.update().draw();
    auto mat = *model.bones()->calcBoneMatML(L"頭");
    //camera.lookat = mat.transform(Vec3(0, 0, 0));
    //camera.pos = mat.transform(Vec3(0, 0, -10));
    camera.update();
    camera.setCamera();

    //Graphics3D::FreeCamera();
    //Graphics3D::SetCamera(camera);
    //if (Input::KeyB.pressed)

    //bulletPtr.DebugDraw();
  }
}
#endif
