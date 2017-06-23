#include "src/mmd_physics.h"
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
  MMD::SetDefaultPhysicsWorld(std::make_shared<MMDPhysicsWorld>(world));
  const MMD model(L"Data/初音ミク/初音ミクVer2.pmd");
  Println(model.name());
  Println(model.comment());
  const Mesh meshGround(MeshData::Plane({ 40, 40 }, { 6, 6 }));

  mmd::MMDMotion motion(VMDReader(L"Data/きしめん.vmd"));
  mmd::MMDMotion output = motion;
  motion.saveVMD(L"Data/きしめん.vmd.sav");
  const VMD vmd(L"Data/きしめん.vmd");

  //vmd.play();
  model.attach(vmd);
  //Bone &bone10 = model.bones()->get(10);
  //bone10.extraBoneControl = true;
  const Font font(30);
  GUI gui(GUIStyle::Default);
  gui.setTitle(L"タイトル");

  gui.add(L"frame", GUISlider::Create(0, 10, 0));
  MouseCamera3D camera;
  System::Update();
  int nowFrame = 0;
  int lastFrame = motion.getLastFrame();
  std::array<String, 18> lists = {
    // 鼻
    L"右目",
    L"首",
    L"右腕", L"右ひじ", L"右手首",
    L"左腕", L"左ひじ", L"左手首",
    L"右足", L"右ひざ", L"右足首",
    L"左足", L"左ひざ", L"左足首",
    L"右目", L"左目",
    // 右耳, 左耳
    L"右目", L"左目"
  };
  std::array<int, 18> lists_int;
  for ( auto& i : step(18) )
  {
    lists_int[i] = *model.bones()->getBoneIndex(lists[i]);
  }
  std::array<std::vector<int>, 18> linkList = {
    std::vector<int>{ 1,15,14 },
    { 2,5,8,11 },
    { 3 },
    { 4 },
    {},
    { 6 },
    { 7 },
    {},
    { 9 },
    { 10 },
    {},
    { 12 },
    { 13 },
    {},
    { 16 },
    { 17 },
    {},
    {}
  };
  std::array<Vec3, 18> diff = {
    Vec3(0.3, -0.9, -0.8),
    Vec3(0, -0.8, 0),
    Vec3(), Vec3(), Vec3(),
    Vec3(), Vec3(), Vec3(),
    Vec3(), Vec3(), Vec3(),
    Vec3(), Vec3(), Vec3(),
    Vec3(0,-0.6,-0.6), Vec3(0,-0.6,-0.6),
    Vec3(-0.3, -0.6, 0), Vec3(0.3, -0.6, 0),
  };

  Camera camerax;
  camerax.pos = Vec3(0, 10, -40);
  camerax.lookat = Vec3(0, 10, 0);

  while ( System::Update() )
  {
    vmd.setPosFrame(nowFrame++);
    //world.update();

    //world.debugDraw();
    font(Profiler::FPS(), L"fps").draw();
    //vmd.setTime(gui.slider(L"frame").valueInt);
    //if (Input::KeyN.pressed)
    meshGround.draw();

    //bone10.extraBoneMat *= Quaternion(10_deg, 0, 0, 1).toMatrix();
    model.update();
    auto& localMats = model.bones()->lastUpdatedModelLocal();

    std::array<Mat4x4, 18> worlds;
    for ( auto& i : step(18) )
    {
      worlds[i] = Mat4x4::Translate(diff[i]) * localMats[lists_int[i]];
    }
    for ( auto& i : step(18) )
    {
      auto v = ToVec3(worlds[i].r[3]);
      //Sphere(v, 0.1).draw();

      for ( auto& j : linkList[i] )
      {
        auto v2 = ToVec3(worlds[j].r[3]);
        Line3D(v, v2).drawForward(Palette::Yellow);
      }
    }
    model.drawForward();
    font(gui.slider(L"frame").value).draw(0, 50);
    //auto mat = *model.bones()->calcBoneMatML(L"頭");
    //camera.lookat = mat.transform(Vec3(0, 0, 0));
    //camera.pos = mat.transform(Vec3(0, 0, -10));
    std::array<Vec2, 18> tmp;
    for ( auto& i : step(18) )
    {
      auto v = worlds[i].r[3];
      tmp[i] = ToVec3(DirectX::XMVector3TransformCoord(v, camerax.calcViewProjectionMatrix())).xy();
      Vec2 halfSize(Window::Width() / 2, Window::Height() / 2);
      tmp[i] = tmp[i] * halfSize;

      tmp[i].x += halfSize.x;
      tmp[i].y *= -1;
      tmp[i].y += halfSize.y;
      Circle(tmp[i], 5).draw(Palette::Red);
    }

    camera.update();
    camera.setCamera();
    Graphics3D::SetCamera(camerax);
    //Graphics3D::FreeCamera();
    //Graphics3D::SetCamera(camera);
    //if (Input::KeyB.pressed)

    //bulletPtr.DebugDraw();
  }
  output.saveVMD(L"output/きしめん.vmd");
}
#endif
