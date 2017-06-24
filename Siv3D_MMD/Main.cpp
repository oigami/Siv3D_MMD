#include "src/mmd_physics.h"
#include <future>
#ifndef LIB_COMPILE
# include <Siv3D.hpp>
#include "MMD/MMD.h"
using namespace s3d_mmd;
#include <iostream>
#include <MMD/mmd_motion.h>

#include <MMD/mouse_camera3d.h>

#include <thread>
#include <experimental/filesystem>
#include <deque>
#include <picojson.h>

namespace fs = std::experimental::filesystem;
constexpr int InputPoseNum = 18;
constexpr int OutputPoseNum = 12;

struct DataX
{
  std::vector<String> dataFile_;
  std::future<void> thread_;
  struct Output
  {
    std::array<Vec2, InputPoseNum> pos2D;
    std::array<Vec4, OutputPoseNum> pos3D;
  };
  std::deque<Output*> outputs_;
  MMD mmd_;

  DataX(const MMDModel& model) :mmd_(model)
  {
  }

  void Run(int cnt,
           const std::array<Vec3, InputPoseNum>& diff,
           const std::array<int, InputPoseNum>& pose2d_int,
           const std::array<int, OutputPoseNum>& pose3d_int)
  {
    Vector halfSize{ Window::Width() / 2, -Window::Height() / 2,0.0f,0.0f };
    Vector halfSize2{ Window::Width() / 2, Window::Height() / 2,0.0f,0.0f };

    Camera camerax;
    camerax.pos = Vec3(0, 10, -40);
    camerax.lookat = Vec3(0, 10, 0);

    for ( auto& file : dataFile_ )
    {
      VMD vmd(file);
      if ( vmd.isEmpty() ) continue;
      mmd_.attach(vmd);
      int lastFrame = vmd.lengthFrame();
      for ( auto& frame : step(0, lastFrame / 5, 5) )
      {
        vmd.setPosFrame(frame);
        mmd_.update();

        auto& bones = *mmd_.bones();
        auto& localMats = bones.lastUpdatedModelLocal();

        std::array<Mat4x4, InputPoseNum> worlds;
        for ( auto& i : step(InputPoseNum) )
        {
          worlds[i] = Mat4x4::Translate(diff[i]) * localMats[pose2d_int[i]];
        }
        Output *output = new Output();

        for ( auto& i : step(OutputPoseNum) )
        {
          output->pos3D[i] = ToVec4(bones[pose3d_int[i]].boneRotation.component);
        }

        auto& tmp = output->pos2D;
        for ( auto& i : step(InputPoseNum) )
        {
          auto v = worlds[i].r[3];
          auto t = DirectX::XMVector3TransformCoord(v, camerax.calcViewProjectionMatrix());
          //t = t * halfSize;
          t = t * Vector{ 1,-1,0,0 };
          //t = t + halfSize2;
          tmp[i] = ToVec3(t).xy();
        }

        outputs_.push_back(output);
      }
    }
    save(cnt);
  }
  void save(int cnt)
  {
    picojson::array o(outputs_.size());
    for ( auto& j : step(outputs_.size()) )
    {
      picojson::object data_set;
      picojson::array in(outputs_[j]->pos2D.size() * 2);
      picojson::array out(outputs_[j]->pos3D.size() * 4);
      for ( auto& k : step(outputs_[j]->pos2D.size()) )
      {
        in[k * 2] = picojson::value(outputs_[j]->pos2D[k].x);
        in[k * 2 + 1] = picojson::value(outputs_[j]->pos2D[k].y);
      }
      for ( auto& k : step(outputs_[j]->pos3D.size()) )
      {
        out[k * 4] = picojson::value(outputs_[j]->pos3D[k].x);
        out[k * 4 + 1] = picojson::value(outputs_[j]->pos3D[k].y);
        out[k * 4 + 2] = picojson::value(outputs_[j]->pos3D[k].z);
        out[k * 4 + 3] = picojson::value(outputs_[j]->pos3D[k].w);
      }
      data_set.insert(std::make_pair("input", in));
      data_set.insert(std::make_pair("output", out));
      o[j] = picojson::value(data_set);
    }
    picojson::value v(o);
    std::string str = v.serialize(false);
    ZIPWriter writer(Format(L"", cnt, L".zip"));
    writer.addFile(L"data.json", str.data(), str.size());
  }
};
using ll = long long;
const ll mod = (ll)1e9 + 7;
template<ll MOD>struct Mint
{
  typedef const Mint&T;
  ll x = 0;
  Mint() {}
  Mint(ll a) { x = a % MOD; if ( x < 0 ) x += MOD; }
  Mint&operator+=(T a) { if ( (x += a.x) >= MOD ) x -= MOD; return *this; }
  Mint&operator-=(T a) { if ( (x += MOD - a.x) >= MOD ) x -= MOD; return *this; }
  Mint&operator*=(T a) { x = x * a.x % MOD; return *this; }
  Mint&operator/=(T a) { return *this *= a.Pow(MOD - 2); }
  Mint operator+(T a)const { return Mint(*this) += a; }
  Mint operator-(T a)const { return Mint(*this) -= a; }
  Mint operator*(T a)const { return Mint(*this) *= a; }
  Mint operator/(T a)const { return Mint(*this) /= a; }
  bool operator<(T a)const { return x < a.x; }
  bool operator==(T a)const { return x == a.x; }
  Mint Pow(ll k)const
  {
    if ( k == 0 ) return 1;
    Mint t = Pow(k / 2);
    return t *= (k & 1) ? t * *this : t;
  }
};
typedef Mint<mod>mint;

void Main()
{
  Window::SetStyle(WindowStyle::Sizeable);
  Graphics::SetBackground(Color(80, 160, 230));
  Graphics3D::SetAmbientLight(ColorF(0.3));
  physics3d::Physics3DWorld world;
  MMD::SetDefaultPhysicsWorld(std::make_shared<MMDPhysicsWorld>(world));
  const MMDModel model(L"Data/初音ミク/初音ミクVer2.pmd");
  Println(model.name());
  Println(model.comment());
  const Mesh meshGround(MeshData::Plane({ 40, 40 }, { 6, 6 }));

  mmd::MMDMotion motion(VMDReader(L"Data/きしめん.vmd"));
  mmd::MMDMotion output = motion;
  motion.saveVMD(L"Data/きしめん.vmd.sav");
  const VMD vmd(L"Data/きしめん.vmd");

  //vmd.play();
  //Bone &bone10 = model.bones()->get(10);
  //bone10.extraBoneControl = true;
  const Font font(30);
  GUI gui(GUIStyle::Default);
  gui.setTitle(L"タイトル");

  gui.add(L"frame", GUISlider::Create(0, 10, 0));
  MouseCamera3D camera;
  System::Update();
  int nowFrame = 0;

  auto pose2d_int = [&model]()
  {
    std::array<int, InputPoseNum> tmp;
    std::array<String, InputPoseNum> lists = {
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
    for ( auto& i : step(InputPoseNum) )
    {
      tmp[i] = *model.bones()->getBoneIndex(lists[i]);
    }
    return tmp;
  }();

  auto pose3d_int = [&model]()
  {
    std::array<int, OutputPoseNum> tmp;
    std::array<String, OutputPoseNum> lists = {
      L"頭",
      L"首",
      L"右肩", L"右腕", L"右ひじ",
      L"左肩", L"左腕", L"左ひじ",
      L"右足", L"右ひざ",
      L"左足", L"左ひざ",
    };
    for ( auto& i : step(lists.size()) )
    {
      tmp[i] = *model.bones()->getBoneIndex(lists[i]);
    }
    return tmp;
  }();


  std::array<std::vector<int>, InputPoseNum> linkList = {
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
  std::array<Vec3, InputPoseNum> diff = {
    Vec3(0.3, -0.9, -0.8),
    Vec3(0, -0.8, 0),
    Vec3(), Vec3(), Vec3(),
    Vec3(), Vec3(), Vec3(),
    Vec3(), Vec3(), Vec3(),
    Vec3(), Vec3(), Vec3(),
    Vec3(0,-0.6,-0.6), Vec3(0,-0.6,-0.6),
    Vec3(-0.3, -0.6, 0), Vec3(0.3, -0.6, 0),
  };

  constexpr int threadNum = 8;

  std::array<DataX, threadNum> data = {
   DataX { model },
    { model },
    { model },
    { model },
    { model },
    { model },
    { model },
    { model },
  };
  fs::directory_iterator it("D:\\VisualStudio\\git\\Pose3dChainer\\dl\\vmd"), last;

  for ( Mint<threadNum> cnt; it != last; ++it, cnt += 1 )
  {
    data[cnt.x].dataFile_.push_back(it->path().wstring());
  }
  for ( auto i : step(data.size()) )
  {
    data[i].thread_ = std::async(std::launch::async, [&, i]()
    {
      data[i].Run(i, diff, pose2d_int, pose3d_int);
    });
  }

  for ( auto& cnt : step(data.size()) )
  {
    auto& i = data[cnt];
    i.thread_.wait();

  }
}
#endif

