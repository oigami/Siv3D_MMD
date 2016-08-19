#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd
{
  namespace vmd
  {
    static_assert(std::is_pod<Float2>::value, "");
    static_assert(std::is_pod<Float3>::value, "");
    static_assert(std::is_pod<Float4>::value, "");
    static_assert(std::is_pod<Vector4D<std::uint8_t>>::value, "");
#pragma pack(push, 1)

    //http://harigane.at.webry.info/201103/article_1.html

    /// VMD構造体定義
    struct Header
    {
      char vmdHeader[30];
      char vmdModelName[20];
    };

    struct Bone
    {
      char boneName[15];
      std::uint32_t frameNo;
      Float3 location;  // 移動量
      Float4 rotation;  // モデルローカル座標系
      Vector4D<std::uint8_t> x1; // 軸の補完パラメータ (x,y,z,r)
      Vector4D<std::uint8_t> y1; // 軸の補完パラメータ (x,y,z,r)
      Vector4D<std::uint8_t> x2; // 軸の補完パラメータ (x,y,z,r)
      Vector4D<std::uint8_t> y2; // 軸の補完パラメータ (x,y,z,r)
      std::uint8_t _interpolation[64 - 4 * 4];
    };

    struct Morph
    {
      char name[15];
      std::uint32_t frameNo;
      float m_weight;
    };

    struct Camera
    {
      std::uint32_t frameNo;   // フレーム番号
      float distance;          // 目標点とカメラの距離(目標点がカメラ前面でマイナス)
      Float3 lookAtPos;        // 目標点
      Float3 rotation;         // カメラの回転(rad)(x軸はMMD数値入力のマイナス値)
      std::uint8_t bezier[24]; // 補間パラメータ
      std::uint32_t viewAngle; // 視野角(deg)
      std::uint8_t parth;      // パースペクティブ, 0:ON, 1:OFF
    };

    struct Light
    {
      std::uint32_t frame; // フレーム番号
      float r;             // 照明色赤(MMD入力値を256で割った値)
      float g;             // 照明色緑(MMD入力値を256で割った値)
      float b;             // 照明色青(MMD入力値を256で割った値)
      float x;             // 照明x位置(MMD入力値)
      float y;             // 照明y位置(MMD入力値)
      float z;             // 照明z位置(MMD入力値)
    };

    struct SelfShadow
    {
      std::uint32_t frame; // フレーム番号
      std::uint8_t type;   // セルフシャドウ種類, 0:OFF, 1:mode1, 2:mode2
      float distance;      // シャドウ距離(MMD入力値Lを(10000-L)/100000とした値)
    };

    struct InfoIk
    {
      char name[20];       // "右足ＩＫ\0"などのIKボーン名の文字列 20byte
      std::uint8_t on_off; // IKのon/off, 0:OFF, 1:ON
    };

    struct ShowIkWithoutArray
    {
      std::uint32_t frame;    // フレーム番号
      std::uint8_t show;      // モデル表示, 0:OFF, 1:ON
      std::uint32_t ik_count; // 記録するIKの数

                              //InfoIK ik[ik_count];  // IK on/off情報配列
    };

#pragma pack(pop)

    struct ShowIk : ShowIkWithoutArray
    {
      Array<InfoIk> ik;
    };

    /// 0～1に規格化されたベジェ曲線
    class Bezie
    {

      Float2 p1, p2; /// 制御点

      mutable float pre_x;
      mutable float pre_out;
    public:

      Bezie() = default;
      Bezie(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);
      float GetY(float x) const;	/// xにおけるyを取得

      float newton(float t, float x) const;
    };

    struct BoneFrame
    {

      DirectX::XMVECTOR position;        /// <summary>位置</summary>
      String boneName; /// <summary>ボーン名</summary>
      int frameNo;          /// <summary>フレーム番号</summary>
      Quaternion rotation;  /// <summary>回転</summary>
      Bezie bezie_x;
      Bezie bezie_y;
      Bezie bezie_z;
      Bezie bezie_r;

      // フレーム番号で比較
      bool operator<(const BoneFrame &k) const
      {
        return frameNo < k.frameNo;
      }

    };
  }

}
