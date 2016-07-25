#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd
{
  namespace vmd
  {

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
      float location[3]; // 移動量
      float rotation[4]; // モデルローカル座標系
      std::uint8_t interpolation[64];
    };

    struct Morph
    {
      char name[15];
      std::uint32_t frameNo;
      float weight;
    };

    struct Camera
    {
      std::uint32_t frameNo;   // フレーム番号
      float distance;          // 目標点とカメラの距離(目標点がカメラ前面でマイナス)
      float x;                 // 目標点のX軸位置
      float y;                 // 目標点のY軸位置
      float z;                 // 目標点のZ軸位置
      float rx;                // カメラのx軸回転(rad)(MMD数値入力のマイナス値)
      float ry;                // カメラのy軸回転(rad)
      float rz;                // カメラのz軸回転(rad)
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

    public:

      Bezie() = default;
      Bezie(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);
      float GetY(float x) const;	/// xにおけるyを取得

      float newton(float t, float x) const
      {
        auto  f = [&](float t, float x)
        {
          return 3 * (1 - t) * (1 - t) * t * p1.x + 3 * (1 - t) * t * t * p2.x + t * t * t - x;
        };
        auto fd = [&](float t)
        {
          return 3 * t * t * (3 * (p1.x - p2.x) + 1) + 6 * t * (p2.x - 2 * p1.x) + 3 * p1.x;
        };
        constexpr int N = 16;
        for ( int i = N - 1; i >= 0; --i )
        {
          float t1 = t - f(t, x) / fd(t);
          if ( fabs(t1 - t) < 1e-6 ) break;
          t = t1;
        }
        return t;
      }
    };

    struct KeyFrame
    {

      std::string boneName; /// <summary>ボーン名</summary>
      int frameNo;          /// <summary>フレーム番号</summary>
      Vec3 position;        /// <summary>位置</summary>
      Quaternion rotation;  /// <summary>回転</summary>
      Bezie bezie_x;
      Bezie bezie_y;
      Bezie bezie_z;
      Bezie bezie_r;

      // フレーム番号で比較
      bool operator<(const KeyFrame &k) const
      {
        return frameNo < k.frameNo;
      }

    };
  }

  class VMDReader
  {

    int last_frame;
    std::map<std::string, std::shared_ptr<Array<vmd::KeyFrame>>> keyFrames;
  public:

    /// <summary>VMDファイルからデータを取り出す</summary>
    /// <param name="file_name"></param>
    VMDReader(const FilePath &file_name);

    /// <summary>ボーン名に応じたキーフレームを返す</summary>
    /// <param name="bone_name">ボーン名</param>
    /// <returns>キーフレーム</returns>
    std::shared_ptr<Array<vmd::KeyFrame>> getKeyFrames(const std::string& bone_name);

    /// <summary>ボーン名に応じたキーフレームを返す</summary>
    /// <param name="bone_name">ボーン名</param>
    /// <returns>キーフレーム</returns>
    std::map<std::string, std::shared_ptr<Array<vmd::KeyFrame>>> getKeyFrames()
    {
      return keyFrames;
    }

    /// <summary>モーションの最終フレームを返す</summary>
    /// <returns>モーションの最終フレーム</returns>
    int GetLastFrame() { return last_frame; }

  };
}
