#include <Siv3D.hpp>
#include "../../include/MMDBone.h"
#include "../../include/PMDStruct.h"
#include "../../include/VMDController.h"
#include "../ReaderHelper.h"
namespace s3d_mmd {

  namespace vmd {
    Bezie::Bezie(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2) {
      p1.x = x1 / 127.0f;
      p1.y = y1 / 127.0f;
      p2.x = x2 / 127.0f;
      p2.y = y2 / 127.0f;
    }

    /// http://d.hatena.ne.jp/edvakf/20111016/1318716097
    /// 二分探索
    float Bezie::GetY(float x) const {
      float t = 0.5f;
      float a, b, c;
      constexpr int N = 15; // 計算繰り返し回数
      for (int i = 0; i < N; ++i) {
        const float s = 1 - t;
        a = s * s * t * 3.0f;
        b = s * t * t * 3.0f;
        c = t * t * t;
        float ft = static_cast<float>(a * p1.x + b * p2.x + c - x);
        if (fabs(ft) < 1e-6) break; // 誤差が定数以内なら終了
        if (ft > 0) { // 範囲を変更して再計算
          t -= 1.0f / (4 << i);
        } else {
          t += 1.0f / (4 << i);
        }
      }
      return static_cast<float>(a * p1.y + b * p2.y + c);
    }
  }


  class VMD::Pimpl {

    Array<uint32> m_nowFrameNumber;
    bool m_blendFlag;
    bool m_isFrameEnd;  /// <summary> フレームが終了したか true:終了 </summary>
    bool m_isLoop;      /// <summary> ループするかどうか </summary>
    uint32 m_loopStart; /// <summary> ループする時間 </summary>
    uint32 m_loopEnd;   /// <summary> ループする時間 </summary>
    uint32 m_nowTime;   /// <summary> 現在の再生時間 </summary>
    Array<pmd::IkData>* m_pmdIkData;

    /// キーフレームアニメーション
    struct KeyFrameData {
      KeyFrameData() :m_nowFrameNum(0) {}

      const vmd::KeyFrame& getNowFrame() const {
        return (*m_keyFrames)[m_nowFrameNum];
      }

      const vmd::KeyFrame& getNextFrame() const {
        return (*m_keyFrames)[m_nowFrameNum + 1];
      }

      bool haveNextFrame() const {
        return m_nowFrameNum + 1 < m_keyFrames->size();
      }

      bool haveNowFrame() const {
        return m_nowFrameNum < m_keyFrames->size();
      }

      std::shared_ptr<Array<vmd::KeyFrame>> m_keyFrames;
      int m_nowFrameNum;
    };

    /// <summary> キーフレームの名前とデータ </summary>
    std::unordered_map<std::string, KeyFrameData> m_keyFrameData;
    Array<Mat4x4> worlds;

  public:

    Pimpl(VMDReader &data) {
      m_nowTime = 0;
      m_loopStart = 0;
      m_loopEnd = 0;
      m_isLoop = false;
      m_blendFlag = true;
      m_isFrameEnd = true;
      for (auto& i : data.getKeyFrames()) {
        m_keyFrameData[i.first].m_keyFrames = i.second;
      }
      //m_bones.m_bones = std::move(*bone);

      m_isFrameEnd = true;
      m_loopEnd = data.GetLastFrame();
    }
    Pimpl() = default;

    void UpdateIK(mmd::Bones &bones) const;		// IKボーン影響下ボーンの行列を更新
    void UpdateIK(mmd::Bones &bones, const mmd::Ik &ik) const;		// IKボーン影響下ボーンの行列を更新
    void UpdateBone(mmd::Bones &bons)const;
    void UpdateTime();

  };

  // IKボーン影響下ボーンの行列を更新
  void VMD::Pimpl::UpdateIK(mmd::Bones &bones) const {
    for (auto& ikData : bones.ikData())
      UpdateIK(bones, ikData);
  }
  void VMD::Pimpl::UpdateIK(mmd::Bones &bones, const mmd::Ik &ikData) const {
    for (int i : step(ikData.iterations)) {
      Vector localEffectorPos{}, localTargetPos{};
      for (auto& attentionIdx : ikData.ik_child_bone_index) {
        using namespace DirectX;
        Mat4x4 effectorMat = bones.CalcBoneMatML(ikData.ik_target_bone_index);
        Mat4x4 targetMat = bones.CalcBoneMatML(ikData.ik_bone_index);
        const Vector &effectorPos = effectorMat.r[3];
        const Vector &targetPos = targetMat.r[3];
        XMVECTOR Determinant;
        const Mat4x4 invCoord = XMMatrixInverse(&Determinant, bones.CalcBoneMatML(attentionIdx));
        localEffectorPos = XMVector3TransformCoord(effectorPos, invCoord); // 注目ボーン基準に変換
        localTargetPos = XMVector3TransformCoord(targetPos, invCoord);     // 注目ボーン基準に変換
        // エフェクタのローカル方向（注目ボーン基準）
        Vector localEffectorDir = XMVector3Normalize(localEffectorPos);
        // ターゲットのローカル方向（注目ボーン基準）
        Vector localTargetDir = XMVector3Normalize(localTargetPos);
        const auto findIt = bones[attentionIdx].name.find("ひざ");
        if (findIt != std::string::npos) {
          localEffectorDir = XMVector3Normalize(XMVectorSetX(localEffectorDir, 0));
          localTargetDir = XMVector3Normalize(XMVectorSetX(localTargetDir, 0));
        }
        float p;
        XMStoreFloat(&p, XMVector3Dot(localEffectorDir, localTargetDir));
        if (p > 1 - 1.0e-8f) continue; // 計算誤差により1を越えるとacos()が発散するので注意!
        float angle = acos(p);
        if (angle > 4 * ikData.control_weight)
          angle = 4.0f * ikData.control_weight;
        const Vector axis = XMVector3Cross(localEffectorDir, localTargetDir);
        //ベクトルがゼロのときエラーになるので確認する
        XMMATRIX rotation = XMVector3Equal(axis, XMVectorZero()) ?
          DirectX::XMMatrixIdentity() : DirectX::XMMatrixRotationAxis(axis, angle);
        const XMMATRIX &xmboneMatBL = bones[attentionIdx].boneMat;
        if (findIt != std::string::npos) {
          const XMMATRIX inv = XMMatrixInverse(&Determinant, bones[attentionIdx].initMat);
          const XMMATRIX def = rotation * xmboneMatBL * inv;
          Vector t = XMVector3TransformCoord(XMVectorSet(0, 0, 1, 0), def);
          if (XMVectorGetY(t) < 0) rotation = DirectX::XMMatrixRotationAxis(axis, -angle);
          // 膝ボーンがエフェクタ(ターゲットボーン)より近い時は回転量を追加する
          const float l = XMVectorGetY(XMVector3Length(localTargetPos)) / XMVectorGetY(XMVector3Length(localEffectorPos));
          if (fabs(angle) <= XM_PI / 2 && l < 1.0f) {
            constexpr float a = 0.5f; // 追加量の比例係数
            float diff = acosf(l) * a; // 追加量
            static const float diff_limit = XM_PI / 6;	// 追加量の制限
            if (diff > diff_limit) {
              diff = diff_limit;
            }
            if (fabs(angle) > 1.0e-6f) diff *= angle / fabs(angle);	// 符号合わせ
            angle += diff;
          }
        }
        bones[attentionIdx].boneMat = rotation * xmboneMatBL;
      }
      constexpr float errToleranceSq = 0.000001f;
      using namespace DirectX;
      if (DirectX::XMVectorGetY(DirectX::XMVector3LengthSq(localEffectorPos - localTargetPos)) < errToleranceSq) {
        return;
      }
    }
  }

  void VMD::Pimpl::UpdateBone(mmd::Bones &bones) const {
    using namespace DirectX;
    for (auto &i : bones) {
      auto it = m_keyFrameData.find(i.name);
      if (it == m_keyFrameData.end()) continue;
      const auto &keyData = it->second;
      // キーフレーム補完
      if (keyData.haveNowFrame()) {
        const auto &nowFrame = keyData.getNowFrame();
        const int t0 = nowFrame.frameNo;
        Vector boneRot = nowFrame.rotation.component;
        Vec3 bonePos = nowFrame.position;
        if (keyData.haveNextFrame()) {
          //次のフレームとの間の位置を計算する
          const Vec3 &p0 = bonePos;
          const vmd::KeyFrame &next = keyData.getNextFrame();
          const int t1 = next.frameNo;
          const Vec3 &p1 = next.position;
          float s = (float)(m_nowTime - t0) / float(t1 - t0);
          boneRot = XMQuaternionSlerp(boneRot, next.rotation.component, next.bezie_r.GetY(s));
          bonePos.x = p0.x + (p1.x - p0.x) * next.bezie_x.GetY(s);
          bonePos.y = p0.y + (p1.y - p0.y) * next.bezie_y.GetY(s);
          bonePos.z = p0.z + (p1.z - p0.z) * next.bezie_z.GetY(s);
        }
        // 親ボーン座標系のボーン行列を求める
        const XMMATRIX rot = XMMatrixRotationQuaternion(boneRot);
        const Mat4x4 trans = Mat4x4::Translate(bonePos.x, bonePos.y, bonePos.z);
        i.boneMat = rot * trans * i.initMat;
      }
    }
    UpdateIK(bones);
  }

  void VMD::Pimpl::UpdateTime() {
    m_nowTime++;
    for (auto& i : m_keyFrameData) {
      KeyFrameData &keyData = i.second;
      const size_t keyframe_size = keyData.m_keyFrames->size();
      int &nowFrameNum = keyData.m_nowFrameNum;
      if (nowFrameNum + 1 < keyframe_size) {
        const Array<vmd::KeyFrame> &key_frame = *keyData.m_keyFrames;
        const uint32 t1 = key_frame[nowFrameNum + 1].frameNo;
        if (m_nowTime > t1) ++nowFrameNum;
      }
    }
  }

  VMD::VMD() {
    m_handle = std::make_shared<Pimpl>();
  }

  VMD::VMD(const FilePath & filename) {
    m_handle = std::make_shared<Pimpl>(VMDReader(filename));
  }

  VMD::~VMD() {
  }
  void VMD::UpdateBone(mmd::Bones &bones) const {
    m_handle->UpdateBone(bones);
  }
  void VMD::UpdateTime() const {
    m_handle->UpdateTime();
  }


}
