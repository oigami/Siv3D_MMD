#include "../../include/VMDReader.h"
#include "../ReaderHelper.h"
namespace s3d_mmd {

  /// <summary>VMDファイルからデータを取り出す</summary>
  /// <param name="file_name"></param>
  VMDReader::VMDReader(const FilePath & file_name) {
    constexpr int frame_rate = 60;     // 本プログラムのフレームレート
    constexpr int mmd_frame_rate = 30; // MMDのフレームレート

                                       // VMDファイルからVMDデータを抽出
    BinaryReader ifs(file_name);
    if (!ifs.isOpened())
      return;
    vmd::Header vmdHeader;
    ifs.read(vmdHeader);

    Array<vmd::Bone> vmdMotions;
    ReadSizeAndArray<uint32>(ifs, vmdMotions);

    // KeyFramesに格納
    last_frame = 0;
    for (const auto& i : vmdMotions) {
      vmd::KeyFrame keyFrame;
      keyFrame.boneName = i.boneName;
      keyFrame.frameNo = i.frameNo;
      keyFrame.frameNo *= frame_rate / mmd_frame_rate;
      last_frame = std::max(last_frame, keyFrame.frameNo);
      keyFrame.position = Vec3(i.location[0], i.location[1], i.location[2]);
      keyFrame.rotation = Quaternion(i.rotation[0], i.rotation[1], i.rotation[2], i.rotation[3]);
      keyFrame.bezie_x = vmd::Bezie(i.interpolation[0], i.interpolation[4], i.interpolation[8], i.interpolation[12]);
      keyFrame.bezie_y = vmd::Bezie(i.interpolation[1], i.interpolation[5], i.interpolation[9], i.interpolation[13]);
      keyFrame.bezie_z = vmd::Bezie(i.interpolation[2], i.interpolation[6], i.interpolation[10], i.interpolation[14]);
      keyFrame.bezie_r = vmd::Bezie(i.interpolation[3], i.interpolation[7], i.interpolation[11], i.interpolation[15]);
      getKeyFrames(i.boneName)->push_back(std::move(keyFrame));
    }

    for (auto& it : keyFrames) {
      auto &i = it.second;
      std::sort(i->begin(), i->end());
    }
  }

  /// <summary>ボーン名に応じたキーフレームを返す</summary>
  /// <param name="bone_name">ボーン名</param>
  /// <returns>キーフレーム</returns>
  std::shared_ptr<Array<vmd::KeyFrame>> VMDReader::getKeyFrames(const std::string & bone_name) {
    auto &frame = keyFrames[bone_name];
    if (frame == nullptr) {
      frame = std::make_shared<Array<vmd::KeyFrame>>();
    }
    return frame;
  }

}
