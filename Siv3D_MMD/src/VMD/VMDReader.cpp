#include <MMD/VMDReader.h>
#include <src/ReaderHelper.h>
namespace s3d_mmd
{

  /// <summary>VMDファイルからデータを取り出す</summary>
  /// <param name="file_name"></param>
  VMDReader::VMDReader(const FilePath & file_name)
  {
    constexpr int frame_rate = 60;     // 本プログラムのフレームレート
    constexpr int mmd_frame_rate = 30; // MMDのフレームレート

                                       // VMDファイルからVMDデータを抽出
    BinaryReader ifs(file_name);
    if ( !ifs.isOpened() )
      return;
    vmd::Header vmdHeader;
    {
      ifs.read(vmdHeader);

      Array<vmd::Bone> vmdMotions;
      ReadSizeAndArray<uint32>(ifs, vmdMotions);

      // KeyFramesに格納
      last_frame = 0;
      for ( const auto& bone : vmdMotions )
      {
        vmd::KeyFrame keyFrame;
        size_t endPos = 0;
        for ( int i = 0; i < sizeof(bone.boneName); i++ )
        {
          if ( bone.boneName[i] == L'\0' )break;
          endPos++;
        }
        keyFrame.boneName = Widen({ bone.boneName, endPos });
        keyFrame.frameNo = bone.frameNo;
        keyFrame.frameNo *= frame_rate / mmd_frame_rate;
        last_frame = std::max(last_frame, keyFrame.frameNo);
        keyFrame.position = DirectX::XMVectorSet(bone.location[0], bone.location[1], bone.location[2], 0);
        keyFrame.rotation = Quaternion(bone.rotation[0], bone.rotation[1], bone.rotation[2], bone.rotation[3]);
        keyFrame.bezie_x = vmd::Bezie(bone.interpolation[0], bone.interpolation[4], bone.interpolation[8], bone.interpolation[12]);
        keyFrame.bezie_y = vmd::Bezie(bone.interpolation[1], bone.interpolation[5], bone.interpolation[9], bone.interpolation[13]);
        keyFrame.bezie_z = vmd::Bezie(bone.interpolation[2], bone.interpolation[6], bone.interpolation[10], bone.interpolation[14]);
        keyFrame.bezie_r = vmd::Bezie(bone.interpolation[3], bone.interpolation[7], bone.interpolation[11], bone.interpolation[15]);
        getKeyFrames(keyFrame.boneName)->push_back(std::move(keyFrame));
      }

      for ( auto& it : keyFrames )
      {
        auto &i = it.second;
        std::sort(i->begin(), i->end());
      }
    }
    {

      ReadSizeAndArray<uint32>(ifs, vmdSkins);
      for ( auto& i : vmdSkins )
      {
        i.frameNo *= frame_rate / mmd_frame_rate;
      }

    }
  }

  /// <summary>ボーン名に応じたキーフレームを返す</summary>
  /// <param name="bone_name">ボーン名</param>
  /// <returns>キーフレーム</returns>
  std::shared_ptr<Array<vmd::KeyFrame>> VMDReader::getKeyFrames(const String& bone_name)
  {
    auto &frame = keyFrames[bone_name];
    if ( frame == nullptr )
    {
      frame = std::make_shared<Array<vmd::KeyFrame>>();
    }
    return frame;
  }

}
