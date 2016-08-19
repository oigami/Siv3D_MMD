#include <MMD/VMDReader.h>
#include <src/ReaderHelper.h>
namespace s3d_mmd
{
  bool VMDReader::reload()
  {
    if ( !path_.isEmpty )
      return open(path_);
    return false;
  }
  bool VMDReader::isOpened() const
  {
    return is_opened_;
  }
  bool VMDReader::openImpl(std::shared_ptr<IReader> reader)
  {

    if ( !reader->isOpened() )
    {
      return false;
    }
    IReader& ifs = *reader;
    reader_ = reader;
    if ( !ifs.read(header) ) return false;
    constexpr char headerTypeData[] = "Vocaloid Motion Data ";
    if ( strncmp(headerTypeData, header.vmdHeader, sizeof(headerTypeData) - 1) != 0 ) return false;

    if ( !ReadSizeAndArray<uint32>(ifs, keyFrames) ) return false;

    if ( !ReadSizeAndArray<uint32>(ifs, morphFrames) ) return false;

    if ( !ReadSizeAndArray<uint32>(ifs, cameraFrames) ) return false;

    Array<vmd::Light> lightFrame;
    if ( !ReadSizeAndArray<uint32>(ifs, lightFrame) ) return false;

    // MMDv6.19以前で保存されたVMDはここまで

    if ( !ReadSizeAndArray<uint32>(ifs, selfShadowFrames) ) return true;

    // MMDv7.39.x64以前で保存されたVMDはここまで

    {
      uint32 showIKCount;
      if ( !ifs.read(showIKCount) ) return true;
      showIKs.resize(showIKCount);
      for ( auto& i : showIKs )
      {
        if ( !ifs.read(static_cast<vmd::ShowIkWithoutArray&>(i)) ) return false;
        i.ik.resize(i.ik_count);
        if ( !ifs.read(i.ik.data(), i.ik_count) ) return false;
      }
    }
    return true;
  }

  VMDReader::VMDReader() :is_opened_(false) {}

  /// <summary>VMDファイルからデータを取り出す</summary>
  /// <param name="file_name"></param>
  VMDReader::VMDReader(const FilePath & file_name) : VMDReader(BinaryReader(file_name))
  {
    is_opened_ = false;
    if ( open(file_name) )
    {
      path_ = file_name;
    }
  }

  bool VMDReader::open(const FilePath & file_name)
  {
    auto res = open(BinaryReader(file_name));
    if ( res )
      path_ = file_name;
    return res;
  }

  bool VMDReader::open(std::shared_ptr<IReader> reader)
  {
    auto res = openImpl(reader);
    if ( res )
    {
      m_lastFrame = 0;

      for ( auto& i : morphFrames )
        m_lastFrame = std::max(m_lastFrame, static_cast<int>(i.frameNo));

      for ( auto& i : keyFrames )
        m_lastFrame = std::max(m_lastFrame, static_cast<int>(i.frameNo));

    }
    else
    {
      close();
    }
    return res;
  }

  void VMDReader::close()
  {
    is_opened_ = false;
    morphFrames.clear();
    keyFrames.clear();
    reader_.reset();
    m_lastFrame = 0;

  }

  bool VMDReader::hasChanged() const
  {
    return false;
  }

  bool VMDReader::isEmpty() const
  {
    return !isOpened();
  }

  double VMDReader::getVersion() const
  {
    return 0.0;
  }

}
