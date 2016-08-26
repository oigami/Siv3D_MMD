#include <MMD/vmd_reader.h>
#include <src/ReaderHelper.h>
namespace s3d_mmd
{
  namespace
  {
    template<class Type>
    bool ReadSizeAndArray(IReader& reader, std::vector<Type>& arr)
    {
      return s3d_mmd::ReadSizeAndArray<typename Type::CountType>(reader, arr);
    }
  }
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

    if ( !ReadSizeAndArray(ifs, keyFrames) ) return false;

    if ( !ReadSizeAndArray(ifs, morphFrames) ) return false;

    if ( !ReadSizeAndArray(ifs, cameraFrames) ) return false;

    if ( !ReadSizeAndArray(ifs, lightFrame) ) return false;

    // MMDv6.19以前で保存されたVMDはここまで

    if ( !ReadSizeAndArray(ifs, selfShadowFrames) ) return true;

    // MMDv7.39.x64以前で保存されたVMDはここまで

    {
      vmd_struct::ShowIkWithoutArray::CountType showIKCount;
      if ( !ifs.read(showIKCount) ) return true;
      showIKs.resize(showIKCount);
      for ( auto& i : showIKs )
      {
        if ( !ifs.read(static_cast<vmd_struct::ShowIkWithoutArray&>(i)) ) return false;
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

  String VMDReader::getModelName() const
  {
    int endPos = sizeof(header.vmdModelName);
    for ( int i = 0; i < sizeof(header.vmdModelName); i++ )
    {
      if ( header.vmdModelName[i] == '\0' )
      {
        endPos = i;
        break;
      }
    }
    return Widen(std::string(header.vmdModelName, endPos));
  }

}
