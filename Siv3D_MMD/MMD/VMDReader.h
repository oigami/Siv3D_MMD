#pragma once
#include <Siv3D.hpp>
#include <src/ReaderHelper.h>
#include <MMD/vmd_struct.h>
namespace s3d_mmd
{
  class VMDReader
  {

    int m_lastFrame;
    vmd::Header header;
    Array<vmd::Bone> keyFrames;
    Array<vmd::Morph> morphFrames;
    Array<vmd::Camera> cameraFrames;
    Array<vmd::SelfShadow> selfShadowFrames;
    Array<vmd::ShowIk> showIKs;
    std::shared_ptr<IReader> reader_;
    FilePath path_;
    bool is_opened_;
    bool openImpl(std::shared_ptr<IReader> reader);
  public:

    VMDReader();

    /// <summary>VMDファイルからデータを取り出す</summary>
    /// <param name="file_name"></param>
    VMDReader(const FilePath& file_name);

    /// <summary>VMDファイルからデータを取り出す</summary>
    /// <param name="file_name"></param>
    template<class Reader, class = std::enable_if_t<std::is_base_of<IReader, Reader>::value>>
    VMDReader(Reader&& reader) { open(std::move(reader)); }

    bool open(const FilePath& file_name);

    template<class Reader, class = std::enable_if_t<std::is_base_of<IReader, Reader>::value>>
    bool open(Reader&& reader) { return open(std::make_shared<Reader>(std::move(reader))); }

    bool open(std::shared_ptr<IReader> reader);

    void close();

    bool hasChanged()const;

    bool isEmpty() const;

    bool reload();

    bool isOpened()const;
    explicit operator bool() const { return isOpened(); }

    const FilePath& path()const { return path_; }

    const vmd::Header& getHeader()const { return header; }

    /// <summary>モーションの最終フレームを返す</summary>
    /// <returns>モーションの最終フレーム</returns>
    int getLastFrame() { return m_lastFrame; }

    double getVersion() const;

    const Array<vmd::Bone>& getKeyFrames()const { return keyFrames; }

    const Array<vmd::Morph>& getMorphFrames() const { return morphFrames; }

    const Array<vmd::Camera>& getCameraFrames() const { return cameraFrames; }

    const Array<vmd::SelfShadow>& getSelfShadowFrames() const { return selfShadowFrames; }

    const Array<vmd::ShowIk>& getShowIk() const { return showIKs; }
  };
}
