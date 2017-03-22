#include <MMD/vmd_writer.h>
#include <src/reader_helper.h>

namespace s3d_mmd
{
  namespace mmd
  {
    namespace
    {
      template<class Type>
      bool WriteSizeAndArray(IWriter& writer, std::vector<Type>& arr)
      {
        return s3d_mmd::WriteSizeAndArray<typename Type::CountType>(writer, arr);
      }
    }

    VMDWriter::VMDWriter(const FilePath& outFileName)
    {
      writer_.open(outFileName);
    }

    VMDWriter::~VMDWriter()
    {
      close();
    }

    bool VMDWriter::isOpened()
    {
      return writer_.isOpened();
    }

    void VMDWriter::close()
    {
      if ( !isOpened() ) return;

      {
        // ヘッダの書き込み
        vmd_struct::Header header{ "Vocaloid Motion Data 0002" };
        std::string name = Narrow(modelName);
        strncpy(header.vmdModelName, name.c_str(),
                Min(name.size(), sizeof(header.vmdModelName)));
        writer_.write(header);
      }

      {
        // ボーンフレームの書き込み
        WriteSizeAndArray(writer_, boneFrame);
      }

      {
        // モーフフレームの書き込み
        WriteSizeAndArray(writer_, morphFrame);
      }

      {
        // カメラフレームの書き込み
        WriteSizeAndArray(writer_, cameraFrame);
      }

      {
        // ライトフレームの書き込み
        WriteSizeAndArray(writer_, lightFrame);
      }

      {
        // セルフシャドウフレームの書き込み
        WriteSizeAndArray(writer_, selfShadowFrame);
      }

      {
        // Ikフレームの書き込み
        writer_.write(static_cast<vmd_struct::ShowIkWithoutArray::CountType>(showIkFrame.size()));
        for ( auto& i : showIkFrame )
        {
          i.ik_count = static_cast<uint32>(i.ik.size());
          writer_.write(static_cast<vmd_struct::ShowIkWithoutArray&>(i));
          writer_.write(i.ik.data(), sizeof(i.ik[0]) * i.ik_count);
        }
      }

      writer_.close();
    }
  }
}
