#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd
{
  template<class Type>
  bool ReadArray(IReader& reader, const size_t size, std::vector<Type> &arr)
  { //Arrayで受け取れないのでvectorにしておく
    static_assert(std::is_pod<Type>::value, "");
    arr.resize(size);
    if ( size == 0 || reader.read(arr.data(), sizeof(Type) * size) )
      return true;
    return false;
  }

  /// <summary>
  /// 最初にsizeof(SizeType)だけ読み込みその値の配列分読み込む
  /// </summary>
  /// <remarks>
  /// サイズ0の場合も正常終了
  /// </remarks>
  /// <returns>
  /// empty   : サイズの読み込みに失敗
  /// それ以外 : データの読み込み成否
  /// </returns>
  template<class SizeType, class Type>
  bool ReadSizeAndArray(IReader& reader, std::vector<Type> &arr)
  {//Arrayで受け取れないのでvectorにしておく
    SizeType num;
    if ( !reader.read(num) ) return false;
    return ReadArray(reader, num, arr);
  }
}
