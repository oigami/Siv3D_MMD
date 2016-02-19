#pragma once
#include <Siv3D.hpp>
namespace s3d_mmd {
  template<class Type>
  bool ReadArray(IReader& reader, const size_t size, std::vector<Type> &arr) { //Array�Ŏ󂯎��Ȃ��̂�vector�ɂ��Ă���
    arr.resize(size);
    if (size == 0 || reader.read(arr.data(), sizeof(Type) * size))
      return true;
    return false;
  }

  /// <summary>
  /// �ŏ���sizeof(SizeType)�����ǂݍ��݂��̒l�̔z�񕪓ǂݍ���
  /// </summary>
  /// <remarks>
  /// �T�C�Y0�̏ꍇ������I��
  /// </remarks>
  /// <returns>
  /// empty   : �T�C�Y�̓ǂݍ��݂Ɏ��s
  /// ����ȊO : �f�[�^�̓ǂݍ��ݐ���
  /// </returns>
  template<class SizeType, class Type>
  Optional<bool> ReadSizeAndArray(IReader& reader, std::vector<Type> &arr) {//Array�Ŏ󂯎��Ȃ��̂�vector�ɂ��Ă���
    SizeType num;
    if (!reader.read(num)) return{};
    return ReadArray(reader, num, arr);
  }
}
