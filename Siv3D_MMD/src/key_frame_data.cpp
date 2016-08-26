﻿#include <MMD/vmd_struct.h>
#include <MMD/key_frame_data.h>
namespace s3d_mmd
{
  namespace mmd
  {
    namespace
    {
      template<int n>void ConvertString(char(&out)[n], const String& in)
      {
        std::string name = Narrow(in);
        name.resize(n);
        strncpy(out, name.c_str(), n);
      }

      template<int n>String ConvertString(const char(&in)[n])
      {
        size_t endPos = 0;
        for ( int i = 0; i < n; i++ )
        {
          if ( in[i] == L'\0' )break;
          endPos++;
        }
        return Widen({ in,endPos });
      }
    }
    namespace key_frame
    {
      constexpr int frame_rate = 60;     // 本プログラムのフレームレート
      constexpr int mmd_frame_rate = 30; // MMDのフレームレート
      String BoneFrame::set(const vmd_struct::Bone & boneFrame)
      {

        frameNo = boneFrame.frameNo;
        frameNo *= frame_rate / mmd_frame_rate;
        position = DirectX::XMVectorSet(boneFrame.location.x, boneFrame.location.y, boneFrame.location.z, 0);
        rotation = Quaternion(boneFrame.rotation.x, boneFrame.rotation.y, boneFrame.rotation.z, boneFrame.rotation.w);
        bezie_x = math::Bezie(boneFrame.x1.x, boneFrame.y1.x, boneFrame.x2.x, boneFrame.y2.x);
        bezie_y = math::Bezie(boneFrame.x1.y, boneFrame.y1.y, boneFrame.x2.y, boneFrame.y2.y);
        bezie_z = math::Bezie(boneFrame.x1.z, boneFrame.y1.z, boneFrame.x2.z, boneFrame.y2.z);
        bezie_r = math::Bezie(boneFrame.x1.w, boneFrame.y1.w, boneFrame.x2.w, boneFrame.y2.w);

        return ConvertString(boneFrame.boneName);
      }

      vmd_struct::Bone BoneFrame::Convert(const String & boneName)
      {
        vmd_struct::Bone res{ {} };

        res.frameNo = frameNo;
        res.frameNo /= frame_rate / mmd_frame_rate;

        {
          const Vector2D<uint8>&
            x_p1 = bezie_x.getP1() / 3 * 127, x_p2 = bezie_x.getP2() / 3 * 127,
            y_p1 = bezie_y.getP1() / 3 * 127, y_p2 = bezie_y.getP2() / 3 * 127,
            z_p1 = bezie_z.getP1() / 3 * 127, z_p2 = bezie_z.getP2() / 3 * 127,
            r_p1 = bezie_r.getP1() / 3 * 127, r_p2 = bezie_r.getP2() / 3 * 127;

          res.x1 = { x_p1.x, y_p1.x, z_p1.x, r_p1.x };
          res.y1 = { x_p1.y, y_p1.y, z_p1.y, r_p1.y };
          res.x2 = { x_p2.x, y_p2.x, z_p2.x, r_p2.x };
          res.y2 = { x_p2.y, y_p2.y, z_p2.y, r_p2.y };

          res._interpolation1 = {
            y_p1.x, z_p1.x, r_p1.x, x_p1.y ,
            y_p1.y, z_p1.y, r_p1.y, x_p2.x ,
            y_p2.x, z_p2.x, r_p2.x, x_p2.y ,
            y_p2.y, z_p2.y, r_p2.y, 127
          };
          res._interpolation2 = {
            z_p1.x, r_p1.x, x_p1.y ,
            y_p1.y, z_p1.y, r_p1.y, x_p2.x ,
            y_p2.x, z_p2.x, r_p2.x, x_p2.y ,
            y_p2.y, z_p2.y, r_p2.y, 127, 0
          };
          res._interpolation3 = {
           r_p1.x, x_p1.y ,
            y_p1.y, z_p1.y, r_p1.y, x_p2.x ,
            y_p2.x, z_p2.x, r_p2.x, x_p2.y ,
            y_p2.y, z_p2.y, r_p2.y, 127, 0, 0
          };

        }

        {
          DirectX::XMFLOAT4 rot;
          DirectX::XMStoreFloat4(&rot, rotation.component);
          res.rotation = { rot.x, rot.y, rot.z, rot.w };
        }

        {
          DirectX::XMFLOAT3 location;
          DirectX::XMStoreFloat3(&location, position);
          res.location = { location.x, location.y, location.z };
        }

        {
          ConvertString(res.boneName, boneName);
        }

        return res;
      }

      String MorphFrame::set(const vmd_struct::Morph & morph)
      {
        m_weight = morph.m_weight;
        frameNo = morph.frameNo;
        frameNo *= frame_rate / mmd_frame_rate;

        return ConvertString(morph.name);
      }

      vmd_struct::Morph MorphFrame::convert(const String & name) const
      {
        vmd_struct::Morph morph;
        ConvertString(morph.name, name);
        morph.frameNo = frameNo;
        morph.frameNo /= frame_rate / mmd_frame_rate;
        morph.m_weight = m_weight;
        return morph;
      }

    }
  }
}
