#include <MMD/mmd_motion.h>
#include <MMD/vmd_writer.h>

namespace s3d_mmd
{
  namespace mmd
  {
    MMDMotion::MMDMotion(const VMDReader & reader)
    {

      for ( const auto& bone : reader.getBoneFrames() )
      {
        key_frame::BoneFrame keyFrame;
        String bonename = keyFrame.set(bone);
        m_keyFrames[bonename].add(std::move(keyFrame));
      }

      m_modelName = reader.getModelName();
      for ( auto& morph : reader.getMorphFrames() )
      {
        key_frame::MorphFrame morphFrame;
        String name = morphFrame.set(morph);
        m_morphs[name].push_back(morphFrame);
      }

      m_cameras = reader.getCameraFrames();

      m_lights = reader.getLightFrames();

      m_selfShadows = reader.getSelfShadowFrames();

      m_showIks = reader.getShowIk();

    }

    const BoneFrames& MMDMotion::getBoneFrames(const String & bone_name) const
    {
      static const Array<BoneFrame> notFindBone;
      auto it = m_keyFrames.find(bone_name);
      if ( it == m_keyFrames.end() )return notFindBone;
      return it->second;
    }

    void MMDMotion::sort()
    {
      for ( auto& i : m_morphs )
      {
        std::sort(i.second.begin(), i.second.end());
      }
    }

    /// <summary>
    /// 最後のフレームの時間を返す
    /// </summary>
    /// <remarks>
    /// 計算量: O(N)
    /// <para> n := Boneの数 </para>
    /// </remarks>
    /// <returns></returns>

    int MMDMotion::getLastFrame() const
    {
      int lastFrame = 0;
      for ( auto& i : m_keyFrames )
      {
        lastFrame = std::max(lastFrame, i.second.lastFrameNo());
      }
      return lastFrame;
    }

    bool MMDMotion::saveVMD(const FilePath & filename)
    {
      VMDWriter writer(filename);
      if ( !writer.isOpened() ) return false;
      for ( auto& i : getBoneFrames() )
      {
        for ( auto& j : i.second.createFrames() )
        {
          writer.boneFrame.push_back(j.Convert(i.first));
        }
      }

      writer.modelName = m_modelName;
      for ( auto& i : m_morphs )
      {
        for ( auto& j : i.second )
          writer.morphFrame.push_back(j.convert(i.first));

      }
      writer.cameraFrame = m_cameras;
      writer.lightFrame = m_lights;
      writer.selfShadowFrame = m_selfShadows;
      writer.showIkFrame = m_showIks;
      writer.close();

      return true;
    }


    /// <summary>
    /// frameNo以下のフレームとframeNoより後のフレームを返す
    /// </summary>
    /// <param name="frameNo"></param>
    /// <returns></returns>

    void BoneFrames::sort()
    {
      if ( isSorted ) return;

      std::sort(m_frames.begin(), m_frames.end());
      isSorted = true;
    }

    void BoneFrames::push(const FrameData & data)
    {
      m_frames.push_back(data);
      isSorted = false;
    }

    std::pair<Array<BoneFrames::FrameData>::iterator, bool> BoneFrames::getFrameImpl(int frameNo)
    {
      FrameData val;
      val.frameNo = frameNo;
      auto first = std::lower_bound(m_frames.begin(), m_frames.end(), val);
      if ( first != m_frames.end() && first->frameNo == frameNo )
        return{ first,true };

      return{ first, false };
    }

    BoneFrames::BoneFrames() :isSorted(true) {}

    BoneFrames::BoneFrames(Array<FrameData> data) : m_frames(std::move(data)) {}

    int BoneFrames::lastFrameNo()const
    {
      int lastFrame = 0;
      for ( auto& i : m_frames )
        lastFrame = std::max(lastFrame, i.frameNo);

      return lastFrame;
    }

    Optional<BoneFrames::FrameData> BoneFrames::add(const FrameData & data)
    {
      auto preFrame = getFrameImpl(data.frameNo);
      if ( preFrame.second )
      {
        auto res = *preFrame.first;
        *preFrame.first = data;
        return res;
      }
      push(data);
      return none;
    }

    bool BoneFrames::remove(int frameNo)
    {
      auto p = getFrameImpl(frameNo);
      if ( p.second )
        m_frames.erase(p.first);

      return p.second;
    }

    std::pair<Optional<const BoneFrames::FrameData&>, Optional<const BoneFrames::FrameData&>> BoneFrames::getSection(int frameNo)
    {
      if ( size() == 0 ) return{ none, none };
      sort();

      // frameNoより後のデータを探索
      FrameData val;
      val.frameNo = frameNo;
      auto it = std::upper_bound(m_frames.begin(), m_frames.end(), val);

      if ( it == m_frames.end() )
        return{ *--it, none };

      // 一つ前に戻れるかチェック
      if ( it != m_frames.begin() )
      {
        auto second = it;
        return{ *--it, *second };    // もしフレームの後のデータがある場合

      }
      else
      {
        return{ none, *it };
      }
    }

    Optional<const BoneFrame&> BoneFrames::getFrame(int frameNo)
    {
      auto p = getFrameImpl(frameNo);
      if ( p.second )
        return *p.first;
      return none;
    }

    Array<BoneFrames::FrameData>& BoneFrames::frames()
    {
      sort();
      return m_frames;
    }

    Array<BoneFrames::FrameData> BoneFrames::createFrames() const
    {
      Array<FrameData> res = m_frames;
      std::sort(res.begin(), res.end());
      return std::move(res);
    }

  }
}
