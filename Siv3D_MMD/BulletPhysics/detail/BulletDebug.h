#ifndef INCLUDE_BULLET_DEBUG_H
#define INCLUDE_BULLET_DEBUG_H

#include <LinearMath/btIDebugDraw.h>

namespace s3d_bullet {

  class DebugDraw : public btIDebugDraw {
    int  m_debugMode;

  public:

    DebugDraw();
    virtual ~DebugDraw();

    /// <summary>
    /// 線の描画
    /// </summary>
    /// <param name="from"> 開始点 </param>
    /// <param name="to"> 終了点 </param>
    /// <param name="color"> 色 </param>
    virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);


    /// <summary>
    /// 球の描画
    /// </summary>
    /// <param name="radius"> 半径 </param>
    /// <param name="transform"> 座標 </param>
    /// <param name="color"> 色 </param>
    virtual void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color);

    /// <summary>
    /// 箱の描画
    /// </summary>
    /// <param name="bbMin"> 開始点 </param>
    /// <param name="bbMax"> 終了点 </param>
    /// <param name="trans"> ? </param>
    /// <param name="color"> 色 </param>
    //virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btTransform& trans, const btVector3& color);

    /// <summary>
    /// 箱の描画
    /// </summary>
    /// <param name="bbMin"> 開始点 </param>
    /// <param name="bbMax"> 終了点 </param>
    /// <param name="color"> 色 </param>
    virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btVector3& color);

    /// <summary>
    /// カプセルの描画
    /// </summary>
    /// <param name="radius"> 半径 </param>
    /// <param name="halfHeight"> 高さの半分 </param>
    /// <param name="upAxis"> 上の向き(1 or -1?) </param>
    /// <param name="transform"> 位置と回転 </param>
    /// <param name="color"> 色 </param>
    virtual void drawCapsule(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color);

    /// <summary>
    /// 円柱の描画
    /// </summary>
    /// <param name="radius"> 半径 </param>
    /// <param name="halfHeight"> 高さの半分 </param>
    /// <param name="upAxis"> 上の向き(1 or -1?) </param>
    /// <param name="transform"> 位置と回転 </param>
    /// <param name="color"> 色 </param>
    virtual void drawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color);

    /// <summary>
    /// 衝突点（方向付き）の描画
    /// </summary>
    /// <param name="PointOnB"> 衝突位置 </param>
    /// <param name="normalOnB"> 衝突方向 </param>
    /// <param name="distance"> 衝突の大きさ </param>
    /// <param name="lifeTime"> 表示時間？ </param>
    /// <param name="color"> 色 </param>
    virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);

    /// <summary>
    /// 警告、エラー文の出力
    /// </summary>
    /// <param name="warningString">表示する文字列</param>
    virtual void reportErrorWarning(const char* warningString);

    /// <summary>
    /// 3D文字の描画
    /// </summary>
    /// <param name="location"> 描画位置 </param>
    /// <param name="textString"> 表示する文字列 </param>
    virtual void draw3dText(const btVector3& location, const char* textString);

    /// <summary>
    /// デバッグモードの設定
    /// </summary>
    /// <param name="debugMode"> デバッグモード </param>
    virtual void setDebugMode(int debugMode);

    /// <summary>
    /// デバッグモードの取得
    /// </summary>
    /// <returns> デバッグモード </returns>
    virtual int getDebugMode() const;

  };

}

#endif  // ___Bullet_HelloWorld_Bullet_HelloWorld_DebugDraw_H___}
