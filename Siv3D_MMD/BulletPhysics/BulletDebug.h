/**************************************************************************//**
    @file       DebugDraw.h
    @brief      DebugDrawクラス 定義
    @author     Simplestar
    @date       2011-08-19
    @par
    [説明]
    DebugDrawクラス 定義ファイル
    @note
    Copyright c 2011  SimpleStar Game
    http://simplestar.syuriken.jp/
    *//***************************************************************************/
//#include "BulletPhysics.h"
#ifndef BULLETDEBUG_H
#define BULLETDEBUG_H

#ifndef BT_VECTOR3_H
#include"bullet/src/LinearMath/btVector3.h"
#endif
#ifndef BT_IDEBUG_DRAW__H
#include"bullet/src/LinearMath/btIDebugDraw.h"
#endif
#include<DirectXTK/PrimitiveBatch.h>
//------------------------------------------------------------------------
// Type Definitions
//------------------------------------------------------------------------


/**********************************************************************//**
    @class           DebugDraw
    @brief           デバッグ描画クラス
    @author          Simplestar
    @par     [説明]
    Bullet Physics の結果を描画します。
    本番で描画がうまく働いているか比較するために用います。
    ほか、Bullet操作のテストなどを目的に使用します。
    *//***********************************************************************/
class DebugDraw : public btIDebugDraw {
  //------------------------------------------------------------------------
  // Private Variables
  //------------------------------------------------------------------------
private:
  struct LineStruct {
    oigami::D3DVertexBuffer VertexBuffer;
    D3DInputLayout *VertexLayout;
    D3DVertexShader *VertexShader;
    D3DPixelShader *PixelShader;
    std::vector<VECTOR3> vertex;
  };
  const ICamera *m_camera;
  static const int MaxVertex = 128;
  //LineStruct line;
  int     m_debugMode;                                //!< デバッグモード
  RECT    m_fpsArea;                                  //!< FPSの表示場所
  char    m_fpsString[32];                            //!< FPSの文字列
  unsigned int m_effectIndex;                         //!< エフェクトインデックス
  bool    m_bInitialized;                             //!< 初期化フラグ

  RECT m_clientRect;                                  //!< ウィンドウの矩形
  MATRIX                  m_matView;              //!< ビュー行列
  MATRIX                  m_matProjection;        //!< プロジェクション行列
  //------------------------------------------------------------------------
  // Private Functions
  //------------------------------------------------------------------------
private:

  VECTOR4*    _GetDXVector4(const btVector3& btVec3, VECTOR4& dxVec4);
  XMCOLOR*      _GetDXColor(const btVector3& btVec3, XMCOLOR& dxColor);

  //------------------------------------------------------------------------
  // Public Functions
  //------------------------------------------------------------------------
public:

  DebugDraw(const ICamera *camera);                //!< コンストラクタ
  virtual ~DebugDraw();       //!< デストラクタ

  // 線の描画
  virtual void    drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
  // 衝突点（方向付き）の描画
  virtual void    drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);
  // 警告、エラー文の出力
  virtual void    reportErrorWarning(const char* warningString);
  // 3D文字の描画
  virtual void    draw3dText(const btVector3& location, const char* textString);
  // デバッグモードの設定
  virtual void    setDebugMode(int debugMode);
  // デバッグモードの取得
  virtual int     getDebugMode() const;

  const MATRIX&   GetViewMatrix() const;
  void                SetViewMatrix(const MATRIX& viewMatrix);
  const MATRIX&   GetProjectionMatrix() const;
  void                SetProjectionMatrix(const MATRIX& projectionMatirx);
};


#endif  // ___Bullet_HelloWorld_Bullet_HelloWorld_DebugDraw_H___