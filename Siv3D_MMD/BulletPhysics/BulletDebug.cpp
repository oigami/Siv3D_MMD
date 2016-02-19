/**************************************************************************//**
    @file       DebugDraw.cpp
    @brief      DebugDrawクラス 実装
    @author     Simplestar
    @date       2011-08-19
    @par
        [説明]
            DebugDrawクラス 実装ファイル
    @note
        Copyright c 2011  SimpleStar Game
        http://simplestar.syuriken.jp/
*//***************************************************************************/

//------------------------------------------------------------------------
// File Dependencies
//------------------------------------------------------------------------



#include    "BulletDebug.h"
#include"BulletPhysics.h"
// Bulletライブラリはマネージオプションでコンパイルできない！
//#pragma unmanaged
//#include"bullet/src/btBulletDynamicsCommon.h"

//------------------------------------------------------------------------
// Implements
//------------------------------------------------------------------------


/**********************************************************************//**
    @brief      btVector3 → D3DXVECTOR4
    @param[in]  btVec3  :   入力
    @param[out] dxVec4  :   出力（第4成分のw値は必ず1.0fになります）
    @author     Simplestar
    @par    [説明]
        Bullet のベクトル成分を DirectX の同次座標に変換します。
    @return     変換した D3DXVECTOR4 へのポインタ
*//***********************************************************************/
VECTOR4* DebugDraw::_GetDXVector4(const btVector3& btVec3, VECTOR4& dxVec4) {
  dxVec4.x = btVec3.x();
  dxVec4.y = btVec3.y();
  dxVec4.z = btVec3.z();
  dxVec4.w = 1.0f;
  return &dxVec4;
}

/**********************************************************************//**
    @brief      btVector3 → D3DXCOLOR
    @param[in]  btVec3  :   入力
    @param[in]  dxColor :   出力（第4成分のa値は必ず1.0fになります）
    @author     Simplestar
    @par    [説明]
        Bullet のRGBカラーを D3DXCOLOR のRGBAに変換します。
    @return     変換した D3DXCOLOR へのポインタ
*//***********************************************************************/
XMCOLOR* DebugDraw::_GetDXColor(const btVector3& btVec3, XMCOLOR& dxColor) {
  dxColor.x = btVec3.x();
  dxColor.y = btVec3.y();
  dxColor.z = btVec3.z();
  dxColor.w = 1.0f;
  return &dxColor;
}

/**********************************************************************//**
    @brief      コンストラクタ
*//***********************************************************************/
DebugDraw::DebugDraw(const ICamera *camera)
  : m_debugMode(0)
  , m_effectIndex(0)
  , m_bInitialized(false)
  , m_camera(camera) {
  // FPS の表示領域
    {
      m_fpsArea.left = 10;
      m_fpsArea.top = 10;
      m_fpsArea.right = 100;
      m_fpsArea.bottom = 24;
    }
    // クライアント領域
    {
      m_clientRect.left = 0;
      m_clientRect.top = 0;
      m_clientRect.right = 800;
      m_clientRect.bottom = 600;
    }
}

/**********************************************************************//**
    @brief      デストラクタ
*//***********************************************************************/
DebugDraw::~DebugDraw() {

}

/**********************************************************************//**
    @brief      線の描画
    @param[in]  from    :   開始点
    @param[in]  to      :   終端点
    @param[in]  color   :   線の色
    @author     Simplestar
    @par    [説明]
        2点間を結ぶラインを描きます。
    @return     なし
*//***********************************************************************/
void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
  std::array<VECTOR3, 2> vec;
  vec[0] = bullet::ConvertVectorBtToDx(from);
  vec[1] = bullet::ConvertVectorBtToDx(to);
  //line.vertex.push_back(vec[0]);
  //line.vertex.push_back(vec[1]);
  XMCOLOR xmcolor(color.x(), color.y(), color.z(), 1.0);
  //DrawLine(vec[0], vec[1],xmcolor);

  //MATRIX view, proj;
  //camera->GetMatrix(&view, &proj);
  //nt240::g_pd3dDevice->SetTransform(D3DTS_WORLD, world);
  //nt240::g_pd3dDevice->SetTransform(D3DTS_VIEW, &view);
  //nt240::g_pd3dDevice->SetTransform(D3DTS_PROJECTION, &proj);
//	nt240::g_pd3dDevice->SetFVF(D3DFVF_XYZ);
  //nt240::g_pd3dDevice->SetRenderState(D3DRS_LIGHTING, FALSE);
  //nt240::g_pd3dDevice->SetTexture(0, 0);
  //nt240::g_pd3dDevice->DrawPrimitiveUP(D3DPT_LINELIST, 1, vec, sizeof(VECTOR3));
  //nt240::g_pd3dDevice->SetRenderState(D3DRS_LIGHTING, TRUE);
}

/**********************************************************************//**
    @brief      衝突点（方向付き）の描画
    @param[in]  PointOnB    :   衝突位置
    @param[in]  normalOnB   :   衝突方向
    @param[in]  distance    :   衝突の大きさ
    @param[in]  lifeTime    :   ライフタイム
    @param[in]  color   :   表示色
    @author     Simplestar
    @par    [説明]
        衝突点の描画、方向は反発する向きを表している
    @return     なし
*//***********************************************************************/
void DebugDraw::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int, const btVector3& color) {
  btVector3 to = distance * (PointOnB + normalOnB);
  drawLine(PointOnB, to, color);
}

/**********************************************************************//**
    @brief      警告、エラー文の出力
    @param[in]  warningString   :   警告、エラー文
    @author     Simplestar
    @par    [説明]
        出力ウィンドウに警告メッセージ、エラーメッセージを出力します。
    @return     なし
*//***********************************************************************/
void DebugDraw::reportErrorWarning(const char* warningString) {
  puts(warningString);
  //wchar_t wideString[MAX_PATH];
  // SMPLDX10::PathUtility::MBToWC(warningString, wideString);
  // OutputDebugString(wideString);
}

/**********************************************************************//**
    @brief      3D文字の描画
    @param[in]  location    :   描画位置
    @param[in]  textString  :   表示する文字列
    @author     Simplestar
    @par    [説明]
        指定した位置に文字列が表示される
    @return     なし
*//***********************************************************************/
void DebugDraw::draw3dText(const btVector3& location, const char* textString) {
  // 文字の描画
  /* if (m_pFont)
  {
      MATRIX matViewProjection;
      XMMatrixMultiply( &matViewProjection, &m_matView, &m_matProjection );
      VECTOR3 dxLocation;
      {
          dxLocation.x = location.x();
          dxLocation.y = location.y();
          dxLocation.z = location.z();
      }
      D3DXVec3TransformCoord( &dxLocation, &dxLocation, &matViewProjection );
      RECT stringArea;
      {
          stringArea.left = static_cast<LONG>((1.0f + dxLocation.x) * m_clientRect.right/2.0f);
          stringArea.top = static_cast<LONG>((1.0f - dxLocation.y) * m_clientRect.bottom/2.0f);
          stringArea.right = stringArea.left + 100;
          stringArea.bottom = stringArea.top + 50;
      }
      m_pFont->DrawTextA(NULL, textString, -1, &stringArea, DT_NOCLIP, D3DXCOLOR(1.0f, 1.0f, 1.0f, 1.0f));
  }*/
}

/**********************************************************************//**
    @brief      デバッグモードの設定
    @param[in]  debugMode   :   デバッグモード
    @author     Simplestar
    @par    [説明]
        デバッグモードの設定
    @return     なし
*//***********************************************************************/
void DebugDraw::setDebugMode(int debugMode) {
  m_debugMode = debugMode;
}

/**********************************************************************//**
    @brief      デバッグモードの取得
    @author     Simplestar
    @par    [説明]
        単なる取得
    @return     デバッグモード
*//***********************************************************************/
int DebugDraw::getDebugMode() const {
  return m_debugMode;
}

/**********************************************************************//**
    @brief      ビュー行列の取得
    @author     Simplestar
    @return     ビュー行列
*//***********************************************************************/
const MATRIX& DebugDraw::GetViewMatrix() const {
  return m_matView;
}

/**********************************************************************//**
    @brief      ビュー行列の設定
    @param[in]  viewMatrix  :   設定するビュー行列
    @author     Simplestar
    @return     なし
*//***********************************************************************/
void DebugDraw::SetViewMatrix(const MATRIX& viewMatrix) {
  m_matView = viewMatrix;
}

/**********************************************************************//**
    @brief      プロジェクション行列の取得
    @author     Simplestar
    @return     プロジェクション行列
*//***********************************************************************/
const MATRIX& DebugDraw::GetProjectionMatrix() const {
  return m_matProjection;
}

/**********************************************************************//**
    @brief      プロジェクション行列の設定
    @param[in]  projectionMatirx    :   設定するプロジェクション行列
    @author     Simplestar
*//***********************************************************************/
void DebugDraw::SetProjectionMatrix(const MATRIX& projectionMatirx) {
  m_matProjection = projectionMatirx;
}


