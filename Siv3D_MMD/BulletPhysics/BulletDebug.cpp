/**************************************************************************//**
    @file       DebugDraw.cpp
    @brief      DebugDraw�N���X ����
    @author     Simplestar
    @date       2011-08-19
    @par
        [����]
            DebugDraw�N���X �����t�@�C��
    @note
        Copyright c 2011  SimpleStar Game
        http://simplestar.syuriken.jp/
*//***************************************************************************/

//------------------------------------------------------------------------
// File Dependencies
//------------------------------------------------------------------------



#include    "BulletDebug.h"
#include"BulletPhysics.h"
// Bullet���C�u�����̓}�l�[�W�I�v�V�����ŃR���p�C���ł��Ȃ��I
//#pragma unmanaged
//#include"bullet/src/btBulletDynamicsCommon.h"

//------------------------------------------------------------------------
// Implements
//------------------------------------------------------------------------


/**********************************************************************//**
    @brief      btVector3 �� D3DXVECTOR4
    @param[in]  btVec3  :   ����
    @param[out] dxVec4  :   �o�́i��4������w�l�͕K��1.0f�ɂȂ�܂��j
    @author     Simplestar
    @par    [����]
        Bullet �̃x�N�g�������� DirectX �̓������W�ɕϊ����܂��B
    @return     �ϊ����� D3DXVECTOR4 �ւ̃|�C���^
*//***********************************************************************/
VECTOR4* DebugDraw::_GetDXVector4(const btVector3& btVec3, VECTOR4& dxVec4) {
  dxVec4.x = btVec3.x();
  dxVec4.y = btVec3.y();
  dxVec4.z = btVec3.z();
  dxVec4.w = 1.0f;
  return &dxVec4;
}

/**********************************************************************//**
    @brief      btVector3 �� D3DXCOLOR
    @param[in]  btVec3  :   ����
    @param[in]  dxColor :   �o�́i��4������a�l�͕K��1.0f�ɂȂ�܂��j
    @author     Simplestar
    @par    [����]
        Bullet ��RGB�J���[�� D3DXCOLOR ��RGBA�ɕϊ����܂��B
    @return     �ϊ����� D3DXCOLOR �ւ̃|�C���^
*//***********************************************************************/
XMCOLOR* DebugDraw::_GetDXColor(const btVector3& btVec3, XMCOLOR& dxColor) {
  dxColor.x = btVec3.x();
  dxColor.y = btVec3.y();
  dxColor.z = btVec3.z();
  dxColor.w = 1.0f;
  return &dxColor;
}

/**********************************************************************//**
    @brief      �R���X�g���N�^
*//***********************************************************************/
DebugDraw::DebugDraw(const ICamera *camera)
  : m_debugMode(0)
  , m_effectIndex(0)
  , m_bInitialized(false)
  , m_camera(camera) {
  // FPS �̕\���̈�
    {
      m_fpsArea.left = 10;
      m_fpsArea.top = 10;
      m_fpsArea.right = 100;
      m_fpsArea.bottom = 24;
    }
    // �N���C�A���g�̈�
    {
      m_clientRect.left = 0;
      m_clientRect.top = 0;
      m_clientRect.right = 800;
      m_clientRect.bottom = 600;
    }
}

/**********************************************************************//**
    @brief      �f�X�g���N�^
*//***********************************************************************/
DebugDraw::~DebugDraw() {

}

/**********************************************************************//**
    @brief      ���̕`��
    @param[in]  from    :   �J�n�_
    @param[in]  to      :   �I�[�_
    @param[in]  color   :   ���̐F
    @author     Simplestar
    @par    [����]
        2�_�Ԃ����ԃ��C����`���܂��B
    @return     �Ȃ�
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
    @brief      �Փ˓_�i�����t���j�̕`��
    @param[in]  PointOnB    :   �Փˈʒu
    @param[in]  normalOnB   :   �Փ˕���
    @param[in]  distance    :   �Փ˂̑傫��
    @param[in]  lifeTime    :   ���C�t�^�C��
    @param[in]  color   :   �\���F
    @author     Simplestar
    @par    [����]
        �Փ˓_�̕`��A�����͔������������\���Ă���
    @return     �Ȃ�
*//***********************************************************************/
void DebugDraw::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int, const btVector3& color) {
  btVector3 to = distance * (PointOnB + normalOnB);
  drawLine(PointOnB, to, color);
}

/**********************************************************************//**
    @brief      �x���A�G���[���̏o��
    @param[in]  warningString   :   �x���A�G���[��
    @author     Simplestar
    @par    [����]
        �o�̓E�B���h�E�Ɍx�����b�Z�[�W�A�G���[���b�Z�[�W���o�͂��܂��B
    @return     �Ȃ�
*//***********************************************************************/
void DebugDraw::reportErrorWarning(const char* warningString) {
  puts(warningString);
  //wchar_t wideString[MAX_PATH];
  // SMPLDX10::PathUtility::MBToWC(warningString, wideString);
  // OutputDebugString(wideString);
}

/**********************************************************************//**
    @brief      3D�����̕`��
    @param[in]  location    :   �`��ʒu
    @param[in]  textString  :   �\�����镶����
    @author     Simplestar
    @par    [����]
        �w�肵���ʒu�ɕ����񂪕\�������
    @return     �Ȃ�
*//***********************************************************************/
void DebugDraw::draw3dText(const btVector3& location, const char* textString) {
  // �����̕`��
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
    @brief      �f�o�b�O���[�h�̐ݒ�
    @param[in]  debugMode   :   �f�o�b�O���[�h
    @author     Simplestar
    @par    [����]
        �f�o�b�O���[�h�̐ݒ�
    @return     �Ȃ�
*//***********************************************************************/
void DebugDraw::setDebugMode(int debugMode) {
  m_debugMode = debugMode;
}

/**********************************************************************//**
    @brief      �f�o�b�O���[�h�̎擾
    @author     Simplestar
    @par    [����]
        �P�Ȃ�擾
    @return     �f�o�b�O���[�h
*//***********************************************************************/
int DebugDraw::getDebugMode() const {
  return m_debugMode;
}

/**********************************************************************//**
    @brief      �r���[�s��̎擾
    @author     Simplestar
    @return     �r���[�s��
*//***********************************************************************/
const MATRIX& DebugDraw::GetViewMatrix() const {
  return m_matView;
}

/**********************************************************************//**
    @brief      �r���[�s��̐ݒ�
    @param[in]  viewMatrix  :   �ݒ肷��r���[�s��
    @author     Simplestar
    @return     �Ȃ�
*//***********************************************************************/
void DebugDraw::SetViewMatrix(const MATRIX& viewMatrix) {
  m_matView = viewMatrix;
}

/**********************************************************************//**
    @brief      �v���W�F�N�V�����s��̎擾
    @author     Simplestar
    @return     �v���W�F�N�V�����s��
*//***********************************************************************/
const MATRIX& DebugDraw::GetProjectionMatrix() const {
  return m_matProjection;
}

/**********************************************************************//**
    @brief      �v���W�F�N�V�����s��̐ݒ�
    @param[in]  projectionMatirx    :   �ݒ肷��v���W�F�N�V�����s��
    @author     Simplestar
*//***********************************************************************/
void DebugDraw::SetProjectionMatrix(const MATRIX& projectionMatirx) {
  m_matProjection = projectionMatirx;
}


