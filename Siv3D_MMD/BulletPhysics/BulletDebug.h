/**************************************************************************//**
    @file       DebugDraw.h
    @brief      DebugDraw�N���X ��`
    @author     Simplestar
    @date       2011-08-19
    @par
    [����]
    DebugDraw�N���X ��`�t�@�C��
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
    @brief           �f�o�b�O�`��N���X
    @author          Simplestar
    @par     [����]
    Bullet Physics �̌��ʂ�`�悵�܂��B
    �{�Ԃŕ`�悪���܂������Ă��邩��r���邽�߂ɗp���܂��B
    �ق��ABullet����̃e�X�g�Ȃǂ�ړI�Ɏg�p���܂��B
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
  int     m_debugMode;                                //!< �f�o�b�O���[�h
  RECT    m_fpsArea;                                  //!< FPS�̕\���ꏊ
  char    m_fpsString[32];                            //!< FPS�̕�����
  unsigned int m_effectIndex;                         //!< �G�t�F�N�g�C���f�b�N�X
  bool    m_bInitialized;                             //!< �������t���O

  RECT m_clientRect;                                  //!< �E�B���h�E�̋�`
  MATRIX                  m_matView;              //!< �r���[�s��
  MATRIX                  m_matProjection;        //!< �v���W�F�N�V�����s��
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

  DebugDraw(const ICamera *camera);                //!< �R���X�g���N�^
  virtual ~DebugDraw();       //!< �f�X�g���N�^

  // ���̕`��
  virtual void    drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
  // �Փ˓_�i�����t���j�̕`��
  virtual void    drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);
  // �x���A�G���[���̏o��
  virtual void    reportErrorWarning(const char* warningString);
  // 3D�����̕`��
  virtual void    draw3dText(const btVector3& location, const char* textString);
  // �f�o�b�O���[�h�̐ݒ�
  virtual void    setDebugMode(int debugMode);
  // �f�o�b�O���[�h�̎擾
  virtual int     getDebugMode() const;

  const MATRIX&   GetViewMatrix() const;
  void                SetViewMatrix(const MATRIX& viewMatrix);
  const MATRIX&   GetProjectionMatrix() const;
  void                SetProjectionMatrix(const MATRIX& projectionMatirx);
};


#endif  // ___Bullet_HelloWorld_Bullet_HelloWorld_DebugDraw_H___