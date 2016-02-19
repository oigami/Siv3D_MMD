#include "fx_stdafx.h"



struct VS_INPUT {
  float4 pos : POSITION;
  float3 normal : NORMAL;
  float2 tex : TEXCOORD0;
  float4x4 worldMatrix : MATRIX;
  float4 diffuseColor : COLOR;
};
struct TexVertex {
  float3 pos;
  int4 idx;
  float3 blend;
  float2 tex;
};
Texture2D texVertex1 : register(t1);
SamplerState DiffuseSampler {
  Filter = MIN_MAG_LINEAR_MIP_POINT;
  AddressU = Wrap;
  AddressV = Wrap;
};

//頂点シェーダ
cbuffer BoneBuff: register(b1) {
  row_major float4x4 BoneMatrix[256];
}
TexVertex GetVertex(float2 pos) {
  TexVertex ret;
  int3 vPos = int3((int)(pos.x), (int)(pos.y), 0);
  ret.pos = texVertex1.Load(vPos).xyz * 100000;
  vPos.x += 1;
  ret.idx = texVertex1.Load(vPos) * 256;
  vPos.x += 1;
  ret.blend = texVertex1.Load(vPos).xyz;
  ret.blend.z = 0;
  vPos.x += 1;
  ret.tex = texVertex1.Load(vPos).xy;
  return ret;
}

struct VS_OUTPUT {
  float4 pos : SV_POSITION;
  float3 normal : TEXCOORD0;
  float3 worldPosition : TEXCOORD1;
  float4 color : TEXCOORD2;
  float2 tex : TEXCOORD3;
};

struct PS_OUTPUT {
  float4 color : SV_Target0;
  float  depth : SV_Target1;
  float4 normal : SV_Target2;
};


cbuffer vscbMesh0 : register(b0) {
  row_major float4x4 g_viewProjectionMatrix;
}
// 頂点シェーダ
VS_OUTPUT VS(VS_INPUT input) {
  VS_OUTPUT Out = (VS_OUTPUT)0;
  TexVertex v = GetVertex(input.tex);
  float3 w = v.blend;
  float4x4 comb = (float4x4)0;
  comb += BoneMatrix[v.idx.x] * w.x;
  comb += BoneMatrix[v.idx.y] * w.y;

  //comb += BoneMatrix[v.idx[3]] * (1.0f - w[0] - w[1] - w[2]);
  input.pos.w = 1;
  float4 pos = mul(input.pos, comb);
  //float4 pos = float4(In.pos, 1.0);
  float4 normal_head = mul(input.pos+float4(input.normal, 0), comb);
  float3 normal = normalize(normal_head.xyz - pos.xyz);
  Out.normal = normal;
  Out.pos = mul(pos, g_viewProjectionMatrix);
  Out.color = input.diffuseColor;
  Out.worldPosition = Out.pos.xyz;
  //Out.color.a=1.0;
  Out.tex = v.tex;
  return Out;
}

cbuffer pscbMesh0 : register(b0) {
  float3 g_cameraPosition;
  uint g_fogType;
  float4 g_fogParam;
  float4 g_fogColor;
}

SamplerState sampler0 : register(s0);
Texture2D texture0 : register(t0);

PS_OUTPUT PS(VS_OUTPUT input) {
  PS_OUTPUT output;
  output.color = input.color * texture0.Sample(sampler0, input.tex);
  output.depth = distance(g_cameraPosition, input.worldPosition);
  output.normal = float4(input.normal, 1);
  return output;
}

/*
// テクニック
technique BlendTech {
pass P0	{
VertexShader = compile vs_2_0 BlendVS();
PixelShader = compile ps_2_0 BlendPS(true);
}
pass P1	{
VertexShader = compile vs_2_0 BlendVS();
PixelShader = compile ps_2_0 BlendPS(false);
}
}*/
