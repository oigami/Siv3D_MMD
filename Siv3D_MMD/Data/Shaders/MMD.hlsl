
struct VS_INPUT
{
  float4 pos : POSITION;
  float3 normal : NORMAL;
  float2 tex : TEXCOORD0;
  float4x4 worldMatrix : MATRIX;
  float4 diffuseColor : COLOR;
};
struct TexVertex
{
  float4 pos;
  float2 w;
  float2 tex;
  int2 idx;
};
Texture2D texVertex1 : register(t1);
SamplerState DiffuseSampler
{
  Filter = MIN_MAG_LINEAR_MIP_POINT;
  AddressU = Wrap;
  AddressV = Wrap;
};

//頂点シェーダ
cbuffer BoneBuff: register(b1)
{
  float4x4 BoneMatrix[256];
}
cbuffer MorphBuff : register(b2)
{
  float4 morphWeight[1024];
}
TexVertex GetVertex(float2 pos, float4 vertexPos)
{
  TexVertex ret;
  int4 vPos = int4(asint(pos.x), asint(pos.y), 0, 0);
  ret.idx = asint(texVertex1.Load(vPos.xyz).xy);

  vPos.x += 1;
  ret.w = texVertex1.Load(vPos.xyz).xy;

  vPos.x += 1;
  float3 tex_and_n = texVertex1.Load(vPos.xyz).xyz;
  ret.tex = tex_and_n.xy;
  vPos.w = asint(tex_and_n.z);

  ret.pos = vertexPos;
  while ( vPos.w )
  {
    [unroll] for ( int i = 16 - 1; i >= 0; i-- )
    {
      vPos.xw += int2(1, -1);
      float4 data = texVertex1.Load(vPos.xyz);
      ret.pos.xyz += data.rgb * morphWeight[asint(data.a)].x;
    }
  }
  return ret;
}

struct VS_OUTPUT
{
  float4 pos : SV_POSITION;
  float3 normal : TEXCOORD0;
  float3 worldPosition : TEXCOORD1;
  float4 color : TEXCOORD2;
  float2 tex : TEXCOORD3;
};

struct PS_OUTPUT
{
  float4 color : SV_Target0;
  float  depth : SV_Target1;
  float4 normal : SV_Target2;
};

cbuffer vscbMesh0 : register(b0)
{
  row_major float4x4 g_viewProjectionMatrix;
}
// 頂点シェーダ
VS_OUTPUT VS(VS_INPUT input)
{
  TexVertex v = GetVertex(input.tex, input.pos);
  float4x3 comb = (float4x3)BoneMatrix[v.idx.x] * v.w.x;
  comb += (float4x3)BoneMatrix[v.idx.y] * v.w.y;

  //comb += BoneMatrix[v.idx[3]] * (1.0f - w[0] - w[1] - w[2]);
  const float3 normal_head = mul(float4(v.pos.xyz + input.normal, v.pos.w), comb);
  const float4 pos = float4(mul(v.pos, comb), v.pos.w);

  VS_OUTPUT Out;
  Out.pos = mul(pos, g_viewProjectionMatrix);
  Out.normal = normalize(normal_head.xyz - pos.xyz);
  Out.color = input.diffuseColor;
  Out.worldPosition = pos.xyz;
  Out.tex = v.tex;
  return Out;
}
