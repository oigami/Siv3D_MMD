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
  float isEdge;
};
Texture2D texVertex1 : register(t1);
SamplerState DiffuseSampler
{
  Filter = MIN_MAG_LINEAR_MIP_POINT;
  AddressU = Wrap;
  AddressV = Wrap;
};
struct VS_OUTPUT
{
  float4 pos : SV_POSITION;
  float3 worldPosition : TEXCOORD0;
  float4 color : TEXCOORD1;
  float2 tex : TEXCOORD2;
};

//-------------------------------------------------------------

cbuffer vscbMesh0 : register(b0)
{
  row_major float4x4 g_viewProjectionMatrix;
}
cbuffer BoneBuff : register(b1)
{
  float4x4 BoneMatrix[256];
}
cbuffer MorphBuff : register(b2)
{
  float4 morphWeight[1024];
}
cbuffer EdgeBuff : register(b3)
{
  float4 edgeSize;
}
TexVertex GetVertex(float2 pos, float4 vertexPos)
{
  TexVertex ret;
  int4 vPos = int4(asint(pos.x), asint(pos.y), 0, 0);
  ret.idx = asint(texVertex1.Load(vPos.xyz).xy);

  vPos.x += 1;
  ret.w = texVertex1.Load(vPos.xyz).xy;

  vPos.x += 1;
  float4 tex_n_and_isEdge = texVertex1.Load(vPos.xyz);
  ret.isEdge = tex_n_and_isEdge.w;
  ret.tex = tex_n_and_isEdge.xy;
  vPos.w = asint(tex_n_and_isEdge.z);

  ret.pos = vertexPos;
  while (vPos.w)
  {
    [unroll]
    for (int i = 16 - 1; i >= 0; i--)
    {
      vPos.xw += int2(1, -1);
      float4 data = texVertex1.Load(vPos.xyz);
      ret.pos.xyz += data.rgb * morphWeight[asint(data.a)].x;
    }
  }
  return ret;
}


VS_OUTPUT VS(VS_INPUT input)
{
  VS_OUTPUT output;
  TexVertex v = GetVertex(input.tex, input.pos);
  float4x3 comb = (float4x3) BoneMatrix[v.idx.x] * v.w.x;
  comb += (float4x3) BoneMatrix[v.idx.y] * v.w.y;
  v.pos.xyz += input.normal.xyz * 0.03 * edgeSize.x * v.isEdge;

  const float4 pos = mul(input.worldMatrix, float4(mul(v.pos, comb), v.pos.w));
  float3 normal_head = mul(v.pos + float4(input.normal, 0) , comb);

  output.pos = mul(pos, g_viewProjectionMatrix);
  output.tex = input.tex;
  output.worldPosition = pos.xyz;
  output.color = 0;
  return output;
}

//-------------------------------------------------------------
Texture2D texture0 : register(t0);
SamplerState sampler0 : register(s0);

cbuffer pscbMesh0 : register(b0)
{
  float3 g_cameraPosition;
  uint g_fogType;
  float4 g_fogParam;
  float4 g_fogColor;
}

float4 PS(VS_OUTPUT input) : SV_Target
{
  return float4(0.0, 0.0, 0.0, 1.0);
}
