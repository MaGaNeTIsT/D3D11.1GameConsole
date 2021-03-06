#ifndef _SHADER_SPACE_TRANSFORM_HLSL
#define _SHADER_SPACE_TRANSFORM_HLSL

#include "./ShaderVariables.hlsl"
#include "./ShaderFunctions.hlsl"

#define ENGINE_CAMERA_POSITION		(_CameraWorldPosition)
#define ENGINE_MATRIX_W				(_WorldMatrix)
#define ENGINE_MATRIX_V				(_ViewMatrix)
#define ENGINE_MATRIX_P				(_ProjectionMatrix)
#define ENGINE_MATRIX_I_W			(_WorldInvMatrix)
#define ENGINE_MATRIX_I_V			(_ViewInvMatrix)
#define ENGINE_MATRIX_I_P			(_ProjectionInvMatrix)
#define ENGINE_MATRIX_I_T_W			(_WorldInvTransposeMatrix)
#define ENGINE_MATRIX_VP			(_ViewProjectionMatrix)
#define ENGINE_MATRIX_I_VP			(_ViewProjectionInvMatrix)

float3 GetCameraWorldPosition()
{
	return ENGINE_CAMERA_POSITION.xyz;
}
float3 TransformObjectToWorld(const float3 position)
{
	return mul(float4(position, 1), ENGINE_MATRIX_W).xyz;
}
float4 TransformObjectToClip(const float3 position)
{
	float3 positionWS = mul(float4(position, 1), ENGINE_MATRIX_W).xyz;
	return mul(float4(positionWS, 1), ENGINE_MATRIX_VP);
}
float3 TransformWorldToObject(const float3 position)
{
	return mul(float4(position, 1), ENGINE_MATRIX_I_W).xyz;
}
float3 TransformWorldToView(const float3 position)
{
	return mul(float4(position, 1), ENGINE_MATRIX_V).xyz;
}
float3 TransformViewToWorld(const float3 position)
{
	return mul(float4(position, 1), ENGINE_MATRIX_I_V).xyz;
}
float4 TransformViewToClip(const float3 position)
{
	return mul(float4(position, 1), ENGINE_MATRIX_P);
}
float3 TransformClipToView(const float4 position)
{
	float4 positionVS = mul(position, ENGINE_MATRIX_I_P);
	return (positionVS.xyz / positionVS.w);
}
float3 TransformClipToWorld(const float4 position)
{
	float4 positionWS = mul(position, ENGINE_MATRIX_I_VP);
	return (positionWS.xyz / positionWS.w);
}
float3 TransformObjectToWorldNormal(const float3 normal, uniform bool normalize = true)
{
	float3 normalWS = mul(normal, (float3x3)ENGINE_MATRIX_I_T_W);
	if (normalize == true)
		normalWS = SafeNormalize(normalWS);
	return normalWS;
}
float3 TransformObjectToWorldDir(const float3 dir, uniform bool normalize = true)
{
	float3 dirWS = mul(dir, (float3x3)ENGINE_MATRIX_W);
	if (normalize == true)
		dirWS = SafeNormalize(dirWS);
	return dirWS;
}
float3 TransformWorldToObjectDir(const float3 dir, uniform bool normalize = true)
{
	float3 dirOS = mul(dir, (float3x3)ENGINE_MATRIX_I_W);
	if (normalize == true)
		dirOS = SafeNormalize(dirOS);
	return dirOS;
}
float3 TransformWorldToViewDir(const float3 dir, uniform bool normalize = true)
{
	float3 dirVS = mul(dir, (float3x3)ENGINE_MATRIX_V);
	if (normalize == true)
		dirVS = SafeNormalize(dirVS);
	return dirVS;
}
float3 TransformViewToWorldDir(const float3 dir, uniform bool normalize = true)
{
	float3 dirWS = mul(dir, (float3x3)ENGINE_MATRIX_I_V);
	if (normalize == true)
		dirWS = SafeNormalize(dirWS);
	return dirWS;
}
float3x3 CreateTangentMatrix(float3 normal, float3 tangent, uniform bool reCalculateTangent = false)
{
	float3 binormal = SafeNormalize(cross(normal, tangent));
	if (reCalculateTangent == true)
		tangent = SafeNormalize(cross(binormal, normal));
	return float3x3(tangent.x, tangent.y, tangent.z,
		binormal.x, binormal.y, binormal.z,
		normal.x, normal.y, normal.z);
}
float3 TransformTangentToSpaceDir(const float3 dir, const float3x3 tangentMatrix, uniform bool normalize = false)
{
	float3 targetDir = mul(dir, tangentMatrix);
	if (normalize == true)
		targetDir = SafeNormalize(targetDir);
	return targetDir;
}

#endif