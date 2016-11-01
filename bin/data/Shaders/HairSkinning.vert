#version 400

layout (location = 0) in vec3 VertexPosition;	// ノーマル頂点位置
layout (location = 1) in vec3 VertexColor;		// ノーマル頂点カラー
layout (location = 2) in vec4 BoneIndices;		// ボーンインディシス
layout (location = 3) in vec4 BoneWeights;		// 頂点ウェイト

out vec3 Color;

uniform ivec2 BoneTextureSize; // ボーンテクスチャのサイズ

// ボーン用頂点テクスチャ
uniform sampler2D BoneTranslationTexture;
uniform sampler2D BoneRotationTexture;

// カメラ系
uniform mat4 ModelViewProjectionMatrix;

//-------------------------------------
// クォータニオンヘルパーメソッド
//-------------------------------------

// クォータニオンと平行移動から行列に変換
mat4 CreateTransformFromQuaternionTransform(vec4 quaternion, vec4 translation)
{
	vec4 q = quaternion;
	float length2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;

	float rlength2;
	if(length2 != 1.0)
	{
		rlength2 = 2.0f / length2;
	}else{
		rlength2 = 2.0f;
	}

	float x2 = rlength2 * q.x;
	float y2 = rlength2 * q.y;
	float z2 = rlength2 * q.z;

	float xx = q.x * x2;
	float xy = q.x * y2;
	float xz = q.x * z2;

	float yy = q.y * y2;
	float yz = q.y * z2;
	float zz = q.z * z2;

	float wx = q.w * x2;
	float wy = q.w * y2;
	float wz = q.w * z2;

	return mat4(
		1.0f-(yy+zz), xy+wz, xz-wy, 0,
		xy-wz, 1.0f-(xx+zz), yz+wx, 0,
		xz+wy, yz-wx, 1.0f-(xx+yy), 0,
		translation.xyz, 1
	);	
}

//mat4 CreateTransformFromQuaternionTransform(vec4 quaternion, vec4 translation)
//{
//	vec4 q = quaternion;
//	float ww = q.w * q.w - 0.5f;
//
//	vec3 v00 = vec3(ww, q.x * q.y, q.x * q.z);
//	vec3 v01 = vec3(q.x * q.x, q.w * q.z, -q.w * q.y);
//	vec3 v10 = vec3(q.x * q.y, ww, q.y * q.z);
//	vec3 v11 = vec3(-q.w * q.z, q.y * q.y, q.w * q.x);
//	vec3 v20 = vec3(q.x * q.z, q.y * q.z, ww);
//	vec3 v21 = vec3(q.w * q.y, -q.w * q.x, q.z * q.z);
//
//	return mat4(
//		2.0f * (v00 + v01), 0,
//		2.0f * (v10 + v11), 0,
//		2.0f * (v20 + v21), 0,
//		translation.xyz, 1
//	);
//}

// 複数のクォータニオン+平行移動のブレンディング処理
mat4 BlendQuaternionTransforms(
	vec4 q1, vec4 t1,
	vec4 q2, vec4 t2,
	vec4 q3, vec4 t3,
	vec4 q4, vec4 t4,
	vec4 weights)
{
	return 
		CreateTransformFromQuaternionTransform(q1, t1) * weights.x +
		CreateTransformFromQuaternionTransform(q2, t2) * weights.y +
		CreateTransformFromQuaternionTransform(q3, t3) * weights.z +
		CreateTransformFromQuaternionTransform(q4, t4) * weights.w;
}

//------------------------------------------------
// 頂点テクスチャからボーン情報の抽出 (フェッチ)
//------------------------------------------------

mat4 CreateTransformFromBoneTexture(vec4 boneIndices, vec4 boneWeights)
{
	// ボーン番号を0~1の範囲に設定
	vec2 uv = 1.0f / BoneTextureSize;
	uv.y = 0.5f;

	vec4 texCoord0 = vec4((0.5f + boneIndices.x) * uv.x, uv.y, 0, 1);
	vec4 texCoord1 = vec4((0.5f + boneIndices.y) * uv.x, uv.y, 0, 1);
	vec4 texCoord2 = vec4((0.5f + boneIndices.z) * uv.x, uv.y, 0, 1);
	vec4 texCoord3 = vec4((0.5f + boneIndices.w) * uv.x, uv.y, 0, 1);

	// テクスチャから回転部分の取り出し (フェッチ)
	vec4 q1 = texture2D(BoneRotationTexture, texCoord0.st);
	vec4 q2 = texture2D(BoneRotationTexture, texCoord1.st);
	vec4 q3 = texture2D(BoneRotationTexture, texCoord2.st);
	vec4 q4 = texture2D(BoneRotationTexture, texCoord3.st);

	// テクスチャから平行移動部分の取り出し (フェッチ)
	vec4 t1 = texture2D(BoneTranslationTexture, texCoord0.st);
	vec4 t2 = texture2D(BoneTranslationTexture, texCoord1.st);
	vec4 t3 = texture2D(BoneTranslationTexture, texCoord2.st);
	vec4 t4 = texture2D(BoneTranslationTexture, texCoord3.st);

	return BlendQuaternionTransforms(
		q1, t1,
		q2, t2,
		q3, t3,
		q4, t4,
		boneWeights);
}

void main()
{
	Color = VertexColor.rgb;

	// スキニング
	mat4 skinTransform = CreateTransformFromBoneTexture(BoneIndices, BoneWeights);
	gl_Position = ModelViewProjectionMatrix * skinTransform * vec4(VertexPosition, 1.0);
}