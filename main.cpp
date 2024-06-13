#include <Novice.h>
#include <Novice.h>
#include <imgui.h>
#include "MakeMatrix.h"
#include "MatrixMath.h"
#include "Vector3Math.h"
#include "Draw.h"
#include "algorithm"

const char kWindowTitle[] = "LE2B_17_ナガイ_コハク_MT3_2_7 AABBと線の当たり判定";

bool IsCollision(OBB obb, Sphere sphere);

bool IsCollision(const AABB& aabb, const Sphere& sphere);

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	//ウィンドウサイズ
	float kWindowWidth = 1280.0f;
	float kWindowHeight = 720.0f;

	//カメラ:平行移動
	Vector3 cameraTranslate{ 0.0f,1.9f,-6.49f };

	//カメラ:回転
	Vector3 cameraRotate{ 0.26f,0.0f,0.0f };

	Vector3 rotate{ 0.0f,0.0f,0.0f };

	OBB obb{
		.center{-1.0f,0.0f,0.0f},
		.orientations =
		{
			{1.0f,0.0f,0.0f},
			{0.0f,1.0f,0.0f},
			{0.0f,0.0f,1.0f}
		},
		.size{0.5f,0.5f,0.5f}
	};

	Sphere sphere{
		.center{0.0f,0.0f,0.0f},
		.radius{0.5f}
	};

	Draw draw;

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		if (ImGui::TreeNode("Camera")) {
			ImGui::DragFloat3("Translate", &cameraTranslate.x, 0.01f);
			ImGui::DragFloat3("Rotate", &cameraRotate.x, 0.01f);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("OBB")) {
			ImGui::DragFloat3("orientations[0]", &obb.orientations[0].x, 0.01f);
			ImGui::DragFloat3("orientations[1]", &obb.orientations[1].x, 0.01f);
			ImGui::DragFloat3("orientations[2]", &obb.orientations[2].x, 0.01f);
			ImGui::DragFloat3("center", &obb.center.x, 0.01f);
			ImGui::DragFloat3("size", &obb.size.x, 0.01f);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Sphere")) {
			ImGui::DragFloat3("center", &sphere.center.x, 0.01f);
			ImGui::DragFloat("radius", &sphere.radius,0.01f);
			ImGui::TreePop();
		}

		ImGui::DragFloat3("rotate", &rotate.x, 0.01f);

		Matrix4x4 rotateMatrix = Multiply(MakeRotateXMatrix(rotate.x), Multiply(MakeRotateYMatrix(rotate.y), MakeRotateZMatrix(rotate.z)));

		obb.orientations[0].x = rotateMatrix.m[0][0];
		obb.orientations[0].y = rotateMatrix.m[0][1];
		obb.orientations[0].z = rotateMatrix.m[0][2];

		obb.orientations[1].x = rotateMatrix.m[1][0];
		obb.orientations[1].y = rotateMatrix.m[1][1];
		obb.orientations[1].z = rotateMatrix.m[1][2];

		obb.orientations[2].x = rotateMatrix.m[2][0];
		obb.orientations[2].y = rotateMatrix.m[2][1];
		obb.orientations[2].z = rotateMatrix.m[2][2];

		draw.Pipeline(cameraTranslate, cameraRotate, kWindowWidth, kWindowHeight);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		draw.DrawGrid();

		if (IsCollision(obb, sphere)) {
			draw.DrawOBB(obb, RED);
		} else {
			draw.DrawOBB(obb, WHITE);
		}

		draw.DrawSphere(sphere, WHITE);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}

bool IsCollision(OBB obb, Sphere sphere) {

	Matrix4x4 scaleMatrix = MakeScaleMatrix(Vector3{ 1.0f,1.0f,1.0f });

	Matrix4x4 rotationMatrix;
	rotationMatrix.m[0][0] = obb.orientations[0].x;
	rotationMatrix.m[0][1] = obb.orientations[0].y;
	rotationMatrix.m[0][2] = obb.orientations[0].z;
	rotationMatrix.m[0][3] = 0.0f;
	rotationMatrix.m[1][0] = obb.orientations[1].x;
	rotationMatrix.m[1][1] = obb.orientations[1].y;
	rotationMatrix.m[1][2] = obb.orientations[1].z;
	rotationMatrix.m[1][3] = 0.0f;
	rotationMatrix.m[2][0] = obb.orientations[2].x;
	rotationMatrix.m[2][1] = obb.orientations[2].y;
	rotationMatrix.m[2][2] = obb.orientations[2].z;
	rotationMatrix.m[2][3] = 0.0f;
	rotationMatrix.m[3][0] = 0.0f;
	rotationMatrix.m[3][1] = 0.0f;
	rotationMatrix.m[3][2] = 0.0f;
	rotationMatrix.m[3][3] = 1.0f;

	Matrix4x4 transformMatrix = MakeTranslateMatrix(obb.center);

	Matrix4x4 worldMatrix = Multiply(scaleMatrix, Multiply(rotationMatrix, transformMatrix));

	Matrix4x4 obbWorldMatrixInverse = Inverse(worldMatrix);

	Vector3 centerInOBBLocalSpace =
		Transform(sphere.center, obbWorldMatrixInverse);

	AABB aabbOBBLocal{
		.min{-obb.size.x,-obb.size.y,-obb.size.z},
		.max{+obb.size.x,+obb.size.y,+obb.size.z}
	};

	Sphere sphereOBBLocal{
		.center{centerInOBBLocalSpace.x,centerInOBBLocalSpace.y,centerInOBBLocalSpace.z},
		.radius{sphere.radius}
	};

	return IsCollision(aabbOBBLocal, sphereOBBLocal);
}

bool IsCollision(const AABB& aabb, const Sphere& sphere) {

	Vector3 closestPoint{
		std::clamp(sphere.center.x,aabb.min.x,aabb.max.x),
		std::clamp(sphere.center.y,aabb.min.y,aabb.max.y),
		std::clamp(sphere.center.z,aabb.min.z,aabb.max.z),
	};

	float distance = Length(Subtract(closestPoint, sphere.center));

	if (distance <= sphere.radius) {
		return true;
	}

	return false;
}