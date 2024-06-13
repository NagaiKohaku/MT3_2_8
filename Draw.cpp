#define PI 3.14f
#include "Draw.h"
#include "Novice.h"
#include "Vector3Math.h"
#include "MatrixMath.h"
#include "MakeMatrix.h"

void Draw::Pipeline(const Vector3& cameraTranslate, const Vector3& cameraRotate, float kWindowWidth, float kWindowHeight) {

	cameraMatrix_ = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);

	viewMatrix_ = Inverse(cameraMatrix_);

	projectionMatrix_ = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);

	viewProjectionMatrix_ = Multiply(viewMatrix_, projectionMatrix_);

	viewPortMatrix_ = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

}

void Draw::DrawGrid() {

	//Gridの半分の幅
	const float kGridHalfWidth = 2.0f;

	//分割数
	const uint32_t kSubdivision = 10;

	//1つ分の長さ
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);

	Matrix4x4 worldMatrix = MakeAffineMatrix(
		Vector3(1, 1, 1),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0)
	);

	Matrix4x4 worldViewProjectionMatrix = Multiply(worldMatrix, viewProjectionMatrix_);

	//奥から手前への線を順順で引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {

		float start = -(kSubdivision / 2.0f) * kGridEvery;
		float end = (kSubdivision / 2.0f) * kGridEvery;

		Vector3 startNdcVertex = Transform(Vector3(((kSubdivision / 2.0f) - xIndex) * kGridEvery, 0, start), worldViewProjectionMatrix);

		Vector3 startScreenVertex = Transform(startNdcVertex, viewPortMatrix_);

		Vector3 endNdcVertex = Transform(Vector3(((kSubdivision / 2.0f) - xIndex) * kGridEvery, 0, end), worldViewProjectionMatrix);

		Vector3 endScreenVertex = Transform(endNdcVertex, viewPortMatrix_);

		if (xIndex == kSubdivision / 2.0f) {
			Novice::DrawLine(
				int(startScreenVertex.x),
				int(startScreenVertex.y),
				int(endScreenVertex.x),
				int(endScreenVertex.y),
				0x000000FF);
		} else {
			Novice::DrawLine(
				int(startScreenVertex.x),
				int(startScreenVertex.y),
				int(endScreenVertex.x),
				int(endScreenVertex.y),
				0xAAAAAAFF);
		}

	}

	//左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {

		float start = -(kSubdivision / 2.0f) * kGridEvery;
		float end = (kSubdivision / 2.0f) * kGridEvery;

		Vector3 startNdcVertex = Transform(Vector3(start, 0, ((kSubdivision / 2.0f) - zIndex) * kGridEvery), worldViewProjectionMatrix);

		Vector3 startScreenVertex = Transform(startNdcVertex, viewPortMatrix_);

		Vector3 endNdcVertex = Transform(Vector3(end, 0, ((kSubdivision / 2.0f) - zIndex) * kGridEvery), worldViewProjectionMatrix);

		Vector3 endScreenVertex = Transform(endNdcVertex, viewPortMatrix_);

		if (zIndex == kSubdivision / 2.0f) {
			Novice::DrawLine(
				int(startScreenVertex.x),
				int(startScreenVertex.y),
				int(endScreenVertex.x),
				int(endScreenVertex.y),
				0x000000FF);
		} else {
			Novice::DrawLine(
				int(startScreenVertex.x),
				int(startScreenVertex.y),
				int(endScreenVertex.x),
				int(endScreenVertex.y),
				0xAAAAAAFF);
		}
	}
}

void Draw::DrawSphere(Sphere sphere, uint32_t color) {

	const uint32_t kSubDivision = 12;

	//経度1つ分の角度(φ)
	const float kLonEvery = (2.0f * PI) / kSubDivision;

	//緯度1つ分の角度(θ)
	const float kLatEvery = PI / kSubDivision;

	Matrix4x4 worldMatrix = MakeAffineMatrix(
		Vector3(1, 1, 1),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0)
	);

	Matrix4x4 worldViewProjectionMatrix = Multiply(worldMatrix, viewProjectionMatrix_);

	//緯度(θ)の方向に分割　-π/2 ~ π/2
	for (uint32_t latIndex = 0; latIndex < kSubDivision; ++latIndex) {

		float lat = -PI / 2.0f + kLatEvery * latIndex;

		//経度(φ)の方向に分割 0 ~ 2π
		for (uint32_t lonIndex = 0; lonIndex < kSubDivision; ++lonIndex) {

			float lon = lonIndex * kLonEvery;

			//world座標系でのa,b,cを決める
			Vector3 a, b, c;

			a = { sphere.radius * cosf(lat) * cosf(lon)             , sphere.radius * sinf(lat)             , sphere.radius * cosf(lat) * sinf(lon) };
			b = { sphere.radius * cosf(lat + kLatEvery) * cosf(lon) , sphere.radius * sinf(lat + kLatEvery) , sphere.radius * cosf(lat + kLatEvery) * sinf(lon) };
			c = { sphere.radius * cosf(lat) * cosf(lon + kLonEvery) , sphere.radius * sinf(lat)             , sphere.radius * cosf(lat) * sinf(lon + kLonEvery) };

			a.x += sphere.center.x;
			a.y += sphere.center.y;
			a.z += sphere.center.z;

			b.x += sphere.center.x;
			b.y += sphere.center.y;
			b.z += sphere.center.z;

			c.x += sphere.center.x;
			c.y += sphere.center.y;
			c.z += sphere.center.z;

			Vector3 ANdcVertex = Transform(a, worldViewProjectionMatrix);
			Vector3 BNdcVertex = Transform(b, worldViewProjectionMatrix);
			Vector3 CNdcVertex = Transform(c, worldViewProjectionMatrix);

			Vector3 AScreenVertex = Transform(ANdcVertex, viewPortMatrix_);
			Vector3 BScreenVertex = Transform(BNdcVertex, viewPortMatrix_);
			Vector3 CScreenVertex = Transform(CNdcVertex, viewPortMatrix_);

			Novice::DrawLine(
				int(AScreenVertex.x),
				int(AScreenVertex.y),
				int(BScreenVertex.x),
				int(BScreenVertex.y),
				color
			);

			Novice::DrawLine(
				int(AScreenVertex.x),
				int(AScreenVertex.y),
				int(CScreenVertex.x),
				int(CScreenVertex.y),
				color
			);

		}
	}
}

void Draw::DrawLine(Vector3 start, Vector3 end, uint32_t color) {

	Matrix4x4 worldMatrix = MakeAffineMatrix(
		Vector3(1, 1, 1),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0)
	);

	Matrix4x4 worldViewProjectionMatrix = Multiply(worldMatrix, viewProjectionMatrix_);

	Vector3 startNdcVertex = Transform(start, worldViewProjectionMatrix);
	Vector3 startScreenVertex = Transform(startNdcVertex, viewPortMatrix_);

	Vector3 endNdcVertex = Transform(Add(start, end), worldViewProjectionMatrix);
	Vector3 endScreenVertex = Transform(endNdcVertex, viewPortMatrix_);

	Novice::DrawLine(
		int(startScreenVertex.x),
		int(startScreenVertex.y),
		int(endScreenVertex.x),
		int(endScreenVertex.y),
		color
	);
}

void Draw::DrawPlane(Plane plane, uint32_t color) {

	Vector3 center = Multiply(plane.distance, plane.normal);

	Vector3 perpendicular[4];
	perpendicular[0] = Normalize(Perpendicular(plane.normal));
	perpendicular[1] = { -perpendicular[0].x,-perpendicular[0].y,-perpendicular[0].z };
	perpendicular[2] = Cross(plane.normal, perpendicular[0]);
	perpendicular[3] = { -perpendicular[2].x,-perpendicular[2].y,-perpendicular[2].z };

	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = Multiply(2.0f, perpendicular[index]);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix_), viewPortMatrix_);
	}

	Novice::DrawLine(
		int(points[0].x),
		int(points[0].y),
		int(points[3].x),
		int(points[3].y),
		color
	);

	Novice::DrawLine(
		int(points[3].x),
		int(points[3].y),
		int(points[1].x),
		int(points[1].y),
		color
	);

	Novice::DrawLine(
		int(points[1].x),
		int(points[1].y),
		int(points[2].x),
		int(points[2].y),
		color
	);

	Novice::DrawLine(
		int(points[2].x),
		int(points[2].y),
		int(points[0].x),
		int(points[0].y),
		color
	);
}

void Draw::DrawTriangle(Triangle triangle, uint32_t color) {

	Vector3 worldVertex[3];

	for (int i = 0; i < 3; i++) {
		worldVertex[i] = Transform(Transform(triangle.vertices[i], viewProjectionMatrix_), viewPortMatrix_);
	}

	Novice::DrawLine(
		int(worldVertex[0].x),
		int(worldVertex[0].y),
		int(worldVertex[1].x),
		int(worldVertex[1].y),
		color
	);

	Novice::DrawLine(
		int(worldVertex[1].x),
		int(worldVertex[1].y),
		int(worldVertex[2].x),
		int(worldVertex[2].y),
		color
	);

	Novice::DrawLine(
		int(worldVertex[2].x),
		int(worldVertex[2].y),
		int(worldVertex[0].x),
		int(worldVertex[0].y),
		color
	);

}
void Draw::DrawAABB(const AABB& aabb, uint32_t color) {

	Vector3 worldVertex[8];

	worldVertex[0].x = aabb.min.x;
	worldVertex[0].y = aabb.min.y;
	worldVertex[0].z = aabb.min.z;

	worldVertex[1].x = aabb.max.x;
	worldVertex[1].y = aabb.min.y;
	worldVertex[1].z = aabb.min.z;

	worldVertex[2].x = aabb.min.x;
	worldVertex[2].y = aabb.max.y;
	worldVertex[2].z = aabb.min.z;

	worldVertex[3].x = aabb.min.x;
	worldVertex[3].y = aabb.min.y;
	worldVertex[3].z = aabb.max.z;

	worldVertex[4].x = aabb.max.x;
	worldVertex[4].y = aabb.max.y;
	worldVertex[4].z = aabb.min.z;

	worldVertex[5].x = aabb.max.x;
	worldVertex[5].y = aabb.min.y;
	worldVertex[5].z = aabb.max.z;

	worldVertex[6].x = aabb.min.x;
	worldVertex[6].y = aabb.max.y;
	worldVertex[6].z = aabb.max.z;

	worldVertex[7].x = aabb.max.x;
	worldVertex[7].y = aabb.max.y;
	worldVertex[7].z = aabb.max.z;

	for (int i = 0; i < 8; i++) {
		worldVertex[i] = Transform(Transform(worldVertex[i], viewProjectionMatrix_), viewPortMatrix_);
	}

	//左下前から右下前
	Novice::DrawLine(
		static_cast<int>(worldVertex[0].x),
		static_cast<int>(worldVertex[0].y),
		static_cast<int>(worldVertex[1].x),
		static_cast<int>(worldVertex[1].y),
		color
	);

	//左下前から左上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[0].x),
		static_cast<int>(worldVertex[0].y),
		static_cast<int>(worldVertex[2].x),
		static_cast<int>(worldVertex[2].y),
		color
	);

	//左下前から左下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[0].x),
		static_cast<int>(worldVertex[0].y),
		static_cast<int>(worldVertex[3].x),
		static_cast<int>(worldVertex[3].y),
		color
	);

	//右上奥から左上奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[7].x),
		static_cast<int>(worldVertex[7].y),
		static_cast<int>(worldVertex[6].x),
		static_cast<int>(worldVertex[6].y),
		color
	);

	//右上奥から右下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[7].x),
		static_cast<int>(worldVertex[7].y),
		static_cast<int>(worldVertex[5].x),
		static_cast<int>(worldVertex[5].y),
		color
	);

	//右上奥から右上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[7].x),
		static_cast<int>(worldVertex[7].y),
		static_cast<int>(worldVertex[4].x),
		static_cast<int>(worldVertex[4].y),
		color
	);

	//左上前から左上奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[2].x),
		static_cast<int>(worldVertex[2].y),
		static_cast<int>(worldVertex[6].x),
		static_cast<int>(worldVertex[6].y),
		color
	);

	//左上前から右上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[2].x),
		static_cast<int>(worldVertex[2].y),
		static_cast<int>(worldVertex[4].x),
		static_cast<int>(worldVertex[4].y),
		color
	);

	//左下奥から左上奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[3].x),
		static_cast<int>(worldVertex[3].y),
		static_cast<int>(worldVertex[6].x),
		static_cast<int>(worldVertex[6].y),
		color
	);

	//左下奥から右下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[3].x),
		static_cast<int>(worldVertex[3].y),
		static_cast<int>(worldVertex[5].x),
		static_cast<int>(worldVertex[5].y),
		color
	);

	//右下前から右下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[1].x),
		static_cast<int>(worldVertex[1].y),
		static_cast<int>(worldVertex[5].x),
		static_cast<int>(worldVertex[5].y),
		color
	);

	//右下前から右上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[1].x),
		static_cast<int>(worldVertex[1].y),
		static_cast<int>(worldVertex[4].x),
		static_cast<int>(worldVertex[4].y),
		color
	);

}

void Draw::DrawOBB(const OBB& obb, uint32_t color) {

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

	Vector3 worldVertex[8];

	worldVertex[0].x = -obb.size.x;
	worldVertex[0].y = -obb.size.y;
	worldVertex[0].z = -obb.size.z;

	worldVertex[1].x = +obb.size.x;
	worldVertex[1].y = -obb.size.y;
	worldVertex[1].z = -obb.size.z;

	worldVertex[2].x = -obb.size.x;
	worldVertex[2].y = +obb.size.y;
	worldVertex[2].z = -obb.size.z;

	worldVertex[3].x = -obb.size.x;
	worldVertex[3].y = -obb.size.y;
	worldVertex[3].z = +obb.size.z;

	worldVertex[4].x = +obb.size.x;
	worldVertex[4].y = +obb.size.y;
	worldVertex[4].z = -obb.size.z;

	worldVertex[5].x = +obb.size.x;
	worldVertex[5].y = -obb.size.y;
	worldVertex[5].z = +obb.size.z;

	worldVertex[6].x = -obb.size.x;
	worldVertex[6].y = +obb.size.y;
	worldVertex[6].z = +obb.size.z;

	worldVertex[7].x = +obb.size.x;
	worldVertex[7].y = +obb.size.y;
	worldVertex[7].z = +obb.size.z;

	for (int i = 0; i < 8; i++) {

		worldVertex[i] = Transform(Transform(Transform(worldVertex[i], worldMatrix), viewProjectionMatrix_), viewPortMatrix_);
	}

	//左下前から右下前
	Novice::DrawLine(
		static_cast<int>(worldVertex[0].x),
		static_cast<int>(worldVertex[0].y),
		static_cast<int>(worldVertex[1].x),
		static_cast<int>(worldVertex[1].y),
		color
	);

	//左下前から左上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[0].x),
		static_cast<int>(worldVertex[0].y),
		static_cast<int>(worldVertex[2].x),
		static_cast<int>(worldVertex[2].y),
		color
	);

	//左下前から左下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[0].x),
		static_cast<int>(worldVertex[0].y),
		static_cast<int>(worldVertex[3].x),
		static_cast<int>(worldVertex[3].y),
		color
	);

	//右上奥から左上奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[7].x),
		static_cast<int>(worldVertex[7].y),
		static_cast<int>(worldVertex[6].x),
		static_cast<int>(worldVertex[6].y),
		color
	);

	//右上奥から右下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[7].x),
		static_cast<int>(worldVertex[7].y),
		static_cast<int>(worldVertex[5].x),
		static_cast<int>(worldVertex[5].y),
		color
	);

	//右上奥から右上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[7].x),
		static_cast<int>(worldVertex[7].y),
		static_cast<int>(worldVertex[4].x),
		static_cast<int>(worldVertex[4].y),
		color
	);

	//左上前から左上奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[2].x),
		static_cast<int>(worldVertex[2].y),
		static_cast<int>(worldVertex[6].x),
		static_cast<int>(worldVertex[6].y),
		color
	);

	//左上前から右上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[2].x),
		static_cast<int>(worldVertex[2].y),
		static_cast<int>(worldVertex[4].x),
		static_cast<int>(worldVertex[4].y),
		color
	);

	//左下奥から左上奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[3].x),
		static_cast<int>(worldVertex[3].y),
		static_cast<int>(worldVertex[6].x),
		static_cast<int>(worldVertex[6].y),
		color
	);

	//左下奥から右下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[3].x),
		static_cast<int>(worldVertex[3].y),
		static_cast<int>(worldVertex[5].x),
		static_cast<int>(worldVertex[5].y),
		color
	);

	//右下前から右下奥
	Novice::DrawLine(
		static_cast<int>(worldVertex[1].x),
		static_cast<int>(worldVertex[1].y),
		static_cast<int>(worldVertex[5].x),
		static_cast<int>(worldVertex[5].y),
		color
	);

	//右下前から右上前
	Novice::DrawLine(
		static_cast<int>(worldVertex[1].x),
		static_cast<int>(worldVertex[1].y),
		static_cast<int>(worldVertex[4].x),
		static_cast<int>(worldVertex[4].y),
		color
	);

}