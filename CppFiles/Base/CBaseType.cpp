#include "../../Headers/Base/CBaseType.h"

namespace CustomType
{
	Matrix4x4 Matrix4x4::m_Identity = Matrix4x4::GetIdentity();
	Matrix4x4::Matrix4x4()
	{
		(*this) = Matrix4x4::m_Identity;
	}
	Matrix4x4::Matrix4x4(const Matrix4x4& m)
	{
		(*this) = m;
	}
	Matrix4x4::Matrix4x4(DirectX::CXMMATRIX m)
	{
		this->SetXMMATRIX(m);
	}
	Matrix4x4::Matrix4x4(const Quaternion& v)
	{
		this->SetXMMATRIX(v.GetXMMATRIX());
	}
	Matrix4x4::Matrix4x4(const Vector3& t, const Quaternion& r)
	{
		this->SetXMMATRIX(DirectX::XMMatrixMultiply(r.GetXMMATRIX(), DirectX::XMMatrixTranslationFromVector(t.GetXMVECTOR())));
	}
	Matrix4x4::Matrix4x4(const Vector3& t, const Quaternion& r, const Vector3& s)
	{
		this->SetXMMATRIX(DirectX::XMMatrixScalingFromVector(s.GetXMVECTOR()) *
			r.GetXMMATRIX() *
			DirectX::XMMatrixTranslationFromVector(t.GetXMVECTOR()));
	}
	Matrix4x4::~Matrix4x4()
	{
	}
	Matrix4x4 Matrix4x4::MultiplyMatrix(const Matrix4x4& l, const Matrix4x4& r)
	{
		Matrix4x4 m(l.GetXMMATRIX() * r.GetXMMATRIX());
		return m;
	}
	Matrix4x4 Matrix4x4::Inverse()
	{
		Matrix4x4 result(DirectX::XMMatrixInverse(nullptr, this->GetXMMATRIX()));
		return result;
	}
	Matrix4x4 Matrix4x4::Transpose()
	{
		Matrix4x4 result(DirectX::XMMatrixTranspose(this->GetXMMATRIX()));
		return result;
	}
	Vector3 Matrix4x4::MultiplyVector(const Vector3& v)
	{
		Vector3 result(DirectX::XMVector3TransformNormal(v.GetXMVECTOR(), this->GetXMMATRIX()));
		return result;
	}
	Vector3 Matrix4x4::MultiplyPosition(const Vector3& v)
	{
		Vector3 result(DirectX::XMVector3TransformCoord(v.GetXMVECTOR(), this->GetXMMATRIX()));
		return result;
	}
	Matrix4x4 Matrix4x4::operator*(const Matrix4x4& m)
	{
		Matrix4x4 result(MultiplyMatrix((*this), m));
		return result;
	}
	void Matrix4x4::operator=(const Matrix4x4& m)
	{
		this->m_Value = m.m_Value;
	}
	void Matrix4x4::operator*=(const Matrix4x4& m)
	{
		this->SetXMMATRIX(MultiplyMatrix((*this), m).GetXMMATRIX());
	}
	Matrix4x4 Matrix4x4::GetIdentity()
	{
		Matrix4x4 m(DirectX::XMMatrixIdentity());
		return m;
	}


	Quaternion Quaternion::m_Identity = Quaternion::GetIdentity();
	Quaternion::Quaternion()
	{
		(*this) = Quaternion::m_Identity;
	}
	Quaternion::Quaternion(const Quaternion& v)
	{
		(*this) = v;
	}
	Quaternion::Quaternion(DirectX::CXMVECTOR v)
	{
		this->SetXMVECTOR(v);
	}
	Quaternion::Quaternion(DirectX::CXMMATRIX m)
	{
		this->SetXMVECTOR(DirectX::XMQuaternionRotationMatrix(m));
	}
	Quaternion::Quaternion(const Vector3& axis, const FLOAT& radian)
	{
		this->SetXMVECTOR(DirectX::XMQuaternionRotationAxis(axis.GetXMVECTOR(), radian));
	}
	Quaternion::Quaternion(const FLOAT& x, const FLOAT& y, const FLOAT& z, const FLOAT& w)
	{
		this->m_Value.x = x;
		this->m_Value.y = y;
		this->m_Value.z = z;
		this->m_Value.w = w;
	}
	Quaternion::~Quaternion()
	{
	}
	void Quaternion::Normalize()
	{
		this->SetXMVECTOR(DirectX::XMQuaternionNormalize(this->GetXMVECTOR()));
	}
	DirectX::XMFLOAT4X4 Quaternion::GetXMFLOAT4X4()const
	{
		DirectX::XMFLOAT4X4 result;
		DirectX::XMStoreFloat4x4(&result, this->GetXMMATRIX());
		return result;
	}
	DirectX::XMFLOAT4X4 Quaternion::GetGPUUploadFloat4x4()
	{
		DirectX::XMFLOAT4X4 result;
		DirectX::XMStoreFloat4x4(&result, DirectX::XMMatrixTranspose(this->GetXMMATRIX()));
		return result;
	}
	Quaternion Quaternion::Normalize(const Quaternion& v)
	{
		Quaternion result(DirectX::XMQuaternionNormalize(v.GetXMVECTOR()));
		return result;
	}
	Quaternion Quaternion::MultiplyQuaternion(const Quaternion& q1, const Quaternion& q2)
	{
		Quaternion result(DirectX::XMQuaternionMultiply(q1.GetXMVECTOR(), q2.GetXMVECTOR()));
		return result;
	}
	Quaternion Quaternion::RotationAxis(const Vector3& axis, const FLOAT& radian)
	{
		Quaternion result(axis, radian);
		return result;
	}
	Matrix4x4 Quaternion::GetMatrix()
	{
		Matrix4x4 result((*this));
		return result;
	}
	Vector3 Quaternion::MultiplyVector(const Vector3& v)
	{
		Vector3 result(DirectX::XMVector3Rotate(v.GetXMVECTOR(), this->GetXMVECTOR()));
		return result;
	}
	Quaternion Quaternion::operator*(const Quaternion& v)
	{
		Quaternion result(DirectX::XMQuaternionMultiply(this->GetXMVECTOR(), v.GetXMVECTOR()));
		return result;
	}
	void Quaternion::operator=(const Quaternion& v)
	{
		this->m_Value = v.m_Value;
	}
	Quaternion Quaternion::GetIdentity()
	{
		Quaternion v(DirectX::XMQuaternionIdentity());
		return v;
	}


	Vector2 Vector2::m_Zero = Vector2::GetZero();
	Vector2::Vector2()
	{
		(*this) = Vector2::m_Zero;
	}
	Vector2::Vector2(const Vector2& v)
	{
		(*this) = v;
	}
	Vector2::Vector2(DirectX::CXMVECTOR v)
	{
		this->SetXMVECTOR(v);
	}
	Vector2::Vector2(DirectX::XMFLOAT2 v)
	{
		this->m_Value = v;
	}
	Vector2::Vector2(const FLOAT& v)
	{
		this->m_Value.x = v;
		this->m_Value.y = v;
	}
	Vector2::Vector2(const FLOAT& x, const FLOAT& y)
	{
		this->m_Value.x = x;
		this->m_Value.y = y;
	}
	Vector2::~Vector2()
	{
	}
	void Vector2::Normalize()
	{
		this->SetXMVECTOR(DirectX::XMVector2Normalize(this->GetXMVECTOR()));
	}
	Vector2 Vector2::Normalize(const Vector2& v)
	{
		Vector2 result(DirectX::XMVector2Normalize(v.GetXMVECTOR()));
		return result;
	}
	Vector2 Vector2::operator+(const Vector2& v)
	{
		Vector2 result(
			this->m_Value.x + v.m_Value.x,
			this->m_Value.y + v.m_Value.y);
		return result;
	}
	Vector2 Vector2::operator-(const Vector2& v)
	{
		Vector2 result(
			this->m_Value.x - v.m_Value.x,
			this->m_Value.y - v.m_Value.y);
		return result;
	}
	Vector2 Vector2::operator*(const Vector2& v)
	{
		Vector2 result(
			this->m_Value.x * v.m_Value.x,
			this->m_Value.y * v.m_Value.y);
		return result;
	}
	Vector2 Vector2::operator/(const Vector2& v)
	{
		Vector2 result(
			this->m_Value.x / v.m_Value.x,
			this->m_Value.y / v.m_Value.y);
		return result;
	}
	Vector2 Vector2::operator+(const FLOAT& v)
	{
		Vector2 result(
			this->m_Value.x + v,
			this->m_Value.y + v);
		return result;
	}
	Vector2 Vector2::operator-(const FLOAT& v)
	{
		Vector2 result(
			this->m_Value.x - v,
			this->m_Value.y - v);
		return result;
	}
	Vector2 Vector2::operator*(const FLOAT& v)
	{
		Vector2 result(
			this->m_Value.x * v,
			this->m_Value.y * v);
		return result;
	}
	Vector2 Vector2::operator/(const FLOAT& v)
	{
		Vector2 result(
			this->m_Value.x / v,
			this->m_Value.y / v);
		return result;
	}
	Vector2 Vector2::operator-()
	{
		Vector2 result(
			-this->m_Value.x,
			-this->m_Value.y);
		return result;
	}
	void Vector2::operator=(const Vector2& v)
	{
		this->m_Value = v.m_Value;
	}
	void Vector2::operator+=(const Vector2& v)
	{
		this->m_Value.x = this->m_Value.x + v.m_Value.x;
		this->m_Value.y = this->m_Value.y + v.m_Value.y;
	}
	void Vector2::operator-=(const Vector2& v)
	{
		this->m_Value.x = this->m_Value.x - v.m_Value.x;
		this->m_Value.y = this->m_Value.y - v.m_Value.y;
	}
	void Vector2::operator*=(const Vector2& v)
	{
		this->m_Value.x = this->m_Value.x * v.m_Value.x;
		this->m_Value.y = this->m_Value.y * v.m_Value.y;
	}
	void Vector2::operator/=(const Vector2& v)
	{
		this->m_Value.x = this->m_Value.x / v.m_Value.x;
		this->m_Value.y = this->m_Value.y / v.m_Value.y;
	}
	void Vector2::operator=(const FLOAT& v)
	{
		this->m_Value.x = v;
		this->m_Value.y = v;
	}
	void Vector2::operator+=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x + v;
		this->m_Value.y = this->m_Value.y + v;
	}
	void Vector2::operator-=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x - v;
		this->m_Value.y = this->m_Value.y - v;
	}
	void Vector2::operator*=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x * v;
		this->m_Value.y = this->m_Value.y * v;
	}
	void Vector2::operator/=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x / v;
		this->m_Value.y = this->m_Value.y / v;
	}
	Vector2 Vector2::GetZero()
	{
		Vector2 result(0.f, 0.f);
		return result;
	}


	Vector3 Vector3::m_Zero = Vector3::GetZero();
	Vector3::Vector3()
	{
		(*this) = Vector3::m_Zero;
	}
	Vector3::Vector3(const Vector3& v)
	{
		(*this) = v;
	}
	Vector3::Vector3(DirectX::CXMVECTOR v)
	{
		this->SetXMVECTOR(v);
	}
	Vector3::Vector3(DirectX::XMFLOAT3 v)
	{
		this->m_Value.x = v.x;
		this->m_Value.y = v.y;
		this->m_Value.z = v.z;
	}
	Vector3::Vector3(DirectX::XMFLOAT4 v)
	{
		this->m_Value.x = v.x;
		this->m_Value.y = v.y;
		this->m_Value.z = v.z;
	}
	Vector3::Vector3(const FLOAT& v)
	{
		this->m_Value.x = v;
		this->m_Value.y = v;
		this->m_Value.z = v;
	}
	Vector3::Vector3(const FLOAT& x, const FLOAT& y, const FLOAT& z)
	{
		this->m_Value.x = x;
		this->m_Value.y = y;
		this->m_Value.z = z;
	}
	Vector3::~Vector3()
	{
	}
	void Vector3::Normalize()
	{
		this->SetXMVECTOR(DirectX::XMVector3Normalize(this->GetXMVECTOR()));
	}
	Vector3 Vector3::Normalize(const Vector3& v)
	{
		Vector3 result(DirectX::XMVector3Normalize(v.GetXMVECTOR()));
		return result;
	}
	Vector3 Vector3::Dot(const Vector3& v1, const Vector3& v2)
	{
		Vector3 result(DirectX::XMVector3Dot(v1.GetXMVECTOR(), v2.GetXMVECTOR()));
		return result;
	}
	Vector3 Vector3::Cross(const Vector3& v1, const Vector3& v2)
	{
		Vector3 result(DirectX::XMVector3Cross(v1.GetXMVECTOR(), v2.GetXMVECTOR()));
		return result;
	}
	Vector3 Vector3::Lerp(const Vector3& v1, const Vector3& v2, const FLOAT& t)
	{
		Vector3 result(
			v1.X() * (1.f - t) + v2.X() * t,
			v1.Y() * (1.f - t) + v2.Y() * t,
			v1.Z() * (1.f - t) + v2.Z() * t);
		return result;
	}
	Vector3 Vector3::operator+(const Vector3& v)
	{
		Vector3 result(
			this->m_Value.x + v.m_Value.x,
			this->m_Value.y + v.m_Value.y,
			this->m_Value.z + v.m_Value.z);
		return result;
	}
	Vector3 Vector3::operator-(const Vector3& v)
	{
		Vector3 result(
			this->m_Value.x - v.m_Value.x,
			this->m_Value.y - v.m_Value.y,
			this->m_Value.z - v.m_Value.z);
		return result;
	}
	Vector3 Vector3::operator*(const Vector3& v)
	{
		Vector3 result(
			this->m_Value.x * v.m_Value.x,
			this->m_Value.y * v.m_Value.y,
			this->m_Value.z * v.m_Value.z);
		return result;
	}
	Vector3 Vector3::operator/(const Vector3& v)
	{
		Vector3 result(
			this->m_Value.x / v.m_Value.x,
			this->m_Value.y / v.m_Value.y,
			this->m_Value.z / v.m_Value.z);
		return result;
	}
	Vector3 Vector3::operator+(const FLOAT& v)
	{
		Vector3 result(
			this->m_Value.x + v,
			this->m_Value.y + v,
			this->m_Value.z + v);
		return result;
	}
	Vector3 Vector3::operator-(const FLOAT& v)
	{
		Vector3 result(
			this->m_Value.x - v,
			this->m_Value.y - v,
			this->m_Value.z - v);
		return result;
	}
	Vector3 Vector3::operator*(const FLOAT& v)
	{
		Vector3 result(
			this->m_Value.x * v,
			this->m_Value.y * v,
			this->m_Value.z * v);
		return result;
	}
	Vector3 Vector3::operator/(const FLOAT& v)
	{
		Vector3 result(
			this->m_Value.x / v,
			this->m_Value.y / v,
			this->m_Value.z / v);
		return result;
	}
	Vector3 Vector3::operator-()
	{
		Vector3 result(
			-this->m_Value.x,
			-this->m_Value.y,
			-this->m_Value.z);
		return result;
	}
	BOOL Vector3::operator==(const Vector3& v)
	{
		BOOL result = false;
		if (fabsf(this->m_Value.x - v.m_Value.x) < 0.00001f &&
			fabsf(this->m_Value.y - v.m_Value.y) < 0.00001f &&
			fabsf(this->m_Value.z - v.m_Value.z) < 0.00001f)
			result = true;
		return result;
	}
	BOOL Vector3::operator!=(const Vector3& v)
	{
		BOOL result = true;
		if ((*this) == v)
			result = false;
		return result;
	}
	void Vector3::operator=(const Vector3& v)
	{
		this->m_Value = v.m_Value;
	}
	void Vector3::operator+=(const Vector3& v)
	{
		this->m_Value.x = this->m_Value.x + v.m_Value.x;
		this->m_Value.y = this->m_Value.y + v.m_Value.y;
		this->m_Value.z = this->m_Value.z + v.m_Value.z;
	}
	void Vector3::operator-=(const Vector3& v)
	{
		this->m_Value.x = this->m_Value.x - v.m_Value.x;
		this->m_Value.y = this->m_Value.y - v.m_Value.y;
		this->m_Value.z = this->m_Value.z - v.m_Value.z;
	}
	void Vector3::operator*=(const Vector3& v)
	{
		this->m_Value.x = this->m_Value.x * v.m_Value.x;
		this->m_Value.y = this->m_Value.y * v.m_Value.y;
		this->m_Value.z = this->m_Value.z * v.m_Value.z;
	}
	void Vector3::operator/=(const Vector3& v)
	{
		this->m_Value.x = this->m_Value.x / v.m_Value.x;
		this->m_Value.y = this->m_Value.y / v.m_Value.y;
		this->m_Value.z = this->m_Value.z / v.m_Value.z;
	}
	void Vector3::operator=(const FLOAT& v)
	{
		this->m_Value.x = v;
		this->m_Value.y = v;
		this->m_Value.z = v;
	}
	void Vector3::operator+=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x + v;
		this->m_Value.y = this->m_Value.y + v;
		this->m_Value.z = this->m_Value.z + v;
	}
	void Vector3::operator-=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x - v;
		this->m_Value.y = this->m_Value.y - v;
		this->m_Value.z = this->m_Value.z - v;
	}
	void Vector3::operator*=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x * v;
		this->m_Value.y = this->m_Value.y * v;
		this->m_Value.z = this->m_Value.z * v;
	}
	void Vector3::operator/=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x / v;
		this->m_Value.y = this->m_Value.y / v;
		this->m_Value.z = this->m_Value.z / v;
	}
	Vector3 Vector3::GetZero()
	{
		Vector3 v(0.f, 0.f, 0.f);
		return v;
	}


	Vector4 Vector4::m_Zero = Vector4::GetZero();
	Vector4::Vector4()
	{
		(*this) = Vector4::m_Zero;
	}
	Vector4::Vector4(const Vector3& v)
	{
		XMFLOAT3 temp = v.GetXMFLOAT3();
		this->m_Value.x = temp.x;
		this->m_Value.y = temp.y;
		this->m_Value.z = temp.z;
		this->m_Value.w = 0.f;
	}
	Vector4::Vector4(const Vector4& v)
	{
		(*this) = v.m_Value;
	}
	Vector4::Vector4(DirectX::CXMVECTOR v)
	{
		this->SetXMVECTOR(v);
	}
	Vector4::Vector4(DirectX::XMFLOAT4 v)
	{
		this->m_Value = v;
	}
	Vector4::Vector4(const FLOAT& v)
	{
		this->m_Value.x = v;
		this->m_Value.y = v;
		this->m_Value.z = v;
		this->m_Value.w = v;
	}
	Vector4::Vector4(const FLOAT& x, const FLOAT& y, const FLOAT& z)
	{
		this->m_Value.x = x;
		this->m_Value.y = y;
		this->m_Value.z = z;
		this->m_Value.w = 0.f;
	}
	Vector4::Vector4(const FLOAT& x, const FLOAT& y, const FLOAT& z, const FLOAT& w)
	{
		this->m_Value.x = x;
		this->m_Value.y = y;
		this->m_Value.z = z;
		this->m_Value.w = w;
	}
	Vector4::Vector4(const INT& x, const INT& y, const INT& z, const INT& w)
	{
		this->m_Value.x = static_cast<FLOAT>(x);
		this->m_Value.y = static_cast<FLOAT>(y);
		this->m_Value.z = static_cast<FLOAT>(z);
		this->m_Value.w = static_cast<FLOAT>(w);
	}
	Vector4::~Vector4()
	{
	}
	Vector4 Vector4::operator+(const Vector4& v)
	{
		Vector4 result(
			this->m_Value.x + v.m_Value.x,
			this->m_Value.y + v.m_Value.y,
			this->m_Value.z + v.m_Value.z,
			this->m_Value.w + v.m_Value.w);
		return result;
	}
	Vector4 Vector4::operator-(const Vector4& v)
	{
		Vector4 result(
			this->m_Value.x - v.m_Value.x,
			this->m_Value.y - v.m_Value.y,
			this->m_Value.z - v.m_Value.z,
			this->m_Value.w - v.m_Value.w);
		return result;
	}
	Vector4 Vector4::operator*(const Vector4& v)
	{
		Vector4 result(
			this->m_Value.x * v.m_Value.x,
			this->m_Value.y * v.m_Value.y,
			this->m_Value.z * v.m_Value.z,
			this->m_Value.w * v.m_Value.w);
		return result;
	}
	Vector4 Vector4::operator/(const Vector4& v)
	{
		Vector4 result(
			this->m_Value.x / v.m_Value.x,
			this->m_Value.y / v.m_Value.y,
			this->m_Value.z / v.m_Value.z,
			this->m_Value.w / v.m_Value.w);
		return result;
	}
	Vector4 Vector4::operator+(const FLOAT& v)
	{
		Vector4 result(
			this->m_Value.x + v,
			this->m_Value.y + v,
			this->m_Value.z + v,
			this->m_Value.w + v);
		return result;
	}
	Vector4 Vector4::operator-(const FLOAT& v)
	{
		Vector4 result(
			this->m_Value.x - v,
			this->m_Value.y - v,
			this->m_Value.z - v,
			this->m_Value.w - v);
		return result;
	}
	Vector4 Vector4::operator*(const FLOAT& v)
	{
		Vector4 result(
			this->m_Value.x * v,
			this->m_Value.y * v,
			this->m_Value.z * v,
			this->m_Value.w * v);
		return result;
	}
	Vector4 Vector4::operator/(const FLOAT& v)
	{
		Vector4 result(
			this->m_Value.x / v,
			this->m_Value.y / v,
			this->m_Value.z / v,
			this->m_Value.w / v);
		return result;
	}
	Vector4 Vector4::operator-()
	{
		Vector4 result(
			-this->m_Value.x,
			-this->m_Value.y,
			-this->m_Value.z,
			-this->m_Value.w);
		return result;
	}
	void Vector4::operator=(const Vector4& v)
	{
		this->m_Value = v.m_Value;
	}
	void Vector4::operator+=(const Vector4& v)
	{
		this->m_Value.x = this->m_Value.x + v.m_Value.x;
		this->m_Value.y = this->m_Value.y + v.m_Value.y;
		this->m_Value.z = this->m_Value.z + v.m_Value.z;
		this->m_Value.w = this->m_Value.w + v.m_Value.w;
	}
	void Vector4::operator-=(const Vector4& v)
	{
		this->m_Value.x = this->m_Value.x - v.m_Value.x;
		this->m_Value.y = this->m_Value.y - v.m_Value.y;
		this->m_Value.z = this->m_Value.z - v.m_Value.z;
		this->m_Value.w = this->m_Value.w - v.m_Value.w;
	}
	void Vector4::operator*=(const Vector4& v)
	{
		this->m_Value.x = this->m_Value.x * v.m_Value.x;
		this->m_Value.y = this->m_Value.y * v.m_Value.y;
		this->m_Value.z = this->m_Value.z * v.m_Value.z;
		this->m_Value.w = this->m_Value.w * v.m_Value.w;
	}
	void Vector4::operator/=(const Vector4& v)
	{
		this->m_Value.x = this->m_Value.x / v.m_Value.x;
		this->m_Value.y = this->m_Value.y / v.m_Value.y;
		this->m_Value.z = this->m_Value.z / v.m_Value.z;
		this->m_Value.w = this->m_Value.w / v.m_Value.w;
	}
	void Vector4::operator=(const FLOAT& v)
	{
		this->m_Value.x = v;
		this->m_Value.y = v;
		this->m_Value.z = v;
		this->m_Value.w = v;
	}
	void Vector4::operator+=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x + v;
		this->m_Value.y = this->m_Value.y + v;
		this->m_Value.z = this->m_Value.z + v;
		this->m_Value.w = this->m_Value.w + v;
	}
	void Vector4::operator-=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x - v;
		this->m_Value.y = this->m_Value.y - v;
		this->m_Value.z = this->m_Value.z - v;
		this->m_Value.w = this->m_Value.w - v;
	}
	void Vector4::operator*=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x * v;
		this->m_Value.y = this->m_Value.y * v;
		this->m_Value.z = this->m_Value.z * v;
		this->m_Value.w = this->m_Value.w * v;
	}
	void Vector4::operator/=(const FLOAT& v)
	{
		this->m_Value.x = this->m_Value.x / v;
		this->m_Value.y = this->m_Value.y / v;
		this->m_Value.z = this->m_Value.z / v;
		this->m_Value.w = this->m_Value.w / v;
	}
	Vector4 Vector4::GetZero()
	{
		Vector4 v(0.f, 0.f, 0.f, 0.f);
		return v;
	}


	Vector2Int Vector2Int::m_Zero = Vector2Int::GetZero();
	Vector2Int::Vector2Int()
	{
		(*this) = Vector2Int::m_Zero;
	}
	Vector2Int::Vector2Int(const Vector2Int& v)
	{
		(*this) = v;
	}
	Vector2Int::Vector2Int(const INT& v)
	{
		this->x = v;
		this->y = v;
	}
	Vector2Int::Vector2Int(const INT& x, const INT& y)
	{
		this->x = x;
		this->y = y;
	}
	Vector2Int::Vector2Int(const FLOAT& v)
	{
		this->x = static_cast<INT>(v);
		this->y = static_cast<INT>(v);
	}
	Vector2Int::Vector2Int(const FLOAT& x, const FLOAT& y)
	{
		this->x = static_cast<INT>(x);
		this->y = static_cast<INT>(y);
	}
	Vector2Int::~Vector2Int()
	{
	}
	Vector2Int Vector2Int::operator+(const Vector2Int& v)
	{
		Vector2Int result(
			this->x + v.x,
			this->y + v.y);
		return result;
	}
	Vector2Int Vector2Int::operator-(const Vector2Int& v)
	{
		Vector2Int result(
			this->x - v.x,
			this->y - v.y);
		return result;
	}
	Vector2Int Vector2Int::operator*(const Vector2Int& v)
	{
		Vector2Int result(
			this->x * v.x,
			this->y * v.y);
		return result;
	}
	Vector2Int Vector2Int::operator/(const Vector2Int& v)
	{
		Vector2Int result(
			this->x / v.x,
			this->y / v.y);
		return result;
	}
	Vector2Int Vector2Int::operator+(const INT& v)
	{
		Vector2Int result(
			this->x + v,
			this->y + v);
		return result;
	}
	Vector2Int Vector2Int::operator-(const INT& v)
	{
		Vector2Int result(
			this->x - v,
			this->y - v);
		return result;
	}
	Vector2Int Vector2Int::operator*(const INT& v)
	{
		Vector2Int result(
			this->x * v,
			this->y * v);
		return result;
	}
	Vector2Int Vector2Int::operator/(const INT& v)
	{
		Vector2Int result(
			this->x / v,
			this->y / v);
		return result;
	}
	Vector2Int Vector2Int::operator-()
	{
		Vector2Int result(-this->x, -this->y);
		return result;
	}
	void Vector2Int::operator=(const Vector2Int& v)
	{
		this->x = v.x;
		this->y = v.y;
	}
	void Vector2Int::operator+=(const Vector2Int& v)
	{
		this->x = this->x + v.x;
		this->y = this->y + v.y;
	}
	void Vector2Int::operator-=(const Vector2Int& v)
	{
		this->x = this->x - v.x;
		this->y = this->y - v.y;
	}
	void Vector2Int::operator*=(const Vector2Int& v)
	{
		this->x = this->x * v.x;
		this->y = this->y * v.y;
	}
	void Vector2Int::operator/=(const Vector2Int& v)
	{
		this->x = this->x / v.x;
		this->y = this->y / v.y;
	}
	void Vector2Int::operator=(const INT& v)
	{
		this->x = v;
		this->y = v;
	}
	void Vector2Int::operator+=(const INT& v)
	{
		this->x = this->x + v;
		this->y = this->y + v;
	}
	void Vector2Int::operator-=(const INT& v)
	{
		this->x = this->x - v;
		this->y = this->y - v;
	}
	void Vector2Int::operator*=(const INT& v)
	{
		this->x = this->x * v;
		this->y = this->y * v;
	}
	void Vector2Int::operator/=(const INT& v)
	{
		this->x = this->x / v;
		this->y = this->y / v;
	}
	Vector2Int Vector2Int::GetZero()
	{
		Vector2Int result(0, 0);
		return result;
	}


	Vector4Int Vector4Int::m_Zero = Vector4Int::GetZero();
	Vector4Int::Vector4Int()
	{
		(*this) = Vector4Int::m_Zero;
	}
	Vector4Int::Vector4Int(const Vector4Int& v)
	{
		(*this) = v;
	}
	Vector4Int::Vector4Int(const INT& v)
	{
		this->x = v;
		this->y = v;
		this->z = v;
		this->w = v;
	}
	Vector4Int::Vector4Int(const INT& x, const INT& y, const INT& z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = 0;
	}
	Vector4Int::Vector4Int(const INT& x, const INT& y, const INT& z, const INT& w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
	Vector4Int::Vector4Int(const FLOAT& v)
	{
		this->x = static_cast<INT>(v);
		this->y = static_cast<INT>(v);
		this->z = static_cast<INT>(v);
		this->w = static_cast<INT>(v);
	}
	Vector4Int::Vector4Int(const FLOAT& x, const FLOAT& y, const FLOAT& z)
	{
		this->x = static_cast<INT>(x);
		this->y = static_cast<INT>(y);
		this->z = static_cast<INT>(z);
		this->w = 0;
	}
	Vector4Int::Vector4Int(const FLOAT& x, const FLOAT& y, const FLOAT& z, const FLOAT& w)
	{
		this->x = static_cast<INT>(x);
		this->y = static_cast<INT>(y);
		this->z = static_cast<INT>(z);
		this->w = static_cast<INT>(w);
	}
	Vector4Int::Vector4Int(const Vector3& v)
	{
		this->x = static_cast<INT>(v.X());
		this->y = static_cast<INT>(v.Y());
		this->z = static_cast<INT>(v.Z());
		this->w = 0;
	}
	Vector4Int::Vector4Int(const Vector4& v)
	{
		this->x = static_cast<INT>(v.X());
		this->y = static_cast<INT>(v.Y());
		this->z = static_cast<INT>(v.Z());
		this->w = static_cast<INT>(v.W());
	}
	Vector4Int::~Vector4Int()
	{
	}
	Vector4Int Vector4Int::operator+(const Vector4Int& v)
	{
		Vector4Int result(
			this->x + v.x,
			this->y + v.y,
			this->z + v.z,
			this->w + v.w);
		return result;
	}
	Vector4Int Vector4Int::operator-(const Vector4Int& v)
	{
		Vector4Int result(
			this->x - v.x,
			this->y - v.y,
			this->z - v.z,
			this->w - v.w);
		return result;
	}
	Vector4Int Vector4Int::operator*(const Vector4Int& v)
	{
		Vector4Int result(
			this->x * v.x,
			this->y * v.y,
			this->z * v.z,
			this->w * v.w);
		return result;
	}
	Vector4Int Vector4Int::operator/(const Vector4Int& v)
	{
		Vector4Int result(
			this->x / v.x,
			this->y / v.y,
			this->z / v.z,
			this->w / v.w);
		return result;
	}
	Vector4Int Vector4Int::operator+(const INT& v)
	{
		Vector4Int result(
			this->x + v,
			this->y + v,
			this->z + v,
			this->w + v);
		return result;
	}
	Vector4Int Vector4Int::operator-(const INT& v)
	{
		Vector4Int result(
			this->x - v,
			this->y - v,
			this->z - v,
			this->w - v);
		return result;
	}
	Vector4Int Vector4Int::operator*(const INT& v)
	{
		Vector4Int result(
			this->x * v,
			this->y * v,
			this->z * v,
			this->w * v);
		return result;
	}
	Vector4Int Vector4Int::operator/(const INT& v)
	{
		Vector4Int result(
			this->x / v,
			this->y / v,
			this->z / v,
			this->w / v);
		return result;
	}
	Vector4Int Vector4Int::operator-()
	{
		Vector4Int result(
			-this->x,
			-this->y,
			-this->z,
			-this->w);
		return result;
	}
	void Vector4Int::operator=(const Vector4Int& v)
	{
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
		this->w = v.w;
	}
	void Vector4Int::operator+=(const Vector4Int& v)
	{
		this->x = this->x + v.x;
		this->y = this->y + v.y;
		this->z = this->z + v.z;
		this->w = this->w + v.w;
	}
	void Vector4Int::operator-=(const Vector4Int& v)
	{
		this->x = this->x - v.x;
		this->y = this->y - v.y;
		this->z = this->z - v.z;
		this->w = this->w - v.w;
	}
	void Vector4Int::operator*=(const Vector4Int& v)
	{
		this->x = this->x * v.x;
		this->y = this->y * v.y;
		this->z = this->z * v.z;
		this->w = this->w * v.w;
	}
	void Vector4Int::operator/=(const Vector4Int& v)
	{
		this->x = this->x / v.x;
		this->y = this->y / v.y;
		this->z = this->z / v.z;
		this->w = this->w / v.w;
	}
	void Vector4Int::operator=(const INT& v)
	{
		this->x = v;
		this->y = v;
		this->z = v;
		this->w = v;
	}
	void Vector4Int::operator+=(const INT& v)
	{
		this->x = this->x + v;
		this->y = this->y + v;
		this->z = this->z + v;
		this->w = this->w + v;
	}
	void Vector4Int::operator-=(const INT& v)
	{
		this->x = this->x - v;
		this->y = this->y - v;
		this->z = this->z - v;
		this->w = this->w - v;
	}
	void Vector4Int::operator*=(const INT& v)
	{
		this->x = this->x * v;
		this->y = this->y * v;
		this->z = this->z * v;
		this->w = this->w * v;
	}
	void Vector4Int::operator/=(const INT& v)
	{
		this->x = this->x / v;
		this->y = this->y / v;
		this->z = this->z / v;
		this->w = this->w / v;
	}
	Vector4Int Vector4Int::GetZero()
	{
		Vector4Int result(0, 0, 0, 0);
		return result;
	}


	FLOAT CMath::m_PI		= 3.1415926536f;
	FLOAT CMath::m_RadToDeg = 57.2957795f;
	FLOAT CMath::m_DegToRad = 0.0174532925f;
	const FLOAT& CMath::GetPI()
	{
		return CMath::m_PI;
	}
	const FLOAT& CMath::GetDegToRad()
	{
		return CMath::m_DegToRad;
	}
	const FLOAT& CMath::GetRadToDeg()
	{
		return CMath::m_RadToDeg;
	}
	BOOL CMath::Lerp(const INT& x0, const INT& y0, const INT& x1, const INT& y1, const INT& t, INT& phi)
	{
		if (t < x0 || t > x1)
			return false;
		phi = (INT)(((FLOAT)(t - x1)) / ((FLOAT)(x0 - x1)) * (FLOAT)y0 + ((FLOAT)(t - x0)) / ((FLOAT)(x1 - x0)) * (FLOAT)y1);
		return true;
	}
	FLOAT CMath::Lerp(const FLOAT& v0, const FLOAT& v1, const FLOAT& t)
	{
		return (v0 * (1.f - t) + v1 * t);
	}
}