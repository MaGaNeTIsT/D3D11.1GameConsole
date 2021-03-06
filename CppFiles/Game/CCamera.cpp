#include "../../Entry/MyMain.h"
#include "../../Headers/Game/CCamera.h"
#include "../../Headers/Base/CTimer.h"
#include "../../Headers/Base/CInput.h"
#include "../../Headers/Base/CManager.h"
#include "../../Headers/Base/CRenderDevice.h"

CCamera::CCamera()
{
	this->m_CameraInfo.Viewport = CustomType::Vector4(0, 0, ENGINE_SCREEN_WIDTH, ENGINE_SCREEN_HEIGHT);
	this->m_CameraInfo.Fov	= ENGINE_CAMERA_FOV;
	this->m_CameraInfo.Near	= ENGINE_CAMERA_NEAR;
	this->m_CameraInfo.Far	= ENGINE_CAMERA_FAR;

	this->m_CameraControlInfo.MoveSpeed = ENGINE_CAMERA_MOVE_SPEED;
	this->m_CameraControlInfo.LookSpeed = ENGINE_CAMERA_LOOK_SPEED * CustomType::CMath::GetDegToRad();

	this->m_Position.Reset();
	this->m_Rotation.Identity();
	this->m_Scale = 1.f;

	this->ReCalculateProjectionMatrix();
	this->ReCalculateViewMatrix();
	this->ReCalculateViewProjectionMatrix();
}
CCamera::CCamera(const CustomType::Vector3& position, const CustomType::Quaternion& rotation)
{
	this->m_CameraInfo.Viewport	= CustomType::Vector4(0, 0, ENGINE_SCREEN_WIDTH, ENGINE_SCREEN_HEIGHT);
	this->m_CameraInfo.Fov	= ENGINE_CAMERA_FOV;
	this->m_CameraInfo.Near	= ENGINE_CAMERA_NEAR;
	this->m_CameraInfo.Far	= ENGINE_CAMERA_FAR;

	this->m_CameraControlInfo.MoveSpeed	= ENGINE_CAMERA_MOVE_SPEED;
	this->m_CameraControlInfo.LookSpeed	= ENGINE_CAMERA_LOOK_SPEED * CustomType::CMath::GetDegToRad();

	this->m_Position	= position;
	this->m_Rotation	= rotation;
	this->m_Scale		= 1.f;

	this->ReCalculateProjectionMatrix();
	this->ReCalculateViewMatrix();
	this->ReCalculateViewProjectionMatrix();
}
CCamera::CCamera(const CCamera& camera)
{
	this->m_Position			= camera.m_Position;
	this->m_Rotation			= camera.m_Rotation;
	this->m_Scale				= 1.f;

	this->m_CameraInfo			= camera.m_CameraInfo;
	this->m_CameraControlInfo	= camera.m_CameraControlInfo;

	this->m_ViewMatrix				= camera.m_ViewMatrix;
	this->m_ViewInvMatrix			= camera.m_ViewInvMatrix;
	this->m_ProjectionMatrix		= camera.m_ProjectionMatrix;
	this->m_ProjectionInvMatrix		= camera.m_ProjectionInvMatrix;
	this->m_ViewProjectionMatrix	= camera.m_ViewProjectionMatrix;
	this->m_ViewProjectionInvMatrix	= camera.m_ViewProjectionInvMatrix;
}
CCamera::~CCamera()
{
}
void CCamera::ReCalculateProjectionMatrix()
{
	this->m_ProjectionMatrix = CustomType::Matrix4x4(XMMatrixPerspectiveFovLH(
		this->m_CameraInfo.Fov * CustomType::CMath::GetDegToRad(),
		static_cast<FLOAT>(this->m_CameraInfo.Viewport.Z() - this->m_CameraInfo.Viewport.X()) / static_cast<FLOAT>(this->m_CameraInfo.Viewport.W() - this->m_CameraInfo.Viewport.Y()),
		this->m_CameraInfo.Near, this->m_CameraInfo.Far));
	this->m_ProjectionInvMatrix = this->m_ProjectionMatrix.Inverse();
}
void CCamera::ReCalculateViewMatrix()
{
	this->m_ViewInvMatrix = CustomType::Matrix4x4(this->m_Position, this->m_Rotation);
	this->m_ViewMatrix = this->m_ViewInvMatrix.Inverse();
}
void CCamera::ReCalculateViewProjectionMatrix()
{
	this->m_ViewProjectionMatrix = this->m_ViewMatrix * this->m_ProjectionMatrix;
	this->m_ViewProjectionInvMatrix = this->m_ViewProjectionMatrix.Inverse();
}
void CCamera::Init()
{
}
void CCamera::Uninit()
{

}
void CCamera::Update()
{
	BOOL moveForward		= CInput::GetKeyPress('W');
	BOOL moveBack			= CInput::GetKeyPress('S');
	BOOL moveRight			= CInput::GetKeyPress('D');
	BOOL moveLeft			= CInput::GetKeyPress('A');
	BOOL moveUp				= CInput::GetKeyPress('Q');
	BOOL moveDown			= CInput::GetKeyPress('E');
	BOOL lookUp				= CInput::GetKeyPress('I');
	BOOL lookDown			= CInput::GetKeyPress('K');
	BOOL lookLeft			= CInput::GetKeyPress('J');
	BOOL lookRight			= CInput::GetKeyPress('L');
	BOOL lookRotClock		= CInput::GetKeyPress('O');
	BOOL lookRotAntiClock	= CInput::GetKeyPress('U');
	FLOAT deltaTime = static_cast<FLOAT>(CManager::GetManager()->GetGameTimer()->GetDeltaTime());
	if (moveForward || moveBack)
	{
		CustomType::Vector3 moveVector = this->GetForwardVector();
		if (moveForward)
			this->m_Position += moveVector * (this->m_CameraControlInfo.MoveSpeed * deltaTime);
		if (moveBack)
			this->m_Position += -moveVector * (this->m_CameraControlInfo.MoveSpeed * deltaTime);
	}
	if (moveRight || moveLeft)
	{
		CustomType::Vector3 moveVector = this->GetRightVector();
		if (moveRight)
			this->m_Position += moveVector * (this->m_CameraControlInfo.MoveSpeed * deltaTime);
		if (moveLeft)
			this->m_Position += -moveVector * (this->m_CameraControlInfo.MoveSpeed * deltaTime);
	}
	if (moveUp || moveDown)
	{
		CustomType::Vector3 moveVector = this->GetUpVector();
		if (moveUp)
			this->m_Position += moveVector * (this->m_CameraControlInfo.MoveSpeed * deltaTime);
		if (moveDown)
			this->m_Position += -moveVector * (this->m_CameraControlInfo.MoveSpeed * deltaTime);
	}
	if (lookUp || lookDown)
	{
		CustomType::Vector3 lookAxis = this->GetRightVector();
		if (lookUp)
		{
			CustomType::Quaternion lookRotation(lookAxis, -this->m_CameraControlInfo.LookSpeed * deltaTime);
			this->m_Rotation = CustomType::Quaternion::MultiplyQuaternion(this->m_Rotation, lookRotation);
		}
		if (lookDown)
		{
			CustomType::Quaternion lookRotation(lookAxis, this->m_CameraControlInfo.LookSpeed * deltaTime);
			this->m_Rotation = CustomType::Quaternion::MultiplyQuaternion(this->m_Rotation, lookRotation);
		}
	}
	if (lookLeft || lookRight)
	{
		CustomType::Vector3 lookAxis = this->GetUpVector();
		if (lookLeft)
		{
			CustomType::Quaternion lookRotation(lookAxis, -this->m_CameraControlInfo.LookSpeed * deltaTime);
			this->m_Rotation = CustomType::Quaternion::MultiplyQuaternion(this->m_Rotation, lookRotation);
		}
		if (lookRight)
		{
			CustomType::Quaternion lookRotation(lookAxis, this->m_CameraControlInfo.LookSpeed * deltaTime);
			this->m_Rotation = CustomType::Quaternion::MultiplyQuaternion(this->m_Rotation, lookRotation);
		}
	}
	if (lookRotClock || lookRotAntiClock)
	{
		CustomType::Vector3 lookAxis = this->GetForwardVector();
		if (lookRotClock)
		{
			CustomType::Quaternion lookRotation(lookAxis, -this->m_CameraControlInfo.LookSpeed * deltaTime);
			this->m_Rotation = CustomType::Quaternion::MultiplyQuaternion(this->m_Rotation, lookRotation);
		}
		if (lookRotAntiClock)
		{
			CustomType::Quaternion lookRotation(lookAxis, this->m_CameraControlInfo.LookSpeed * deltaTime);
			this->m_Rotation = CustomType::Quaternion::MultiplyQuaternion(this->m_Rotation, lookRotation);
		}
	}
	this->ReCalculateViewMatrix();
	this->ReCalculateViewProjectionMatrix();
}