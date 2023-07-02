#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>
#include <cuda_runtime.h>
#include "FVector3.h"
#include "FMatrix.h"
#include "FVector4.h"
#include "Timer.h"

struct Camera
{
	Camera() = default;
	Camera(const FVector3& _origin, float _fovAngle) : origin(_origin), fovAngle(_fovAngle) {}


	FVector3 origin{};
	float fovAngle{90.f};

	FVector3 forward{ 0.266f,-0.453f,0.860f };
	FVector3 up {FVector3::UnitY};
	FVector3 right{FVector3::UnitX};

	float totalPitch = 0.f;
	float totalYaw = 0.f;

	FMatrix cameraToWorld{};

	void CalculateCameraToWorld()
	{
		right = FVector3::Cross(FVector3::UnitY, forward).Normalized();
		up = FVector3::Cross(forward, right).Normalized();

		cameraToWorld = { right,up,forward,origin };
	}
	void Update(Raytracing::Timer* pTimer)
	{
		const float deltaTime = pTimer->GetElapsed();

		//Keyboard Input
		const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

		//MOVEMENT
		//========
		float keyboardMoveSpeed = 20.f;
		keyboardMoveSpeed *= pKeyboardState[SDL_SCANCODE_LSHIFT] ? 4.f : 1.f; //Increase speed when SHIFT is pressed

		//FORWARD-BACKWARD
		float displacement = (pKeyboardState[SDL_SCANCODE_W] - pKeyboardState[SDL_SCANCODE_S]) * keyboardMoveSpeed *
			deltaTime;
		origin += forward * displacement;


		//RIGHT-LEFT
		displacement = (pKeyboardState[SDL_SCANCODE_D] - pKeyboardState[SDL_SCANCODE_A]) * keyboardMoveSpeed *
			deltaTime;
		origin += right * displacement;

		//ROTATION
		//========
		constexpr float mouseMoveSpeed = 16.f;
		constexpr float mouseRotationSensitivity = 0.03f;

		//Mouse Input
		int mouseX{}, mouseY{};
		const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

		if (mouseState == SDL_BUTTON_LMASK) //Forward-Backward & Yaw Rotation
		{
			//Movement
			displacement = -static_cast<float>(mouseY) * mouseMoveSpeed * deltaTime;
			origin += forward * displacement;

			//Yaw Rotation
			totalYaw += static_cast<float>(mouseX) * mouseRotationSensitivity;
		}
		else if (mouseState == SDL_BUTTON_RMASK) //Pitch & Yaw Rotation
		{
			//Pitch
			totalPitch -= static_cast<float>(mouseY) * mouseRotationSensitivity;

			//Yaw
			totalYaw += static_cast<float>(mouseX) * mouseRotationSensitivity;
		}
		else if (mouseState == (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK)) //World Up Movement
		{
			displacement = -static_cast<float>(mouseY) * mouseMoveSpeed * deltaTime;
			origin += FVector3::UnitY * displacement;
		}

		////Update Forward Axis based on Yaw/Pitch
		const FMatrix yawRotation = FMatrix::CreateRotationY(totalYaw);
		const FMatrix pitchRotation = FMatrix::CreateRotationX(totalPitch);
		const FMatrix finalRotation = pitchRotation * yawRotation;

		forward = finalRotation.TransformVector(FVector3::UnitZ);
		forward.Normalize();
	}
};