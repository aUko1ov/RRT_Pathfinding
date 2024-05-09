
#pragma once

#include "CoreMinimal.h"

class UPlayerMovementComponent;

class MovementState
{
public:
	virtual ~MovementState() = default;
	virtual void Movement_Tick(float DeltaTime, UPlayerMovementComponent& MovementComponent) = 0;
};

class AcceleratingState : public MovementState
{
public:
	virtual void Movement_Tick(float DeltaTime, UPlayerMovementComponent& MovementComponent) override;
};

class DeceleratingState : public MovementState
{
public:
	virtual void Movement_Tick(float DeltaTime, UPlayerMovementComponent& MovementComponent) override;
};
