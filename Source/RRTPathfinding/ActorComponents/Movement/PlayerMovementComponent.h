
#pragma once

#include "CoreMinimal.h"

#include "Components/ActorComponent.h"
#include "PlayerMovementComponent.generated.h"

class ANavigationBox;
class MovementState;
class AcceleratingState;
class DeceleratingState;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class RRTPATHFINDING_API UPlayerMovementComponent : public UActorComponent
{
	GENERATED_BODY()

	friend class MovementState;
	friend class AcceleratingState;
	friend class DeceleratingState;

protected:
	UPlayerMovementComponent();
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	virtual void BeginPlay() override;
	void ChangeState(const TSharedPtr<MovementState>& NewState);

	void SetTargetLocation(const FVector& NewTargetLocation);
	
	UFUNCTION()
	void SetPathPoints(const TArray<FVector>& NewPathPoints);
	
	UPROPERTY(EditAnywhere, Category = "[RRT]")
	TObjectPtr<AActor> EndPoint;
	
	UPROPERTY(EditAnywhere, Category = "[RRT]")
	float ObjectSize = 50.f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float MaxSpeed = 600.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float MinSpeed_OnDeceleration = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float TurnSpeed = 10.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Movement")
	float TurnSpeedOnDeceleration = 3.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float InterpolationChangeSpeed = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float DecelerationSpeedThreshold = 200.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	float ArrivalThreshold = 100.0f;
	
	FVector TargetLocation = FVector::ZeroVector;
	FVector CurrentVelocity = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "[RRT]")
	TObjectPtr<ANavigationBox> NavigationBox;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
	TArray<FVector> PathPoints;
	int32 CurrentPathPointIndex = 0;
	
	void GetNavigationBox();
	void GoPathToFinish() const;
	void UpdateTargetLocation_Tick();
	void UpdateSpeedStage_Tick();

private:
	TSharedPtr<MovementState> CurrentState;
	TSharedPtr<AcceleratingState> Accelerating;
	TSharedPtr<DeceleratingState> Decelerating;
};
