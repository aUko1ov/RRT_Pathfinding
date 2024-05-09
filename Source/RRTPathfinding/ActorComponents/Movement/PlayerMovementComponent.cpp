#include "PlayerMovementComponent.h"

#include "RRTPathfinding/Actors/NavigationBox.h"
#include "RRTPathfinding/Movement/MoveStateMachine.h"


UPlayerMovementComponent::UPlayerMovementComponent()
{
    PrimaryComponentTick.bCanEverTick = true;

    Accelerating = MakeShareable(new AcceleratingState());
    Decelerating = MakeShareable(new DeceleratingState());
    
    CurrentState = Accelerating; 
}

void UPlayerMovementComponent::BeginPlay()
{
    Super::BeginPlay();
    GetNavigationBox();
    GoPathToFinish();
}

void UPlayerMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    UpdateTargetLocation_Tick();
    UpdateSpeedStage_Tick();
    
    if (CurrentState)
    {
        CurrentState->Movement_Tick(DeltaTime, *this);
    }
}


void UPlayerMovementComponent::GetNavigationBox()
{
    if (NavigationBox)
    {
        NavigationBox->OnPathCalculated.AddDynamic(this, &UPlayerMovementComponent::SetPathPoints);
    }
}

void UPlayerMovementComponent::GoPathToFinish() const
{
    if (NavigationBox && EndPoint)
    {
        const FVector StartLocation = GetOwner()->GetActorLocation();
        const FVector EndLocation = EndPoint->GetActorLocation();
        NavigationBox->RunRRT_WithKDTree(ObjectSize, StartLocation, EndLocation); 
    }
}


void UPlayerMovementComponent::ChangeState(const TSharedPtr<MovementState>& NewState)
{
    if (CurrentState != NewState)
    {
        CurrentState = NewState;
    }
}

void UPlayerMovementComponent::SetPathPoints(const TArray<FVector>& NewPathPoints)
{
    PathPoints = NewPathPoints;
    CurrentPathPointIndex = 0;
    if (PathPoints.Num() > 0)
    {
        SetTargetLocation(PathPoints[0]);
    }
}

void UPlayerMovementComponent::UpdateTargetLocation_Tick()
{
    if (PathPoints.Num() == 0) return; 

    if (CurrentPathPointIndex < PathPoints.Num())
    {
        const FVector& CurrentTargetLocation = PathPoints[CurrentPathPointIndex];
        if (FVector::Dist(GetOwner()->GetActorLocation(), CurrentTargetLocation) < ArrivalThreshold)
        {
            CurrentPathPointIndex++;
            if (CurrentPathPointIndex < PathPoints.Num())
            {
                SetTargetLocation(PathPoints[CurrentPathPointIndex]);
            }
        }
    }
    else
    {
        TargetLocation = FVector::ZeroVector; 
        CurrentVelocity = FVector::ZeroVector; 
    }
}

void UPlayerMovementComponent::UpdateSpeedStage_Tick()
{
    const FVector ActorLocation = GetOwner()->GetActorLocation();
    const float DistanceToTarget = FVector::Dist(ActorLocation, TargetLocation);
    const float DistanceToPastTarget = (CurrentPathPointIndex - 1) > 0 ?
        FVector::Dist(ActorLocation, PathPoints[CurrentPathPointIndex - 1]) : FLT_MAX;
    
    if (DistanceToTarget < DecelerationSpeedThreshold || DistanceToPastTarget < DecelerationSpeedThreshold) 
    {
        ChangeState(Decelerating);
    }
    else
    {
        ChangeState(Accelerating);
    }
}

void UPlayerMovementComponent::SetTargetLocation(const FVector& NewTargetLocation)
{
    TargetLocation = NewTargetLocation;
}
