
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RRTPathfinder.generated.h"


struct FNodeRRT;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class RRTPATHFINDING_API URRTPathfinder : public UActorComponent
{
	GENERATED_BODY()

	URRTPathfinder();

	void DrawPath(const TArray<TSharedPtr<FNodeRRT>>& Path, FLinearColor LineColor) const;
	void DrawAllPaths(const TArray<TSharedPtr<FNodeRRT>>& Tree, FLinearColor LineColor) const;

	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathCalculated, const TArray<FVector>&, NewPathPoints);

public:
	FOnPathCalculated OnPathCalculated;

	UPROPERTY()
	TArray<AActor*> ObstacleActors;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<ULineBatchComponent> LineBatchComponent;

	UPROPERTY(EditAnywhere, Category = "[RRT]")
	float Size_RRT_Step = 50.f;
    
	UPROPERTY(EditAnywhere, Category = "[RRT]")
	int32 MaxIterations = 10000;
    
	void RunRRT_WithKDTree(const FVector& LowerCorner, const FVector& UpperOppositeCorner, const float ObjectSize,
		const FVector& StartLocation, const FVector& EndLocation);

	void GeneratePathPlayer(const TArray<TSharedPtr<FNodeRRT>>& Path);

	void FullObstacleActors();
	void RunRRTStar(const FVector& LowerCorner, const FVector& UpperOppositeCorner, const float ObjectSize,
		const FVector& StartLocation, const FVector& EndLocation);

private:
	float NeighborRadius;
	float GoalThreshold;
	int32 GoalProbabilityEach = 10;
};

