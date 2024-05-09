#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavigationBox.generated.h"

class URRTPathfinder;

UCLASS()
class RRTPATHFINDING_API ANavigationBox : public AActor
{
	GENERATED_BODY()

	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathCalculated, const TArray<FVector>&, NewPathPoints);
	
public:    
	ANavigationBox();
	void RunRRT_WithKDTree(const float ObjectSize, const FVector& StartLocation, const FVector& EndLocation);

	UPROPERTY(VisibleAnywhere, Category = "[RRT]")
	TObjectPtr<URRTPathfinder> RRTPathfinder;

	FOnPathCalculated OnPathCalculated;
	
protected:
	void DetermineMeshCorners();

	UFUNCTION()
	void BroadcastPath(const TArray<FVector>& PathPoints);

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UStaticMeshComponent> RectangleMesh;

private:
	FVector LowerCorner;
	FVector UpperOppositeCorner;

};
