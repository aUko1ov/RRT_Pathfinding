#pragma once

#include "CoreMinimal.h"
#include "Templates/SharedPointer.h"
#include "NodeRRT.generated.h"

USTRUCT(BlueprintType)
struct FNodeRRT
{
	GENERATED_BODY()
	
	FVector PointLocation; 
	TSharedPtr<FNodeRRT> ParentNode; 
	
	FNodeRRT()
		: PointLocation(FVector::ZeroVector), ParentNode(nullptr)
	{
	}
	
	FNodeRRT(const FVector& InPoint)
		: PointLocation(InPoint), ParentNode(nullptr)
	{
	}
	
	FNodeRRT(const FVector& InPoint, const TSharedPtr<FNodeRRT>& InParent)
		: PointLocation(InPoint), ParentNode(InParent)
	{
	}
};
