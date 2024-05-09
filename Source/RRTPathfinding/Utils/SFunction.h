
#pragma once

#include "CoreMinimal.h"
#include "RRTPathfinding/Structs/RRT/NodeRRT.h"


struct AABB {
	FVector Min;
	FVector Max;
	
	AABB(const FVector& min, const FVector& max) : Min(min), Max(max) {}

	bool Intersects(const AABB& other) const {
		return (Min.X <= other.Max.X && Max.X >= other.Min.X) &&
			   (Min.Y <= other.Max.Y && Max.Y >= other.Min.Y) &&
			   (Min.Z <= other.Max.Z && Max.Z >= other.Min.Z);
	}
};

class FNode;

class SFunction
{
public:
	static bool IsCollidingWithSphere(const FVector& Start, const FVector& End, const AActor* ObstacleActor, float Radius);
	
	static AABB GetAABBFromStaticMesh(const AActor* Actor);

	static TArray<AActor*> FindStaticMeshesInsideSearchArea(const AActor* SearchArea, const TArray<AActor*>& AllActors);

	static TSharedPtr<FNodeRRT> FindNearestNode(const TArray<TSharedPtr<FNodeRRT>>& Tree, const FVector& Point);

	static TSharedPtr<FNodeRRT> AddNewNodeWithRadius(TArray<TSharedPtr<FNodeRRT>>& Tree, TSharedPtr<FNodeRRT> NearestNode, const FVector& NewPoint, const TArray<AActor*>& Obstacles, float RadiusAgent);


	static TArray<TSharedPtr<FNodeRRT>> ExtractPath(const TSharedPtr<FNodeRRT>& EndNode);
	
	static void SmoothPath(TArray<TSharedPtr<FNodeRRT>>& Path, const TArray<AActor*>& Obstacles, float RadiusAgent);

	static TArray<FVector> ConvertNodePathToVectorPath(const TArray<TSharedPtr<FNodeRRT>>& NodePath);
};
