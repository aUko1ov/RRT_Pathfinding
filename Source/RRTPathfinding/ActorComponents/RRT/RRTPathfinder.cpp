
#include "RRTPathfinder.h"

#include "Components/LineBatchComponent.h"
#include "Kismet/GameplayStatics.h"
#include "RRTPathfinding/KDTree/KDTree.h"
#include "RRTPathfinding/Utils/SFunction.h"

URRTPathfinder::URRTPathfinder()
{
	LineBatchComponent = CreateDefaultSubobject<ULineBatchComponent>(TEXT("LineBatchComponent"));
	NeighborRadius = Size_RRT_Step;
	GoalThreshold = Size_RRT_Step;
}

void URRTPathfinder::FullObstacleActors()
{
	TArray<AActor*> ObstaclesRaw;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), ObstaclesRaw);
	const AActor* TransparentRectangleActor = GetOwner();
	ObstacleActors = SFunction::FindStaticMeshesInsideSearchArea(TransparentRectangleActor, ObstaclesRaw);

}

void URRTPathfinder::RunRRTStar(const FVector& LowerCorner, const FVector& UpperOppositeCorner,
	const float ObjectSize,	const FVector& StartLocation, const FVector& EndLocation)
{
	TSharedPtr<FNodeRRT> NearestToEndNode = MakeShared<FNodeRRT>(StartLocation);

	TArray<TSharedPtr<FNodeRRT>> Tree;
	Tree.Add(NearestToEndNode);

	float NearestToEndDistance = FVector::Dist(StartLocation, EndLocation);

	for (int32 i = 0; i < MaxIterations; ++i)
	{
		FVector RandomPoint = FMath::RandPointInBox(FBox(LowerCorner, UpperOppositeCorner));
		if (i % GoalProbabilityEach == 0)
		{
			RandomPoint = EndLocation;
		}

		const TSharedPtr<FNodeRRT> NearestNode = SFunction::FindNearestNode(Tree, RandomPoint);
		FVector Direction = (RandomPoint - NearestNode->PointLocation).GetSafeNormal();
		FVector NewPoint = NearestNode->PointLocation + Direction * Size_RRT_Step;

		TSharedPtr<FNodeRRT> NewNode = SFunction::AddNewNodeWithRadius(Tree, NearestNode, NewPoint, ObstacleActors, ObjectSize);
		if (NewNode.IsValid())
		{
			const float DistanceToEnd = FVector::Dist(NewNode->PointLocation, EndLocation);
			if (DistanceToEnd < NearestToEndDistance)
			{
				NearestToEndNode = NewNode;
				NearestToEndDistance = DistanceToEnd;
			}

			if (NearestToEndDistance < GoalThreshold)
			{
				break;
			}
		}
	}
	
	TArray<TSharedPtr<FNodeRRT>> Path = SFunction::ExtractPath(NearestToEndNode);

	DrawPath(Path, FLinearColor(0.0f, 1.0f, 0.0f));

	SFunction::SmoothPath(Path, ObstacleActors, ObjectSize);

	DrawPath(Path, FLinearColor(1.0f, 0.0f, 0.0f));
	DrawAllPaths(Tree, FLinearColor(0.8f, 0.8f, 0.8f));

	GeneratePathPlayer(Path);	
}


void URRTPathfinder::RunRRT_WithKDTree(const FVector& LowerCorner, const FVector& UpperOppositeCorner,
	const float ObjectSize,	const FVector& StartLocation, const FVector& EndLocation)
{
	UKDTree* KDTree = NewObject<UKDTree>(this);
	TArray<TSharedPtr<FNodeRRT>> Tree;

	TSharedPtr<FNodeRRT> NearestToEndNode = MakeShared<FNodeRRT>(StartLocation);
	Tree.Add(NearestToEndNode);
	KDTree->Insert(StartLocation);

	float NearestToEndDistance = FVector::Dist(StartLocation, EndLocation);

	for (int32 i = 0; i < MaxIterations; ++i)
	{
		FVector RandomPoint = FMath::RandPointInBox(FBox(LowerCorner, UpperOppositeCorner));
		if (i % GoalProbabilityEach == 0)
		{
			RandomPoint = EndLocation;
		}

		FVector NearestPoint = KDTree->FindNearest(RandomPoint);

		TSharedPtr<FNodeRRT> NearestNode = *Tree.FindByPredicate([&NearestPoint](const TSharedPtr<FNodeRRT>& Node) {
			return Node->PointLocation == NearestPoint; // нужное условие
			});

		if (!NearestNode) continue;

		FVector Direction = (RandomPoint - NearestNode->PointLocation).GetSafeNormal();
		FVector NewPoint = NearestNode->PointLocation + Direction * Size_RRT_Step;

		bool bCollision = false;
		for (const AActor* Obstacle : ObstacleActors)
		{
			if (SFunction::IsCollidingWithSphere(NearestNode->PointLocation, NewPoint, Obstacle, ObjectSize))
			{
				bCollision = true;
				break;
			}
		}

		if (!bCollision)
		{
			TSharedPtr<FNodeRRT> NewNode = MakeShared<FNodeRRT>(NewPoint, NearestNode);
			Tree.Add(NewNode);
			KDTree->Insert(NewPoint);

			const float DistanceToEnd = FVector::Dist(NewPoint, EndLocation);
			if (DistanceToEnd < NearestToEndDistance)
			{
				NearestToEndNode = NewNode;
				NearestToEndDistance = DistanceToEnd;
			}

			if (NearestToEndDistance < GoalThreshold) // Закончили итерацию поиска пути здесь
			{
				break;
			}
		}
	}

	TArray<TSharedPtr<FNodeRRT>> Path = SFunction::ExtractPath(NearestToEndNode);

	DrawPath(Path, FLinearColor(0.0f, 1.0f, 0.0f));

	SFunction::SmoothPath(Path, ObstacleActors, ObjectSize);

	DrawPath(Path, FLinearColor(1.0f, 0.0f, 0.0f));
	DrawAllPaths(Tree, FLinearColor(0.8f, 0.8f, 0.8f));

	GeneratePathPlayer(Path);	
}

void URRTPathfinder::DrawPath(const TArray<TSharedPtr<FNodeRRT>>& Path, FLinearColor LineColor) const
{
	constexpr float Lifetime = -1.0f; // Время жизни линии, -1 для бесконечности

	if (Path.Num() > 1) // Убедимся, что в пути есть хотя бы две точки для соединения
	{
		for (int32 i = 0; i < Path.Num() - 1; ++i)
		{
			if (Path[i] && Path[i + 1]) // Проверяем, что текущий и следующий узлы существуют
			{
				constexpr bool bDepthIsForeground = false;
				constexpr float LineThickness = 15.0f;

				// Получаем точки из текущего и следующего узлов
				const FVector& StartPointInner = Path[i]->PointLocation;
				const FVector& EndPointInner = Path[i + 1]->PointLocation;

				// Рисуем линию между точками
				LineBatchComponent->DrawLine(StartPointInner, EndPointInner, LineColor, SDPG_World, LineThickness, Lifetime, bDepthIsForeground);
			}
		}
	}
}

void URRTPathfinder::DrawAllPaths(const TArray<TSharedPtr<FNodeRRT>>& Tree, FLinearColor LineColor) const
{
	constexpr float Lifetime = -1.0f;

	for (const TSharedPtr<FNodeRRT>& Node : Tree)
	{
		if (Node && Node->ParentNode)
		{
			constexpr float LineThickness = 5.0f;
			constexpr bool bDepthIsForeground = false;

			const FVector& StartPointInner = Node->PointLocation;
			const FVector& EndPointInner = Node->ParentNode->PointLocation;

			LineBatchComponent->DrawLine(StartPointInner, EndPointInner, LineColor, SDPG_World, LineThickness, Lifetime, bDepthIsForeground);
		}
	}
}

void URRTPathfinder::GeneratePathPlayer(const TArray<TSharedPtr<FNodeRRT>>& Path)
{
	const TArray<FVector> PathPoints = SFunction::ConvertNodePathToVectorPath(Path);

	OnPathCalculated.Broadcast(PathPoints);
}
