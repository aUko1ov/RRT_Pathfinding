
#include "SFunction.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/TargetPoint.h"


bool SFunction::IsCollidingWithSphere(const FVector& Start, const FVector& End, const AActor* ObstacleActor, float AgentRadius)
{
    if (!ObstacleActor || !ObstacleActor->GetWorld())
    {
        return false; 
    }

    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = true;
    QueryParams.bReturnPhysicalMaterial = false;
    
    const bool bHit = ObstacleActor->GetWorld()->SweepSingleByChannel(
        HitResult, // Результат трассировки
        Start, // Начальная точка
        End, // Конечная точка
        FQuat::Identity, // Без вращения
        ECC_Visibility, // Используем канал трассировки Visibility
        FCollisionShape::MakeSphere(AgentRadius), // Форма сферы с заданным радиусом
        QueryParams // Параметры трассировки
    );
    
    if (bHit && HitResult.GetActor() != nullptr && HitResult.GetActor() != ObstacleActor)
    {
        return true; 
    }

    return false; 
}



AABB SFunction::GetAABBFromStaticMesh(const AActor* Actor) {
    if (!Actor) return AABB(FVector::ZeroVector, FVector::ZeroVector);

    // Получаем ограничивающий объем Static Mesh
    const FBox Bounds = Actor->GetComponentsBoundingBox(true);
    return AABB(Bounds.Min, Bounds.Max);
}

TArray<AActor*> SFunction::FindStaticMeshesInsideSearchArea(const AActor* SearchArea, const TArray<AActor*>& AllActors)
{
    TArray<AActor*> InsideActors;

    const AABB SearchAreaAABB = GetAABBFromStaticMesh(SearchArea);
    
    for (AActor* Actor : AllActors) {
        if (Actor == SearchArea || Actor->IsA<ATargetPoint>()) continue;

        AABB ActorAABB = GetAABBFromStaticMesh(Actor);
        if (SearchAreaAABB.Intersects(ActorAABB)) {
            InsideActors.Add(Actor);
        }
    }

    return InsideActors;
}

// Нахождение ближайшего узла
TSharedPtr<FNodeRRT> SFunction::FindNearestNode(const TArray<TSharedPtr<FNodeRRT>>& Tree, const FVector& Point)
{
    TSharedPtr<FNodeRRT> NearestNode = Tree[0];
    float MinDistance = FVector::Dist(NearestNode->PointLocation, Point);

    for (const TSharedPtr<FNodeRRT>& Node : Tree)
    {
        const float Distance = FVector::Dist(Node->PointLocation, Point);
        if (Distance < MinDistance)
        {
            NearestNode = Node;
            MinDistance = Distance;
        }
    }

    return NearestNode;
}

TSharedPtr<FNodeRRT> SFunction::AddNewNodeWithRadius(TArray<TSharedPtr<FNodeRRT>>& Tree, TSharedPtr<FNodeRRT> NearestNode,
    const FVector& NewPoint, const TArray<AActor*>& Obstacles, float RadiusAgent)
{
    for (const AActor* ObstacleActor : Obstacles)
    {
        if (IsCollidingWithSphere(NearestNode->PointLocation, NewPoint, ObstacleActor, RadiusAgent))
        {
            return nullptr;
        }
    }

    TSharedPtr<FNodeRRT> NewNode = MakeShared<FNodeRRT>(NewPoint, NearestNode);
    Tree.Add(NewNode);
    return NewNode;
}


TArray<TSharedPtr<FNodeRRT>> SFunction::ExtractPath(const TSharedPtr<FNodeRRT>& EndNode)
{
    TArray<TSharedPtr<FNodeRRT>> Path;
    TSharedPtr<FNodeRRT> CurrentNode = EndNode;

    while (CurrentNode != nullptr)
    {
        Path.Insert(CurrentNode, 0);
        CurrentNode = CurrentNode->ParentNode;
    }

    return Path;
}


void SFunction::SmoothPath(TArray<TSharedPtr<FNodeRRT>>& Path, const TArray<AActor*>& Obstacles, float RadiusAgent)
{
    bool Change = true;
    constexpr int32 MaxIterations = 100;
    int32 Iterations = 0;

    while (Change && Iterations < MaxIterations)
    {
        Change = false;
        Iterations++;

        for (int32 i = 0; i < Path.Num() - 2; ++i)
        {
            for (int32 j = Path.Num() - 1; j > i + 1; --j)
            {
                bool IsNotColliding = true;
                for (const AActor* ObstacleActor : Obstacles)
                {
                    if (IsCollidingWithSphere(Path[i]->PointLocation, Path[j]->PointLocation, ObstacleActor, RadiusAgent))
                    {
                        IsNotColliding = false;
                        break;
                    }
                }

                if (IsNotColliding)
                {
                    // Удалить узлы между i и j
                    Path.RemoveAt(i + 1, j - i - 1);
                    Change = true;
                    break;
                }
            }
        }
    }
}

TArray<FVector> SFunction::ConvertNodePathToVectorPath(const TArray<TSharedPtr<FNodeRRT>>& NodePath)
{
    TArray<FVector> VectorPath;
    VectorPath.Reserve(NodePath.Num()); 

    for (const TSharedPtr<FNodeRRT>& Node : NodePath)
    {
        if (Node.IsValid()) 
        {
            VectorPath.Add(Node->PointLocation);
        }
    }

    return VectorPath;
}