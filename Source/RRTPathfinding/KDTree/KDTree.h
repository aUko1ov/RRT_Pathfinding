#pragma once

#include <stack>

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "KDTree.generated.h"


USTRUCT(BlueprintType)
struct FKDTreeNode
{
    GENERATED_BODY()
	
    FVector PointLocation; // Точка в пространстве, представленная узлом
    int32 Axis; // Ось, по которой производится разделение
    TSharedPtr<FKDTreeNode> LeftTreeNode; // Левое поддерево
    TSharedPtr<FKDTreeNode> RightTreeNode; // Правое поддерево

    FKDTreeNode() : PointLocation(FVector::ZeroVector), Axis(0), LeftTreeNode(nullptr), RightTreeNode(nullptr)
    {
    }
	
    FKDTreeNode(const FVector& InPoint, int32 InAxis)
        : PointLocation(InPoint), Axis(InAxis), LeftTreeNode(nullptr), RightTreeNode(nullptr)
    {
    }
};


UCLASS()
class RRTPATHFINDING_API UKDTree : public UObject
{
    GENERATED_BODY()
    
    TSharedPtr<FKDTreeNode> Root;
    int32 Dimensions;

public:
    UKDTree() : Root(nullptr), Dimensions(3) {}
    explicit UKDTree(int32 InDimensions) : Root(nullptr), Dimensions(InDimensions) {}

    void Insert(const FVector& Point)
    {
        if (!Root)
        {
            Root = MakeShared<FKDTreeNode>(Point, 0);
            return;
        }

        TSharedPtr<FKDTreeNode> CurrentNode = Root;
        TSharedPtr<FKDTreeNode> ParentNode = nullptr;
        bool insertToLeft = true;

        int32 Depth = 0;
        while (CurrentNode)
        {
            const int32 Axis = Depth % Dimensions;
            ParentNode = CurrentNode; 

            if (Point[Axis] < CurrentNode->PointLocation[Axis])
            {
                CurrentNode = CurrentNode->LeftTreeNode;
                insertToLeft = true;
            }
            else
            {
                CurrentNode = CurrentNode->RightTreeNode;
                insertToLeft = false;
            }
            Depth++;
        }
        
        if (insertToLeft)
        {
            ParentNode->LeftTreeNode = MakeShared<FKDTreeNode>(Point, Depth % Dimensions);
        }
        else
        {
            ParentNode->RightTreeNode = MakeShared<FKDTreeNode>(Point, Depth % Dimensions);
        }
    }


    FVector FindNearest(const FVector& TargetPoint) const
    {
        if (!Root) return FVector::ZeroVector;

        std::stack<TSharedPtr<FKDTreeNode>> stack;
        TSharedPtr<FKDTreeNode> BestNode = nullptr;
        float BestDistance = TNumericLimits<float>::Max();

        stack.push(Root);
        while (!stack.empty())
        {
            const TSharedPtr<FKDTreeNode> CurrentNode = stack.top();
            stack.pop();

            if (!CurrentNode) continue;

            const float CurrentDistance = FVector::Dist(CurrentNode->PointLocation, TargetPoint);
            if (CurrentDistance < BestDistance)
            {
                BestDistance = CurrentDistance;
                BestNode = CurrentNode;
            }

            const int32 Axis = CurrentNode->Axis;
            TSharedPtr<FKDTreeNode> NextNode = TargetPoint[Axis] < CurrentNode->PointLocation[Axis] ? CurrentNode->LeftTreeNode : CurrentNode->RightTreeNode;
            TSharedPtr<FKDTreeNode> OtherNode = TargetPoint[Axis] < CurrentNode->PointLocation[Axis] ? CurrentNode->RightTreeNode : CurrentNode->LeftTreeNode;

            if (NextNode) stack.push(NextNode);
            if (FMath::Abs(TargetPoint[Axis] - CurrentNode->PointLocation[Axis]) < BestDistance)
            {
                if (OtherNode) stack.push(OtherNode);
            }
        }

        return BestNode ? BestNode->PointLocation : FVector::ZeroVector;
    }
};
