
#include "NavigationBox.h"

#include "RRTPathfinding/ActorComponents/RRT/RRTPathfinder.h"


ANavigationBox::ANavigationBox()
{
	PrimaryActorTick.bCanEverTick = true;
	
	RectangleMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RectangleMesh"));
	RootComponent = RectangleMesh;
	RectangleMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	RRTPathfinder = CreateDefaultSubobject<URRTPathfinder>(TEXT("RRTPathfinder"));
}

void ANavigationBox::RunRRT_WithKDTree(const float ObjectSize, const FVector& StartLocation, const FVector& EndLocation)
{    
	DetermineMeshCorners();

	RRTPathfinder->OnPathCalculated.AddDynamic(this, &ANavigationBox::BroadcastPath);

	RRTPathfinder->FullObstacleActors();
	RRTPathfinder->RunRRTStar(LowerCorner, UpperOppositeCorner, ObjectSize, StartLocation, EndLocation); 
}


void ANavigationBox::DetermineMeshCorners()
{
	if (!RectangleMesh || !RectangleMesh->GetStaticMesh()) return;

	const FBoxSphereBounds LocalBounds = RectangleMesh->GetStaticMesh()->GetBounds();
	const FVector LocalMin = LocalBounds.Origin - LocalBounds.BoxExtent;
	const FVector LocalMax = LocalBounds.Origin + LocalBounds.BoxExtent;

	const FVector WorldMin = RectangleMesh->GetComponentTransform().TransformPosition(LocalMin);
	const FVector WorldMax = RectangleMesh->GetComponentTransform().TransformPosition(LocalMax);
	
	LowerCorner = FVector(WorldMin.X, WorldMin.Y, WorldMin.Z);
	UpperOppositeCorner = FVector(WorldMax.X, WorldMax.Y, WorldMax.Z);
}

void ANavigationBox::BroadcastPath(const TArray<FVector>& PathPoints)
{
	OnPathCalculated.Broadcast(PathPoints);
}
