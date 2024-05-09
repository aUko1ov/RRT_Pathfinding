
#include "MoveStateMachine.h"

#include "RRTPathfinding/ActorComponents/Movement/PlayerMovementComponent.h"

// Функция ManualSlerp для интерполяции между двумя кватернионами A и B с заданным коэффициентом SlerpFactor.
FQuat ManualSlerp(const FQuat& A, const FQuat& B, float SlerpFactor)
{
    // Вычисляем косинус угла theta между A и B, используя скалярное произведение кватернионов.
    float cosTheta = A | B;
    // Если cosTheta < 0, интерполяция будет происходить по длинному пути на сфере.
    const float Sign = (cosTheta >= 0.f) ? 1.f : -1.f;
    cosTheta *= Sign;

    float scale0, scale1;

    // Осуществляем интерполяцию
    if (cosTheta < 0.9999f) // Используем slerp для больших углов
        {
        // Вычисляем угол theta через арккосинус от cosTheta
        const float theta = FMath::Acos(cosTheta);
        // Вычисляем синус угла theta
        const float sinTheta = FMath::Sin(theta);
        // Вычисляем масштабные коэффициенты для кватернионов A и B
        scale0 = FMath::Sin((1.f - SlerpFactor) * theta) / sinTheta;
        scale1 = FMath::Sin(SlerpFactor * theta) / sinTheta;
        }
    else // Для меньших углов используем линейную интерполяцию
        {
        scale0 = 1.f - SlerpFactor;
        scale1 = SlerpFactor;
        }
    // Модифицируем scale1, учитывая знак
    scale1 *= Sign;

    // Создаем результатирующий кватернион
    FQuat result;
    result.X = scale0 * A.X + scale1 * B.X;
    result.Y = scale0 * A.Y + scale1 * B.Y;
    result.Z = scale0 * A.Z + scale1 * B.Z;
    result.W = scale0 * A.W + scale1 * B.W;

    // Нормализуем результатирующий кватернион и возвращаем его
    return result.GetNormalized();
}

void AcceleratingState::Movement_Tick(float DeltaTime, UPlayerMovementComponent& MovementComponent)
{
    if (!MovementComponent.TargetLocation.IsNearlyZero())
    {
        const FVector ActorLocation = MovementComponent.GetOwner()->GetActorLocation();
        const FVector DirectionToTarget = (MovementComponent.TargetLocation - ActorLocation).GetSafeNormal();
        const FQuat TargetQuat = DirectionToTarget.ToOrientationQuat();
        const FQuat CurrentQuat = MovementComponent.GetOwner()->GetActorQuat();
        
        const float Alpha = FMath::Clamp(MovementComponent.TurnSpeed * DeltaTime, 0.0f, 1.0f);
        FQuat NewQuat = ManualSlerp(CurrentQuat, TargetQuat, Alpha);
        
        MovementComponent.GetOwner()->SetActorRotation(NewQuat.Rotator());

        const FVector NewDirection = FRotationMatrix(NewQuat.Rotator()).GetUnitAxis(EAxis::X);
        const FVector TargetVelocity = NewDirection * MovementComponent.MaxSpeed;
        FVector NewVelocity = MovementComponent.CurrentVelocity + (TargetVelocity - MovementComponent.CurrentVelocity) * MovementComponent.InterpolationChangeSpeed * DeltaTime;
        
        if (NewVelocity.Size() > MovementComponent.MaxSpeed)
        {
            NewVelocity = NewVelocity.GetSafeNormal() * MovementComponent.MaxSpeed;
        }

        const FVector NewLocation = ActorLocation + NewVelocity * DeltaTime;
        MovementComponent.GetOwner()->SetActorLocation(NewLocation);
        MovementComponent.CurrentVelocity = NewVelocity;
    }
}

void DeceleratingState::Movement_Tick(float DeltaTime, UPlayerMovementComponent& MovementComponent)
{
    if (!MovementComponent.TargetLocation.IsNearlyZero())
    {
        const FVector ActorLocation = MovementComponent.GetOwner()->GetActorLocation();
        const FVector DirectionToTarget = (MovementComponent.TargetLocation - ActorLocation).GetSafeNormal();
        const FQuat TargetQuat = DirectionToTarget.ToOrientationQuat();
        const FQuat CurrentQuat = MovementComponent.GetOwner()->GetActorQuat();
        
        const float Alpha = FMath::Clamp(MovementComponent.TurnSpeedOnDeceleration * DeltaTime, 0.0f, 1.0f);
        FQuat NewQuat = ManualSlerp(CurrentQuat, TargetQuat, Alpha);
        
        MovementComponent.GetOwner()->SetActorRotation(NewQuat.Rotator());

        const FVector NewDirection = FRotationMatrix(NewQuat.Rotator()).GetUnitAxis(EAxis::X);
        const FVector TargetVelocity = NewDirection * MovementComponent.MinSpeed_OnDeceleration;
        const FVector NewVelocity = MovementComponent.CurrentVelocity + (TargetVelocity - MovementComponent.CurrentVelocity) * MovementComponent.InterpolationChangeSpeed * DeltaTime;

        const FVector NewLocation = ActorLocation + NewVelocity * DeltaTime;
        MovementComponent.GetOwner()->SetActorLocation(NewLocation);
        MovementComponent.CurrentVelocity = NewVelocity;
    }
}

