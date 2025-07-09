#include "HexapodRobot.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"

AHexapodRobot::AHexapodRobot()
{
	PrimaryActorTick.bCanEverTick = true;

	// Create root component
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	// Create body mesh
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(RootComponent);

	// Initialize arrays
	LegRoots.SetNum(6);
	HipMeshes.SetNum(6);
	ThighMeshes.SetNum(6);
	ShinMeshes.SetNum(6);
	FootMeshes.SetNum(6);

	// Create leg components
	for (int32 i = 0; i < 6; ++i)
	{
		// Leg root
		LegRoots[i] = CreateDefaultSubobject<USceneComponent>(*FString::Printf(TEXT("LegRoot_%d"), i));
		LegRoots[i]->SetupAttachment(RootComponent);

		// Hip mesh
		HipMeshes[i] = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("HipMesh_%d"), i));
		HipMeshes[i]->SetupAttachment(LegRoots[i]);

		// Thigh mesh
		ThighMeshes[i] = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("ThighMesh_%d"), i));
		ThighMeshes[i]->SetupAttachment(HipMeshes[i]);

		// Shin mesh
		ShinMeshes[i] = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("ShinMesh_%d"), i));
		ShinMeshes[i]->SetupAttachment(ThighMeshes[i]);

		// Foot mesh
		FootMeshes[i] = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("FootMesh_%d"), i));
		FootMeshes[i]->SetupAttachment(ShinMeshes[i]);
	}

	// Default configuration
	BodyRadius = 15.0f;
	HipLength = 5.0f;
	ThighLength = 12.0f;
	ShinLength = 12.0f;
	LegSpacing = 60.0f;
	AnimationSpeed = 5.0f;
	bSmoothInterpolation = true;

	// Initialize state
	CurrentState.JointAngles.SetNum(6);
	CurrentState.LegPositions.SetNum(6);
	TargetState.JointAngles.SetNum(6);
	TargetState.LegPositions.SetNum(6);

	// Debug settings
	bShowDebugInfo = true;
	bShowLegTrajectories = false;
	bShowSupportPolygon = true;

	// Animation state
	InterpolationAlpha = 0.0f;
	bIsInterpolating = false;
}

void AHexapodRobot::BeginPlay()
{
	Super::BeginPlay();

	InitializeLegs();
	ResetToInitialPosition();
}

void AHexapodRobot::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Update interpolation
	if (bIsInterpolating)
	{
		InterpolationAlpha += DeltaTime * AnimationSpeed;
		if (InterpolationAlpha >= 1.0f)
		{
			InterpolationAlpha = 1.0f;
			bIsInterpolating = false;
		}

		InterpolateState(InterpolationAlpha);
	}

	// Update leg transforms
	UpdateLegTransforms();

	// Draw debug info
	if (bShowDebugInfo)
	{
		DrawDebugInfo();
	}
}

void AHexapodRobot::InitializeLegs()
{
	// Calculate leg attachment points
	LegAttachmentPoints.SetNum(6);
	for (int32 i = 0; i < 6; ++i)
	{
		float Angle = FMath::DegreesToRadians(i * LegSpacing);
		float X = BodyRadius * FMath::Cos(Angle);
		float Y = BodyRadius * FMath::Sin(Angle);
		LegAttachmentPoints[i] = FVector(X, Y, 0.0f);
	}

	// Position leg roots
	for (int32 i = 0; i < 6; ++i)
	{
		LegRoots[i]->SetRelativeLocation(LegAttachmentPoints[i]);
	}

	// Set up leg hierarchy and initial transforms
	for (int32 i = 0; i < 6; ++i)
	{
		// Position hip mesh at leg root
		HipMeshes[i]->SetRelativeLocation(FVector::ZeroVector);

		// Position thigh mesh at hip length
		ThighMeshes[i]->SetRelativeLocation(FVector(HipLength, 0.0f, 0.0f));

		// Position shin mesh at thigh length
		ShinMeshes[i]->SetRelativeLocation(FVector(ThighLength, 0.0f, 0.0f));

		// Position foot mesh at shin length
		FootMeshes[i]->SetRelativeLocation(FVector(ShinLength, 0.0f, 0.0f));
	}
}

void AHexapodRobot::UpdateLegTransforms()
{
	for (int32 i = 0; i < 6; ++i)
	{
		if (i < CurrentState.JointAngles.Num())
		{
			const FJointAngles& Angles = CurrentState.JointAngles[i];

			// Apply hip rotation (around Z-axis)
			FRotator HipRotation(0.0f, 0.0f, FMath::RadiansToDegrees(Angles.Hip));
			HipMeshes[i]->SetRelativeRotation(HipRotation);

			// Apply thigh rotation (around Y-axis)
			FRotator ThighRotation(0.0f, FMath::RadiansToDegrees(Angles.Thigh), 0.0f);
			ThighMeshes[i]->SetRelativeRotation(ThighRotation);

			// Apply shin rotation (around Y-axis)
			FRotator ShinRotation(0.0f, FMath::RadiansToDegrees(Angles.Shin), 0.0f);
			ShinMeshes[i]->SetRelativeRotation(ShinRotation);
		}
	}
}

void AHexapodRobot::InterpolateState(float Alpha)
{
	// Smooth interpolation using smoothstep
	float SmoothAlpha = 3.0f * Alpha * Alpha - 2.0f * Alpha * Alpha * Alpha;

	// Interpolate position
	CurrentState.Position = FMath::Lerp(CurrentState.Position, TargetState.Position, SmoothAlpha);

	// Interpolate rotation
	CurrentState.Rotation = FMath::Lerp(CurrentState.Rotation, TargetState.Rotation, SmoothAlpha);

	// Interpolate joint angles
	for (int32 i = 0; i < CurrentState.JointAngles.Num() && i < TargetState.JointAngles.Num(); ++i)
	{
		CurrentState.JointAngles[i].Hip = FMath::Lerp(CurrentState.JointAngles[i].Hip, TargetState.JointAngles[i].Hip, SmoothAlpha);
		CurrentState.JointAngles[i].Thigh = FMath::Lerp(CurrentState.JointAngles[i].Thigh, TargetState.JointAngles[i].Thigh, SmoothAlpha);
		CurrentState.JointAngles[i].Shin = FMath::Lerp(CurrentState.JointAngles[i].Shin, TargetState.JointAngles[i].Shin, SmoothAlpha);
	}

	// Interpolate leg positions
	for (int32 i = 0; i < CurrentState.LegPositions.Num() && i < TargetState.LegPositions.Num(); ++i)
	{
		CurrentState.LegPositions[i].Position = FMath::Lerp(CurrentState.LegPositions[i].Position, TargetState.LegPositions[i].Position, SmoothAlpha);
	}

	// Update other state values
	CurrentState.GaitPhase = FMath::Lerp(CurrentState.GaitPhase, TargetState.GaitPhase, SmoothAlpha);
	CurrentState.Stability = FMath::Lerp(CurrentState.Stability, TargetState.Stability, SmoothAlpha);
}

void AHexapodRobot::DrawDebugInfo()
{
	UWorld* World = GetWorld();
	if (!World) return;

	// Draw leg positions
	for (int32 i = 0; i < CurrentState.LegPositions.Num(); ++i)
	{
		const FLegPosition& LegPos = CurrentState.LegPositions[i];
		FVector WorldPos = GetActorLocation() + GetActorRotation().RotateVector(LegPos.Position);
		
		// Draw leg position sphere
		FColor Color = LegPos.bIsSupportPhase ? FColor::Green : FColor::Red;
		DrawDebugSphere(World, WorldPos, 2.0f, 8, Color, false, -1.0f, 0, 1.0f);
		
		// Draw leg number
		DrawDebugString(World, WorldPos, FString::Printf(TEXT("%d"), i), nullptr, FColor::White, 0.0f, true);
	}

	// Draw support polygon
	if (bShowSupportPolygon)
	{
		TArray<FVector> SupportPoints;
		for (int32 i = 0; i < CurrentState.LegPositions.Num(); ++i)
		{
			if (CurrentState.LegPositions[i].bIsSupportPhase)
			{
				FVector WorldPos = GetActorLocation() + GetActorRotation().RotateVector(CurrentState.LegPositions[i].Position);
				SupportPoints.Add(WorldPos);
			}
		}

		if (SupportPoints.Num() >= 3)
		{
			// Draw support polygon lines
			for (int32 i = 0; i < SupportPoints.Num(); ++i)
			{
				FVector Start = SupportPoints[i];
				FVector End = SupportPoints[(i + 1) % SupportPoints.Num()];
				DrawDebugLine(World, Start, End, FColor::Yellow, false, -1.0f, 0, 2.0f);
			}
		}
	}

	// Draw body center
	DrawDebugSphere(World, GetActorLocation(), 5.0f, 8, FColor::Blue, false, -1.0f, 0, 1.0f);

	// Draw movement direction
	if (CurrentState.bIsMoving)
	{
		FVector Direction = FVector(FMath::Cos(CurrentState.Rotation), FMath::Sin(CurrentState.Rotation), 0.0f);
		FVector End = GetActorLocation() + Direction * 20.0f;
		DrawDebugArrow(World, GetActorLocation(), End, 5.0f, FColor::Orange, false, -1.0f, 0, 3.0f);
	}
}

void AHexapodRobot::UpdateState(const FHexapodState& NewState)
{
	TargetState = NewState;
	InterpolationAlpha = 0.0f;
	bIsInterpolating = true;
}

void AHexapodRobot::SetJointAngles(int32 LegIndex, const FJointAngles& Angles)
{
	if (LegIndex >= 0 && LegIndex < CurrentState.JointAngles.Num())
	{
		CurrentState.JointAngles[LegIndex] = Angles;
		UpdateLegTransforms();
	}
}

FJointAngles AHexapodRobot::GetJointAngles(int32 LegIndex) const
{
	if (LegIndex >= 0 && LegIndex < CurrentState.JointAngles.Num())
	{
		return CurrentState.JointAngles[LegIndex];
	}
	return FJointAngles();
}

void AHexapodRobot::SetLegPosition(int32 LegIndex, const FVector& Position, bool bIsSupportPhase)
{
	if (LegIndex >= 0 && LegIndex < CurrentState.LegPositions.Num())
	{
		CurrentState.LegPositions[LegIndex] = FLegPosition(Position, bIsSupportPhase);
	}
}

FVector AHexapodRobot::GetLegPosition(int32 LegIndex) const
{
	if (LegIndex >= 0 && LegIndex < CurrentState.LegPositions.Num())
	{
		return CurrentState.LegPositions[LegIndex].Position;
	}
	return FVector::ZeroVector;
}

void AHexapodRobot::ResetToInitialPosition()
{
	// Reset to initial state
	CurrentState.Position = FVector::ZeroVector;
	CurrentState.Rotation = 0.0f;
	CurrentState.GaitPhase = 0.0f;
	CurrentState.bIsMoving = false;
	CurrentState.Stability = 0.0f;

	// Reset joint angles to standing position
	for (int32 i = 0; i < CurrentState.JointAngles.Num(); ++i)
	{
		CurrentState.JointAngles[i] = FJointAngles(0.0f, 0.2f, -0.4f);
	}

	// Reset leg positions
	for (int32 i = 0; i < CurrentState.LegPositions.Num(); ++i)
	{
		CurrentState.LegPositions[i] = FLegPosition(LegAttachmentPoints[i], true);
	}

	// Copy to target state
	TargetState = CurrentState;

	// Reset interpolation
	InterpolationAlpha = 0.0f;
	bIsInterpolating = false;

	// Update transforms
	UpdateLegTransforms();
}

void AHexapodRobot::SetGaitPattern(const FString& PatternName)
{
	// This would be implemented to communicate with the Python kinematics engine
	// For now, just log the pattern change
	UE_LOG(LogTemp, Log, TEXT("Gait pattern changed to: %s"), *PatternName);
} 