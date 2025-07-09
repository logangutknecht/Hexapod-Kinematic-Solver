#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "HexapodRobot.generated.h"

USTRUCT(BlueprintType)
struct FJointAngles
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Hip;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Thigh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Shin;

	FJointAngles()
	{
		Hip = 0.0f;
		Thigh = 0.0f;
		Shin = 0.0f;
	}

	FJointAngles(float InHip, float InThigh, float InShin)
	{
		Hip = InHip;
		Thigh = InThigh;
		Shin = InShin;
	}
};

USTRUCT(BlueprintType)
struct FLegPosition
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bIsSupportPhase;

	FLegPosition()
	{
		Position = FVector::ZeroVector;
		bIsSupportPhase = true;
	}

	FLegPosition(const FVector& InPosition, bool InIsSupportPhase)
	{
		Position = InPosition;
		bIsSupportPhase = InIsSupportPhase;
	}
};

USTRUCT(BlueprintType)
struct FHexapodState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Rotation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<FJointAngles> JointAngles;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<FLegPosition> LegPositions;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float GaitPhase;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bIsMoving;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Stability;

	FHexapodState()
	{
		Position = FVector::ZeroVector;
		Rotation = 0.0f;
		GaitPhase = 0.0f;
		bIsMoving = false;
		Stability = 0.0f;
	}
};

UCLASS(BlueprintType, Blueprintable)
class HEXAPODSIMULATION_API AHexapodRobot : public AActor
{
	GENERATED_BODY()
	
public:	
	AHexapodRobot();

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

public:
	// Body mesh
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hexapod")
	UStaticMeshComponent* BodyMesh;

	// Leg components
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hexapod")
	TArray<USceneComponent*> LegRoots;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hexapod")
	TArray<UStaticMeshComponent*> HipMeshes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hexapod")
	TArray<UStaticMeshComponent*> ThighMeshes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hexapod")
	TArray<UStaticMeshComponent*> ShinMeshes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hexapod")
	TArray<UStaticMeshComponent*> FootMeshes;

	// Configuration
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Configuration")
	float BodyRadius;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Configuration")
	float HipLength;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Configuration")
	float ThighLength;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Configuration")
	float ShinLength;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Configuration")
	float LegSpacing;

	// Animation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Animation")
	float AnimationSpeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Animation")
	bool bSmoothInterpolation;

	// Current state
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "State")
	FHexapodState CurrentState;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "State")
	FHexapodState TargetState;

	// Blueprint events
	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	void UpdateState(const FHexapodState& NewState);

	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	void SetJointAngles(int32 LegIndex, const FJointAngles& Angles);

	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	FJointAngles GetJointAngles(int32 LegIndex) const;

	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	void SetLegPosition(int32 LegIndex, const FVector& Position, bool bIsSupportPhase);

	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	FVector GetLegPosition(int32 LegIndex) const;

	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	void ResetToInitialPosition();

	UFUNCTION(BlueprintCallable, Category = "Hexapod")
	void SetGaitPattern(const FString& PatternName);

	// Debug visualization
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowDebugInfo;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowLegTrajectories;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowSupportPolygon;

protected:
	// Internal methods
	void InitializeLegs();
	void UpdateLegTransforms();
	void InterpolateState(float Alpha);
	void DrawDebugInfo();

	// Leg attachment points
	TArray<FVector> LegAttachmentPoints;

	// Animation state
	float InterpolationAlpha;
	bool bIsInterpolating;
}; 