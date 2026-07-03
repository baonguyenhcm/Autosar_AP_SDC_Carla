// AvUdpReceiverActor.h
// Live UDP receiver: drives the ego from a *running* pipeline_demo built with
// -DAV_UDP=ON, which streams one CSV-style datagram per step to 127.0.0.1:9999.
//
// Same coordinate conversion as AvReplayActor (av-stack RH metres -> UE4 LH cm):
//   UE_X = x*100,  UE_Y = -y*100,  UE_YawDeg = -yaw*180/PI.
//
// Requires the "Sockets" and "Networking" modules in your <Project>.Build.cs:
//   PublicDependencyModuleNames.AddRange(new[]{ "Sockets", "Networking" });
//
// Compile into your primary game module (no PROJECT_API macro).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AvUdpReceiverActor.generated.h"

class UStaticMeshComponent;
class UStaticMesh;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class ACameraActor;
class FSocket;

UCLASS()
class AAvUdpReceiverActor : public AActor
{
    GENERATED_BODY()

public:
    AAvUdpReceiverActor();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void Tick(float DeltaSeconds) override;

    UPROPERTY(EditAnywhere, Category = "AV Live")
    int32 Port = 9999;

    UPROPERTY(EditAnywhere, Category = "AV Live")
    float MetersToUU = 100.f;

    // Visual smoothing toward the latest received pose (higher = snappier).
    UPROPERTY(EditAnywhere, Category = "AV Live")
    float SmoothingSpeed = 12.f;

    UPROPERTY(EditAnywhere, Category = "AV Live")
    bool bShowGhost = true;

    UPROPERTY(EditAnywhere, Category = "AV Live")
    bool bSpawnChaseCamera = true;

    UPROPERTY(EditAnywhere, Category = "AV Live")
    bool bShowTelemetry = true;

    UPROPERTY(EditAnywhere, Category = "AV Live|Custom actors")
    AActor* EgoActorOverride = nullptr;

    UPROPERTY(EditAnywhere, Category = "AV Live|Custom actors")
    AActor* ObstacleActorOverride = nullptr;

private:
    UPROPERTY() USceneComponent*      Root     = nullptr;
    UPROPERTY() UStaticMeshComponent* EgoBox   = nullptr;
    UPROPERTY() UStaticMeshComponent* GhostBox = nullptr;
    UPROPERTY() UStaticMeshComponent* ObsBox   = nullptr;
    UPROPERTY() UStaticMeshComponent* Ground   = nullptr;
    UPROPERTY() UStaticMesh*          CubeMesh = nullptr;
    UPROPERTY() UMaterialInterface*   BaseMat  = nullptr;
    UPROPERTY() ACameraActor*         ChaseCam = nullptr;

    FSocket* Socket = nullptr;

    // Latest target (from the network) and smoothed display state.
    FVector TgtEgo = FVector(0, 0, 75), DispEgo = FVector(0, 0, 75);
    FVector TgtGhost = FVector(0, 0, 75), DispGhost = FVector(0, 0, 75);
    float   TgtEgoYaw = 0.f, DispEgoYaw = 0.f;
    float   TgtGhostYaw = 0.f, DispGhostYaw = 0.f;
    bool    bGotData = false;

    // Telemetry snapshot.
    float LastT = 0.f, LastV = 0.f, LastThr = 0.f, LastBrk = 0.f, LastSteer = 0.f;
    int32 LastNObj = 0;

    void ReceiveAll();
    void ParseLine(const FString& Line);
    FVector AvToUE(double x, double y, double zUU = 75.0) const;
    void SetPose(AActor* A, UStaticMeshComponent* C, const FVector& Pos, float YawDeg);
    UMaterialInstanceDynamic* MakeTint(FLinearColor Color);
};
