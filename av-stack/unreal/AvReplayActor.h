// AvReplayActor.h
// Replays an av-stack `av_trace.csv` inside Unreal Engine 4 (tested against 4.27).
//
// Drop this actor into a level; on BeginPlay it loads the CSV, places the
// obstacle, and on every Tick interpolates the ego (and its EKF-estimate ghost)
// along the recorded trajectory. If you don't assign your own meshes it spawns
// simple coloured boxes and a chase camera so you see motion immediately.
//
// Coordinate conversion (av-stack -> UE4):
//   av-stack is right-handed, metres, x forward / y LEFT / z up, yaw CCW (rad).
//   UE4 is left-handed, centimetres, X forward / Y RIGHT / Z up, yaw CW (deg).
//   => UE_X = x * 100,  UE_Y = -y * 100,  UE_YawDeg = -yaw * 180/PI.
//
// NOTE: this class has no PROJECT_API macro, so compile it into your primary
// game module. If you move it to a separate module, add YOURMODULE_API before
// the class name.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AvReplayActor.generated.h"

class UStaticMeshComponent;
class UStaticMesh;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class ACameraActor;

// One parsed row of the trace (already in UE world space where it makes sense).
USTRUCT()
struct FAvFrame
{
    GENERATED_BODY()

    double  T        = 0.0;   // seconds
    FVector TruePos  = FVector::ZeroVector;   // UE cm
    float   TrueYaw  = 0.f;                    // UE degrees
    FVector EstPos   = FVector::ZeroVector;   // UE cm
    float   EstYaw   = 0.f;                    // UE degrees
    float   V        = 0.f;   // m/s
    float   Throttle = 0.f;   // 0..1
    float   Brake    = 0.f;   // 0..1
    float   Steer    = 0.f;   // rad
    int32   NObj     = 0;
    FVector ObjPos   = FVector::ZeroVector;   // UE cm
    bool    bHasObj  = false;
};

UCLASS()
class AAvReplayActor : public AActor
{
    GENERATED_BODY()

public:
    AAvReplayActor();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;

    // ---- Configuration (editable per-instance in the Details panel) ----

    // File name (looked up under the project directory) or an absolute path.
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    FString CsvFileName = TEXT("av_trace.csv");

    // Metres -> Unreal units. 100 = 1 m per 100 uu (UE default).
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    float MetersToUU = 100.f;

    // Playback speed multiplier.
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    float TimeScale = 1.f;

    // Loop the trace when it reaches the end.
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    bool bLoop = true;

    // Show the semi-transparent EKF-estimate ghost car.
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    bool bShowGhost = true;

    // Spawn + drive a chase camera that follows the ego.
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    bool bSpawnChaseCamera = true;

    // Print speed / throttle / brake to the top-left of the viewport.
    UPROPERTY(EditAnywhere, Category = "AV Replay")
    bool bShowTelemetry = true;

    // OPTIONAL: assign your own actors instead of the auto-spawned boxes.
    // Their transforms will be driven each Tick.
    UPROPERTY(EditAnywhere, Category = "AV Replay|Custom actors")
    AActor* EgoActorOverride = nullptr;

    UPROPERTY(EditAnywhere, Category = "AV Replay|Custom actors")
    AActor* GhostActorOverride = nullptr;

    UPROPERTY(EditAnywhere, Category = "AV Replay|Custom actors")
    AActor* ObstacleActorOverride = nullptr;

    // Fired every Tick with the interpolated state — bind in a Blueprint child
    // to drive a UMG HUD if you want a nicer overlay.
    UFUNCTION(BlueprintImplementableEvent, Category = "AV Replay")
    void OnFrame(float Time, float SpeedMps, float Throttle, float Brake, float SteerRad);

private:
    // Built-in visuals (used when no override actor is assigned).
    UPROPERTY() USceneComponent*      Root     = nullptr;
    UPROPERTY() UStaticMeshComponent* EgoBox   = nullptr;
    UPROPERTY() UStaticMeshComponent* GhostBox = nullptr;
    UPROPERTY() UStaticMeshComponent* ObsBox   = nullptr;
    UPROPERTY() UStaticMeshComponent* Ground   = nullptr;

    UPROPERTY() UStaticMesh*     CubeMesh    = nullptr;
    UPROPERTY() UMaterialInterface* BaseMat  = nullptr;
    UPROPERTY() ACameraActor*    ChaseCam    = nullptr;

    TArray<FAvFrame> Frames;

    // Scenario constants from the CSV header comments.
    FVector ObstacleUE = FVector::ZeroVector;
    FVector ObstacleSizeUU = FVector(200.f, 200.f, 150.f);
    double  LaneStartM = 0.0, LaneEndM = 200.0;

    double  PlaybackTime = 0.0;

    bool LoadCsv();
    void ApplyStatic();                 // one-time placement (obstacle, ground)
    FAvFrame Sample(double Time) const; // interpolated frame at Time
    void SetActorPose(AActor* A, const FVector& Pos, float YawDeg);
    void SetCompPose(UStaticMeshComponent* C, const FVector& Pos, float YawDeg);
    FVector AvToUE(double x, double y, double zUU = 0.0) const;
    UMaterialInstanceDynamic* MakeTint(FLinearColor Color, float Opacity = 1.f);
};
