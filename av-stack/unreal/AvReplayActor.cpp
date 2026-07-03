// AvReplayActor.cpp  (Unreal Engine 4.27)
#include "AvReplayActor.h"

#include "Components/StaticMeshComponent.h"
#include "Camera/CameraActor.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"

AAvReplayActor::AAvReplayActor()
{
    PrimaryActorTick.bCanEverTick = true;

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootComponent = Root;

    EgoBox   = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("EgoBox"));
    GhostBox = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("GhostBox"));
    ObsBox   = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ObsBox"));
    Ground   = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Ground"));
    for (UStaticMeshComponent* C : {EgoBox, GhostBox, ObsBox, Ground})
    {
        C->SetupAttachment(Root);
        C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    }

    // Grab engine basic-shape assets so the actor is self-contained.
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeF(
        TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (CubeF.Succeeded()) { CubeMesh = CubeF.Object; }

    static ConstructorHelpers::FObjectFinder<UMaterialInterface> MatF(
        TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (MatF.Succeeded()) { BaseMat = MatF.Object; }

    for (UStaticMeshComponent* C : {EgoBox, GhostBox, ObsBox, Ground})
    {
        if (CubeMesh) { C->SetStaticMesh(CubeMesh); }
    }
}

FVector AAvReplayActor::AvToUE(double x, double y, double zUU) const
{
    // x forward, y LEFT (m)  ->  X forward, Y RIGHT (cm)
    return FVector(x * MetersToUU, -y * MetersToUU, zUU);
}

UMaterialInstanceDynamic* AAvReplayActor::MakeTint(FLinearColor Color, float Opacity)
{
    if (!BaseMat) { return nullptr; }
    UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(BaseMat, this);
    if (MID)
    {
        MID->SetVectorParameterValue(TEXT("Color"), Color);
        MID->SetScalarParameterValue(TEXT("Opacity"), Opacity); // ignored if absent
    }
    return MID;
}

void AAvReplayActor::BeginPlay()
{
    Super::BeginPlay();

    if (!LoadCsv() || Frames.Num() < 2)
    {
        UE_LOG(LogTemp, Error, TEXT("AvReplayActor: could not load a usable '%s'."), *CsvFileName);
        SetActorTickEnabled(false);
        return;
    }
    UE_LOG(LogTemp, Log, TEXT("AvReplayActor: loaded %d frames."), Frames.Num());

    // Car footprint 4.7 x 2.0 x 1.5 m; cube asset is 1 m, so scale == metres.
    EgoBox->SetRelativeScale3D(FVector(4.7f, 2.0f, 1.5f));
    EgoBox->SetMaterial(0, MakeTint(FLinearColor(0.0f, 0.85f, 0.75f)));   // teal

    GhostBox->SetRelativeScale3D(FVector(4.7f, 2.0f, 1.5f));
    GhostBox->SetMaterial(0, MakeTint(FLinearColor(0.25f, 0.55f, 1.0f))); // blue
    GhostBox->SetVisibility(bShowGhost && !GhostActorOverride);

    ObsBox->SetRelativeScale3D(ObstacleSizeUU / 100.f);
    ObsBox->SetMaterial(0, MakeTint(FLinearColor(1.0f, 0.25f, 0.25f)));   // red

    // Ground: span the lane with a margin, 20 cm thick, grey.
    const float LenM = static_cast<float>(LaneEndM - LaneStartM) + 40.f;
    Ground->SetRelativeScale3D(FVector(LenM, 20.f, 0.2f));
    Ground->SetMaterial(0, MakeTint(FLinearColor(0.10f, 0.11f, 0.13f)));

    // Hide built-in boxes that the user overrode with their own actors.
    if (EgoActorOverride)      { EgoBox->SetVisibility(false); }
    if (ObstacleActorOverride) { ObsBox->SetVisibility(false); }

    ApplyStatic();

    if (bSpawnChaseCamera)
    {
        ChaseCam = GetWorld()->SpawnActor<ACameraActor>();
        if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
        {
            PC->SetViewTargetWithBlend(ChaseCam, 0.f);
        }
    }

    PlaybackTime = Frames[0].T;
}

void AAvReplayActor::ApplyStatic()
{
    // Ground centre at the middle of the lane; top surface at z = 0.
    const float MidM = static_cast<float>(0.5 * (LaneStartM + LaneEndM));
    const float GroundHalfThicknessUU = 10.f; // scale 0.2 * 100 uu / 2
    Ground->SetWorldLocation(GetActorLocation() +
        FVector(MidM * MetersToUU, 0.f, -GroundHalfThicknessUU));

    // Obstacle (static). Sits on the ground.
    const FVector ObsPos = GetActorLocation() +
        FVector(ObstacleUE.X, ObstacleUE.Y, 0.5f * ObstacleSizeUU.Z);
    if (ObstacleActorOverride) { SetActorPose(ObstacleActorOverride, ObsPos, 0.f); }
    else                       { ObsBox->SetWorldLocation(ObsPos); }
}

void AAvReplayActor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (Frames.Num() < 2) { return; }

    const double T0 = Frames[0].T;
    const double T1 = Frames.Last().T;
    const double Dur = T1 - T0;

    PlaybackTime += DeltaSeconds * TimeScale;
    if (PlaybackTime > T1)
    {
        if (bLoop) { PlaybackTime = T0 + FMath::Fmod(PlaybackTime - T0, Dur); }
        else       { PlaybackTime = T1; }
    }

    const FAvFrame F = Sample(PlaybackTime);

    // Ego.
    if (EgoActorOverride) { SetActorPose(EgoActorOverride, F.TruePos, F.TrueYaw); }
    else                  { SetCompPose(EgoBox, F.TruePos, F.TrueYaw); }

    // EKF-estimate ghost.
    if (bShowGhost)
    {
        if (GhostActorOverride) { SetActorPose(GhostActorOverride, F.EstPos, F.EstYaw); }
        else                    { SetCompPose(GhostBox, F.EstPos, F.EstYaw); }
    }

    // Chase camera follows the ego.
    if (ChaseCam)
    {
        const FVector EgoLoc = GetActorLocation() + F.TruePos;
        const FVector Fwd = FRotator(0.f, F.TrueYaw, 0.f).Vector();
        const FVector CamLoc = EgoLoc - Fwd * 750.f + FVector(0.f, 0.f, 380.f);
        const FRotator CamRot = (EgoLoc - CamLoc).Rotation();
        ChaseCam->SetActorLocationAndRotation(CamLoc, CamRot);
    }

    if (bShowTelemetry && GEngine)
    {
        GEngine->AddOnScreenDebugMessage(101, 0.f, FColor::Cyan,
            FString::Printf(TEXT("t = %5.1f s   v = %5.2f m/s"), F.T, F.V));
        GEngine->AddOnScreenDebugMessage(102, 0.f, FColor::Green,
            FString::Printf(TEXT("throttle %3.0f%%   brake %3.0f%%   steer %+5.1f deg"),
                F.Throttle * 100.f, F.Brake * 100.f, FMath::RadiansToDegrees(F.Steer)));
        GEngine->AddOnScreenDebugMessage(103, 0.f, FColor::Yellow,
            FString::Printf(TEXT("objects: %d"), F.NObj));
    }

    OnFrame(static_cast<float>(F.T), F.V, F.Throttle, F.Brake, F.Steer);
}

FAvFrame AAvReplayActor::Sample(double Time) const
{
    if (Time <= Frames[0].T)      { return Frames[0]; }
    if (Time >= Frames.Last().T)  { return Frames.Last(); }

    int32 i = 0;
    while (i < Frames.Num() - 1 && Frames[i + 1].T < Time) { ++i; }

    const FAvFrame& A = Frames[i];
    const FAvFrame& B = Frames[i + 1];
    const double span = FMath::Max(1e-6, B.T - A.T);
    const float a = static_cast<float>((Time - A.T) / span);

    FAvFrame R;
    R.T       = Time;
    R.TruePos = FMath::Lerp(A.TruePos, B.TruePos, a);
    R.EstPos  = FMath::Lerp(A.EstPos,  B.EstPos,  a);
    R.TrueYaw = A.TrueYaw + FMath::FindDeltaAngleDegrees(A.TrueYaw, B.TrueYaw) * a;
    R.EstYaw  = A.EstYaw  + FMath::FindDeltaAngleDegrees(A.EstYaw,  B.EstYaw)  * a;
    R.V       = FMath::Lerp(A.V, B.V, a);
    R.Throttle= FMath::Lerp(A.Throttle, B.Throttle, a);
    R.Brake   = FMath::Lerp(A.Brake, B.Brake, a);
    R.Steer   = FMath::Lerp(A.Steer, B.Steer, a);
    R.NObj    = B.NObj;
    return R;
}

void AAvReplayActor::SetActorPose(AActor* A, const FVector& Pos, float YawDeg)
{
    if (A) { A->SetActorLocationAndRotation(GetActorLocation() + Pos, FRotator(0.f, YawDeg, 0.f)); }
}

void AAvReplayActor::SetCompPose(UStaticMeshComponent* C, const FVector& Pos, float YawDeg)
{
    if (C) { C->SetWorldLocationAndRotation(GetActorLocation() + Pos, FRotator(0.f, YawDeg, 0.f)); }
}

bool AAvReplayActor::LoadCsv()
{
    // Resolve the path: absolute as-is, else relative to the project directory.
    FString Path = CsvFileName;
    if (FPaths::IsRelative(Path))
    {
        Path = FPaths::Combine(FPaths::ProjectDir(), CsvFileName);
    }
    if (!FPaths::FileExists(Path))
    {
        UE_LOG(LogTemp, Error, TEXT("AvReplayActor: file not found: %s"), *Path);
        return false;
    }

    TArray<FString> Lines;
    if (!FFileHelper::LoadFileToStringArray(Lines, *Path)) { return false; }

    Frames.Reset();
    bool bHeaderSeen = false;
    const float CarZ = 75.f; // car origin height so the box base rests on z = 0

    for (const FString& Raw : Lines)
    {
        FString Line = Raw.TrimStartAndEnd();
        if (Line.IsEmpty()) { continue; }

        if (Line.StartsWith(TEXT("#")))
        {
            TArray<FString> P;
            Line.RightChop(1).ParseIntoArray(P, TEXT(","), false);
            for (FString& s : P) { s = s.TrimStartAndEnd(); }
            if (P.Num() >= 5 && P[0] == TEXT("lane"))
            {
                LaneStartM = FCString::Atod(*P[1]);
                LaneEndM   = FCString::Atod(*P[3]);
            }
            else if (P.Num() >= 3 && P[0] == TEXT("obstacle"))
            {
                const double ox = FCString::Atod(*P[1]);
                const double oy = FCString::Atod(*P[2]);
                ObstacleUE = AvToUE(ox, oy, 0.0);
                const float sx = P.Num() > 3 ? FCString::Atof(*P[3]) : 2.f;
                const float sy = P.Num() > 4 ? FCString::Atof(*P[4]) : 2.f;
                ObstacleSizeUU = FVector(sx * 100.f, sy * 100.f, 150.f);
            }
            continue;
        }

        if (!bHeaderSeen && Line.StartsWith(TEXT("t,"))) { bHeaderSeen = true; continue; }

        TArray<FString> C;
        Line.ParseIntoArray(C, TEXT(","), false); // keep empty fields for alignment
        if (C.Num() < 12) { continue; }

        auto D = [&C](int32 i) { return C.IsValidIndex(i) ? FCString::Atod(*C[i]) : 0.0; };
        auto F = [&C](int32 i) { return C.IsValidIndex(i) ? FCString::Atof(*C[i]) : 0.f; };

        FAvFrame R;
        R.T       = D(0);
        R.TruePos = AvToUE(D(1), D(2), CarZ);
        R.TrueYaw = -FMath::RadiansToDegrees(F(3));
        R.EstPos  = AvToUE(D(4), D(5), CarZ);
        R.EstYaw  = -FMath::RadiansToDegrees(F(6));
        R.V       = F(7);
        R.Throttle= F(8);
        R.Brake   = F(9);
        R.Steer   = F(10);
        R.NObj    = FCString::Atoi(*C[11]);
        R.bHasObj = R.NObj > 0 && C.IsValidIndex(12) && !C[12].IsEmpty();
        if (R.bHasObj) { R.ObjPos = AvToUE(D(12), D(13), CarZ); }
        Frames.Add(R);
    }

    return Frames.Num() >= 2;
}
