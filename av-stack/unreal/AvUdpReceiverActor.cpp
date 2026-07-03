// AvUdpReceiverActor.cpp  (Unreal Engine 4.27; needs "Sockets" + "Networking" modules)
#include "AvUdpReceiverActor.h"

#include "Components/StaticMeshComponent.h"
#include "Camera/CameraActor.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"

#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Common/UdpSocketBuilder.h"
#include "Interfaces/IPv4/IPv4Address.h"

AAvUdpReceiverActor::AAvUdpReceiverActor()
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

FVector AAvUdpReceiverActor::AvToUE(double x, double y, double zUU) const
{
    return FVector(x * MetersToUU, -y * MetersToUU, zUU);
}

UMaterialInstanceDynamic* AAvUdpReceiverActor::MakeTint(FLinearColor Color)
{
    if (!BaseMat) { return nullptr; }
    UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(BaseMat, this);
    if (MID) { MID->SetVectorParameterValue(TEXT("Color"), Color); }
    return MID;
}

void AAvUdpReceiverActor::BeginPlay()
{
    Super::BeginPlay();

    // Visuals.
    EgoBox->SetRelativeScale3D(FVector(4.7f, 2.0f, 1.5f));
    EgoBox->SetMaterial(0, MakeTint(FLinearColor(0.0f, 0.85f, 0.75f)));
    GhostBox->SetRelativeScale3D(FVector(4.7f, 2.0f, 1.5f));
    GhostBox->SetMaterial(0, MakeTint(FLinearColor(0.25f, 0.55f, 1.0f)));
    GhostBox->SetVisibility(bShowGhost);
    ObsBox->SetRelativeScale3D(FVector(2.0f, 2.0f, 1.5f));
    ObsBox->SetMaterial(0, MakeTint(FLinearColor(1.0f, 0.25f, 0.25f)));
    ObsBox->SetVisibility(false); // shown once an "# obstacle" line arrives
    Ground->SetRelativeScale3D(FVector(240.f, 20.f, 0.2f));
    Ground->SetMaterial(0, MakeTint(FLinearColor(0.10f, 0.11f, 0.13f)));
    Ground->SetWorldLocation(GetActorLocation() + FVector(100.f * MetersToUU, 0.f, -10.f));
    if (EgoActorOverride) { EgoBox->SetVisibility(false); }

    // UDP socket, non-blocking, bound to the configured port.
    Socket = FUdpSocketBuilder(TEXT("AvUdpRecv"))
                 .AsNonBlocking()
                 .AsReusable()
                 .BoundToAddress(FIPv4Address::Any)
                 .BoundToPort(static_cast<uint16>(Port))
                 .WithReceiveBufferSize(1 << 20)
                 .Build();
    if (!Socket)
    {
        UE_LOG(LogTemp, Error, TEXT("AvUdpReceiver: failed to bind UDP port %d"), Port);
        SetActorTickEnabled(false);
        return;
    }
    UE_LOG(LogTemp, Log, TEXT("AvUdpReceiver: listening on UDP %d"), Port);

    if (bSpawnChaseCamera)
    {
        ChaseCam = GetWorld()->SpawnActor<ACameraActor>();
        if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
        {
            PC->SetViewTargetWithBlend(ChaseCam, 0.f);
        }
    }
}

void AAvUdpReceiverActor::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Socket)
    {
        Socket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
        Socket = nullptr;
    }
    Super::EndPlay(Reason);
}

void AAvUdpReceiverActor::ReceiveAll()
{
    if (!Socket) { return; }
    ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    TSharedRef<FInternetAddr> Sender = SS->CreateInternetAddr();

    uint32 PendingSize = 0;
    while (Socket->HasPendingData(PendingSize))
    {
        TArray<uint8> Buf;
        Buf.SetNumUninitialized(FMath::Min(PendingSize, 65507u) + 1);
        int32 Read = 0;
        if (Socket->RecvFrom(Buf.GetData(), Buf.Num() - 1, Read, *Sender) && Read > 0)
        {
            Buf[Read] = 0;
            const FString Payload = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf.GetData())));
            TArray<FString> Lines;
            Payload.ParseIntoArrayLines(Lines);
            for (const FString& L : Lines) { ParseLine(L); }
        }
        else { break; }
    }
}

void AAvUdpReceiverActor::ParseLine(const FString& Raw)
{
    FString Line = Raw.TrimStartAndEnd();
    if (Line.IsEmpty()) { return; }

    if (Line.StartsWith(TEXT("#")))
    {
        TArray<FString> P;
        Line.RightChop(1).ParseIntoArray(P, TEXT(","), false);
        for (FString& s : P) { s = s.TrimStartAndEnd(); }
        if (P.Num() >= 3 && P[0] == TEXT("obstacle"))
        {
            const FVector Pos = AvToUE(FCString::Atod(*P[1]), FCString::Atod(*P[2]), 75.0);
            const float sx = P.Num() > 3 ? FCString::Atof(*P[3]) : 2.f;
            const float sy = P.Num() > 4 ? FCString::Atof(*P[4]) : 2.f;
            ObsBox->SetRelativeScale3D(FVector(sx, sy, 1.5f));
            if (ObstacleActorOverride) { SetPose(ObstacleActorOverride, nullptr, Pos, 0.f); }
            else { ObsBox->SetVisibility(true); ObsBox->SetWorldLocation(GetActorLocation() + Pos); }
        }
        return;
    }
    if (Line.StartsWith(TEXT("t,"))) { return; } // header line, ignore

    TArray<FString> C;
    Line.ParseIntoArray(C, TEXT(","), false);
    if (C.Num() < 12) { return; }
    auto D = [&C](int32 i) { return C.IsValidIndex(i) && !C[i].IsEmpty() ? FCString::Atod(*C[i]) : 0.0; };
    auto F = [&C](int32 i) { return C.IsValidIndex(i) && !C[i].IsEmpty() ? FCString::Atof(*C[i]) : 0.f; };

    TgtEgo      = AvToUE(D(1), D(2), 75.0);
    TgtEgoYaw   = -FMath::RadiansToDegrees(F(3));
    TgtGhost    = AvToUE(D(4), D(5), 75.0);
    TgtGhostYaw = -FMath::RadiansToDegrees(F(6));
    LastT     = F(0);
    LastV     = F(7);
    LastThr   = F(8);
    LastBrk   = F(9);
    LastSteer = F(10);
    LastNObj  = FCString::Atoi(*C[11]);

    if (!bGotData)
    {
        DispEgo = TgtEgo; DispGhost = TgtGhost;
        DispEgoYaw = TgtEgoYaw; DispGhostYaw = TgtGhostYaw;
        bGotData = true;
    }
}

void AAvUdpReceiverActor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    ReceiveAll();
    if (!bGotData) { return; }

    const float a = FMath::Clamp(SmoothingSpeed * DeltaSeconds, 0.f, 1.f);
    DispEgo   = FMath::Lerp(DispEgo, TgtEgo, a);
    DispGhost = FMath::Lerp(DispGhost, TgtGhost, a);
    DispEgoYaw   += FMath::FindDeltaAngleDegrees(DispEgoYaw, TgtEgoYaw) * a;
    DispGhostYaw += FMath::FindDeltaAngleDegrees(DispGhostYaw, TgtGhostYaw) * a;

    SetPose(EgoActorOverride, EgoBox, DispEgo, DispEgoYaw);
    if (bShowGhost) { SetPose(nullptr, GhostBox, DispGhost, DispGhostYaw); }

    if (ChaseCam)
    {
        const FVector EgoLoc = GetActorLocation() + DispEgo;
        const FVector Fwd = FRotator(0.f, DispEgoYaw, 0.f).Vector();
        const FVector CamLoc = EgoLoc - Fwd * 750.f + FVector(0.f, 0.f, 380.f);
        ChaseCam->SetActorLocationAndRotation(CamLoc, (EgoLoc - CamLoc).Rotation());
    }

    if (bShowTelemetry && GEngine)
    {
        GEngine->AddOnScreenDebugMessage(201, 0.f, FColor::Cyan,
            FString::Printf(TEXT("[live] t = %5.1f s   v = %5.2f m/s"), LastT, LastV));
        GEngine->AddOnScreenDebugMessage(202, 0.f, FColor::Green,
            FString::Printf(TEXT("throttle %3.0f%%   brake %3.0f%%   steer %+5.1f deg"),
                LastThr * 100.f, LastBrk * 100.f, FMath::RadiansToDegrees(LastSteer)));
        GEngine->AddOnScreenDebugMessage(203, 0.f, FColor::Yellow,
            FString::Printf(TEXT("objects: %d"), LastNObj));
    }
}

void AAvUdpReceiverActor::SetPose(AActor* A, UStaticMeshComponent* C, const FVector& Pos, float YawDeg)
{
    const FVector World = GetActorLocation() + Pos;
    const FRotator Rot(0.f, YawDeg, 0.f);
    if (A)      { A->SetActorLocationAndRotation(World, Rot); }
    else if (C) { C->SetWorldLocationAndRotation(World, Rot); }
}
