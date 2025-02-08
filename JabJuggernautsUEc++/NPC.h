// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MPCCombatInterface.h"
#include "BehaviorTree/BehaviorTree.h"
#include "GameFramework/Character.h"
#include "NPC.generated.h"

UCLASS()
class MPC_API ANPC : public ACharacter, public IMPCCombatInterface
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ANPC();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AI", meta = (AllowPrivateAccess = "true"))
	UAudioComponent* AudioComponent;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AI", meta = (AllowPrivateAccess = "true"))
	float ScaleUp = 1.0;



protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	void initTimers();
	FTimerHandle TimerHandle;
	FTimerHandle TimerHandle2;
private:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AI", meta = (AllowPrivateAccess = "true"))
	UBehaviorTree* Tree;

	//UPROPERTY(EditAnywhere,BlueprintReadWrite, Category="AI", meta=(AllowPrivateAccess="true"))
	//APatrolPath* PatrolPath;

	//UPROPERTY(EditAnywhere,BlueprintReadWrite, Category=Animation, meta=(AllowPrivateAccess="true"))
	//UAnimMontage* Montage;


	void ModifyComponentTransformlocByName(FName ComponentName, FVector NewLocation);
	void ModifyComponentTransformrotByName(FName ComponentName, FQuat NewRotation);
	void ModifyComponentTransformlocByCom(USceneComponent* com, FVector NewLocation);
	void ModifyComponentTransformrotByCom(USceneComponent* com, FQuat NewRotation);

	void updateALLbodyTransform();
	FVector pelvisPos;
	FQuat pelvisRot;
	FQuat waist_lowerRot;

	bool MoveToPointNow = false;
	bool RotateToPointNow = false;
	FVector MoveToPointVector;
	int DirectionMoveAnimID = 3;
	void DirectionRotate(FVector PlayerLocation);
	bool SetMPCAnimModeInternal(int Mode);



	///audio control for sim percentage
	void AudioFXforSimPercentage();

public:

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	UBehaviorTree* GetBehaviorTree() const;

	// 0.1 tick timer
	void MoveToPointTick();
	//APatrolPath* GetPatrolPath() const;
	//UAnimMontage* GetMontage();
	//int MeleeAttack_Implementation() override;


	virtual void MoveToNavPoint_Implementation(FVector Location, bool rotate, bool position, int AnimId) override;
	virtual bool SetMPCAnimMode_Implementation(int AnimMode) override;

	USceneComponent* ComponentMJMPC;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transform", meta = (AllowPrivateAccess = "true"))
	FName FnameComponentMJMPC = "humanoid2StandAlone643bodyOnly";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transform", meta = (AllowPrivateAccess = "true"))
	FName FnameFirstBody = "torso";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transform", meta = (AllowPrivateAccess = "true"))
	TArray<FName> FnameChildBodies;

	USceneComponent* SceneComFirstBody;
	TArray<USceneComponent*> SceneComChildBodies;

	void SetRootToMJMPCLocationAndRotation();


	void setRotationMatrix(FName namecomponent, int id);
	float lerpAngle(float startAngle, float endAngle, float t);
};
