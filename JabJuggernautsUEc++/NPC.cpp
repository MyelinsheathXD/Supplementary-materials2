// Fill out your copyright notice in the Description page of Project Settings.


#include "NPC.h"

#include "mjueutilities.h"
#include "varMJGlobal.h"
#include "Components/AudioComponent.h"
#include <string>
#include <algorithm> 
#include <cmath>
#include "Math/UnrealMathUtility.h"



#include "mjpc/mjmp.h"


// Sets default values
ANPC::ANPC()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Set the tick group to PrePhysics
	//PrimaryActorTick.TickGroup = TG_PostPhysics;


	// Create an audio component and make it available in Blueprints
	AudioComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("AudioComponent"));
	AudioComponent->bAutoActivate = false; // Don't play the sound immediately
	AudioComponent->SetupAttachment(RootComponent);



	// Set spatialization settings
	AudioComponent->bAllowSpatialization = true;
	AudioComponent->bOverrideAttenuation = true;


}

// Called when the game starts or when spawned
//Singleton* singletonCache;
Singleton* singletonCache;
void ANPC::BeginPlay()
{
	Super::BeginPlay();
	singletonCache = &Singleton::getInstance();
	//singletonCache = &Singleton::getInstance();

	initTimers();


	// Iterate over all components of the actor
	TArray<USceneComponent*> SceneComponents;
	GetComponents<USceneComponent>(SceneComponents);

	for (USceneComponent* Component : SceneComponents)
	{
		if (Component->GetName().Equals(FnameComponentMJMPC.ToString()))
		{
			ComponentMJMPC = Component;

		}
	}


	for (USceneComponent* Component : SceneComponents)
	{
		if (Component->GetName().Equals(FnameFirstBody.ToString()))
		{
			SceneComFirstBody = Component;
		}
	}
	SceneComChildBodies.Empty();
	for (int i = 0; i < FnameChildBodies.Num(); ++i)
	{
		for (USceneComponent* Component : SceneComponents)
		{
			if (Component->GetName().Equals(FnameChildBodies[i].ToString()))
			{
				SceneComChildBodies.Add(Component);
			}
		}

	}

	// scale original vis mesh
	ComponentMJMPC->SetWorldScale3D(FVector::One() * ScaleUp);
	//
	singletonCache->firstBodyName = TCHAR_TO_UTF8(*FnameFirstBody.ToString());

	singletonCache->childBodyRotAxis.resize(9 * FnameChildBodies.Num());

	singletonCache->childBodyNames.clear();
	for (int i = 0; i < FnameChildBodies.Num(); ++i)
	{
		singletonCache->childBodyNames.push_back(TCHAR_TO_UTF8(*FnameChildBodies[i].ToString()));
	}

}

void ANPC::initTimers()
{
	GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &ANPC::MoveToPointTick, 0.3f, true);
	GetWorld()->GetTimerManager().SetTimer(TimerHandle2, this, &ANPC::SetRootToMJMPCLocationAndRotation, 0.2f, true);

}

void ANPC::ModifyComponentTransformlocByName(FName ComponentName, FVector NewLocation)
{
	TArray<USceneComponent*> Components;
	GetComponents<USceneComponent>(Components);
	for (USceneComponent* Component : Components)
	{
		if (Component && Component->GetName() == ComponentName.ToString())
		{

			Component->SetWorldLocation(NewLocation);

			break; // We found the component, no need to continue the loop
		}
	}
}

void ANPC::ModifyComponentTransformrotByName(FName ComponentName, FQuat NewRotation)
{
	TArray<USceneComponent*> Components;
	GetComponents<USceneComponent>(Components);
	for (USceneComponent* Component : Components)
	{
		if (Component && Component->GetName() == ComponentName.ToString())
		{
			// Apply the new transform
			Component->SetWorldRotation(NewRotation);
			break; // We found the component, no need to continue the loop
		}
	}
}

void ANPC::ModifyComponentTransformlocByCom(USceneComponent* com, FVector NewLocation)
{
	com->SetWorldLocation(NewLocation);
}

void ANPC::ModifyComponentTransformrotByCom(USceneComponent* com, FQuat NewRotation)
{
	com->SetWorldRotation(NewRotation);
}


void ANPC::updateALLbodyTransform()
{
	// Lock the mutex externally using unique_lock
	//std::unique_lock<std::mutex> lock(singletonCache->getLock());

	//FName ComponentName = TEXT("torso");
	FName ComponentName = FnameFirstBody;
	//
		//pelvisPos = UEMJU::MJtoUEpos(singletonCache->torso);
	pelvisPos = UEMJU::MJtoUEpos(singletonCache->firstBodypos) * ScaleUp;
	//ModifyComponentTransformlocByName(ComponentName, pelvisPos);
	ModifyComponentTransformlocByCom(SceneComFirstBody, pelvisPos);

	//pelvisRot = UEMJU::MJtoUErot(singletonCache->torsoq);

	//pelvisPos = UEMJU::MJtoUEpos(singletonCache->torso2);


	FVector f = -UEMJU::MJtoUEpos(singletonCache->firstBodyRotAxis) * 0.01;
	FVector r = UEMJU::MJtoUEpos(singletonCache->firstBodyRotAxis + 3) * 0.01;
	FVector u = -UEMJU::MJtoUEpos(singletonCache->firstBodyRotAxis + 6) * 0.01;

	f.Normalize();
	r.Normalize();
	u.Normalize();


	FMatrix RotationMatrix1(
		FPlane(f, 0.0f),
		FPlane(u, 0.0f),
		FPlane(r, 0.0f),
		FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	);
	FQuat RotationQuat1(RotationMatrix1);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat1);
	ModifyComponentTransformrotByCom(SceneComFirstBody, RotationQuat1);

	//// loop rotation
	for (int i = 0; i < FnameChildBodies.Num(); ++i)
	{
		setRotationMatrix(FnameChildBodies[i], i);
	}




	//ComponentName = TEXT("pelvis");
	//f = -UEMJU::MJtoUEpos(singletonCache->pelvisx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->pelvisy) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->pelvisz) * 0.01;

	//f.Normalize();
	//r.Normalize();
	//u.Normalize();



	//FMatrix RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//FQuat RotationQuat2 = FQuat(RotationMatrix2);

	//ModifyComponentTransformrotByName(ComponentName, RotationQuat2);


	//waist_lower
	//ComponentName = TEXT("waist_lower");
	// f = -UEMJU::MJtoUEpos(singletonCache->waist_lowerx) * 0.01;
	// r = UEMJU::MJtoUEpos(singletonCache->waist_lowery) * 0.01;
	// u = -UEMJU::MJtoUEpos(singletonCache->waist_lowerz) * 0.01;

	//f.Normalize();
	//r.Normalize();
	//u.Normalize();


	//RotationMatrix2 =FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 =FQuat(RotationMatrix2);



	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);



	//

	//ComponentName = TEXT("pelvis");
	//f = -UEMJU::MJtoUEpos(singletonCache->pelvisx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->pelvisy) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->pelvisz) * 0.01;

	//f.Normalize();
	//r.Normalize();
	//u.Normalize();



	//RotationMatrix2 =FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);

	//


	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);

	//
	//ComponentName = TEXT("thigh_right");
	//f = -UEMJU::MJtoUEpos(singletonCache->thigh_rightx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->thigh_righty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->thigh_rightz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 =FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);



	//
	//ComponentName = TEXT("shin_right");
	//f = -UEMJU::MJtoUEpos(singletonCache->shin_rightx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->shin_righty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->shin_rightz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);

	//
	//ComponentName = TEXT("foot_right");
	//f = -UEMJU::MJtoUEpos(singletonCache->foot_rightx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->foot_righty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->foot_rightz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);


	//
	//ComponentName = TEXT("thigh_left");
	//f = -UEMJU::MJtoUEpos(singletonCache->thigh_leftx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->thigh_lefty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->thigh_leftz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);



	//
	//ComponentName = TEXT("shin_left");
	//f = -UEMJU::MJtoUEpos(singletonCache->shin_leftx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->shin_lefty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->shin_leftz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);

	//
	//ComponentName = TEXT("foot_left");
	//f = -UEMJU::MJtoUEpos(singletonCache->foot_leftx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->foot_lefty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->foot_leftz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);




	//
	//ComponentName = TEXT("upper_arm_right");
	//f = -UEMJU::MJtoUEpos(singletonCache->upper_arm_rightx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->upper_arm_righty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->upper_arm_rightz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);


	//
	//ComponentName = TEXT("lower_arm_right");
	//f = -UEMJU::MJtoUEpos(singletonCache->lower_arm_rightx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->lower_arm_righty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->lower_arm_rightz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);


	//
	//ComponentName = TEXT("upper_arm_left");
	//f = -UEMJU::MJtoUEpos(singletonCache->upper_arm_leftx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->upper_arm_lefty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->upper_arm_leftz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);


	//
	//ComponentName = TEXT("lower_arm_left");
	//f = -UEMJU::MJtoUEpos(singletonCache->lower_arm_leftx) * 0.01;
	//r = UEMJU::MJtoUEpos(singletonCache->lower_arm_lefty) * 0.01;
	//u = -UEMJU::MJtoUEpos(singletonCache->lower_arm_leftz) * 0.01;
	//f.Normalize();
	//r.Normalize();
	//u.Normalize();
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);



	ComponentName = TEXT("sspherebody7");
	f = -UEMJU::MJtoUEpos(singletonCache->lower_arm_leftx) * 0.01;
	r = UEMJU::MJtoUEpos(singletonCache->lower_arm_lefty) * 0.01;
	u = -UEMJU::MJtoUEpos(singletonCache->lower_arm_leftz) * 0.01;
	f.Normalize();
	r.Normalize();
	u.Normalize();


	FVector tempPos;
	//singletonCache->rhip
	tempPos = UEMJU::MJtoUEpos(singletonCache->rhip) * ScaleUp;
	//RotationMatrix2 = FMatrix(
	//	FPlane(f, 0.0f),
	//	FPlane(u, 0.0f),
	//	FPlane(r, 0.0f),
	//	FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	//);
	//RotationQuat2 = FQuat(RotationMatrix2);
	//ModifyComponentTransformrotByName( ComponentName, RotationQuat2);

	//DrawDebugLine(
	//	GetWorld(),           // The world context
	//	tempPos,            // Starting point
	//	tempPos+ f*100,              // Ending point
	//	FColor::Red,            // Line color
	//	false,                // Persistent (will disappear after a short time, false means temporary)
	//	-1.0f,                // Lifetime (-1 means it will persist for a frame if `false` is passed in above)
	//	0,                    // Depth priority (0 means it will draw normally)
	//	5.0f                  // Line thickness
	//);
	//	DrawDebugLine(
	//	GetWorld(),           // The world context
	//	tempPos,            // Starting point
	//	tempPos+ r*100,              // Ending point
	//	FColor::Green,            // Line color
	//	false,                // Persistent (will disappear after a short time, false means temporary)
	//	-1.0f,                // Lifetime (-1 means it will persist for a frame if `false` is passed in above)
	//	0,                    // Depth priority (0 means it will draw normally)
	//	5.0f                  // Line thickness
	//);
	//	DrawDebugLine(
	//	GetWorld(),           // The world context
	//	tempPos,            // Starting point
	//	tempPos+ u*225,              // Ending point
	//	FColor::Blue,            // Line color
	//	false,                // Persistent (will disappear after a short time, false means temporary)
	//	-1.0f,                // Lifetime (-1 means it will persist for a frame if `false` is passed in above)
	//	0,                    // Depth priority (0 means it will draw normally)
	//	5.0f                  // Line thickness
	//);

}

void ANPC::DirectionRotate(FVector PlayerLocation)
{
	if (GetAnimMode(getAppSolo()) != 1 && GetAnimMode(getAppSolo()) != 2)
	{
		double loc[3] = { 0.0,0.0,0.0 };
		double locFirstBody[3] = { 0.0,0.0,0.0 };
		UEMJU::UEtoMJpos(PlayerLocation, loc);
		UEMJU::UEtoMJpos(SceneComFirstBody->GetComponentLocation(), locFirstBody);
		//Singleton& singletonCache = Singleton::getInstance();

		//float angle = atan2(y, x) * 180.0 / M_PI;
		float angle = atan2(loc[1] - locFirstBody[1], loc[0] - locFirstBody[0]) * 180.0 / 3.14;


		if (RotateToPointNow)
		{

			//SetRootRotation(getAppSolo(), angle);
			//GetRootRotation(getAppSolo());
			SetRootRotation(getAppSolo(), lerpAngle(GetRootRotation(getAppSolo()), angle, 0.5));
		}
		//move forward anim
		if (MoveToPointNow)
		{
			SetMPCAnimModeInternal(DirectionMoveAnimID);


		}


	}
}

bool ANPC::SetMPCAnimModeInternal(int mode)
{
	if (GetAnimMode(getAppSolo()) != 1 && GetAnimMode(getAppSolo()) != 2)
	{
		SetAnimMode(getAppSolo(), mode);
		return true;
	}

	return  false;
}

void ANPC::AudioFXforSimPercentage()
{
	UE_LOG(LogTemp, Warning, TEXT("The value of GetSimPhyRate is: %d"), GetSimnPhysRate(getAppSolo()));
	if (GetSimnPhysRate(getAppSolo()) == 0)
	{
		AudioComponent->SetPitchMultiplier(0);
	}
	else if (GetSimnPhysRate(getAppSolo()) == 1)
	{
		AudioComponent->SetPitchMultiplier(0.8);
	}
	else if (GetSimnPhysRate(getAppSolo()) == 2)
	{
		AudioComponent->SetPitchMultiplier(0.7);

	}
	else if (GetSimnPhysRate(getAppSolo()) == 3)
	{
		AudioComponent->SetPitchMultiplier(0.6);

	}
}


// Called every frame
void ANPC::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	updateALLbodyTransform();
}

// Called to bind functionality to input
void ANPC::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

UBehaviorTree* ANPC::GetBehaviorTree() const
{
	return Tree;
}

void ANPC::MoveToPointTick()
{

	DirectionRotate(MoveToPointVector);
}

void ANPC::MoveToNavPoint_Implementation(FVector Location, bool rotate, bool position, int AnimId)
{
	MoveToPointVector = Location;
	MoveToPointNow = position;
	RotateToPointNow = rotate;
	DirectionMoveAnimID = AnimId;

}

bool ANPC::SetMPCAnimMode_Implementation(int AnimMode)
{

	return SetMPCAnimModeInternal(AnimMode);
}

FTransform TcomMJMPC;
FRotator RotRoot;
void ANPC::SetRootToMJMPCLocationAndRotation()
{
	TcomMJMPC = ComponentMJMPC->GetComponentTransform();
	//FName ComponentName = TEXT("torso");
	//Singleton& singletonCache = Singleton::getInstance();
	//pelvisPos = UEMJU::MJtoUEpos(singletonCache->torso);

	//pelvisRot = UEMJU::MJtoUErot(singletonCache->torsoq);
	//SetActorLocation(pelvisPos);
	SetActorLocation(SceneComFirstBody->GetComponentLocation());

	//RotRoot=pelvisRot.Rotator();
	RotRoot = SceneComFirstBody->GetComponentRotation();
	RotRoot.Pitch = 0.0f;
	RotRoot.Roll = 0.0f;
	SetActorRotation(RotRoot);

	ComponentMJMPC->SetWorldTransform(TcomMJMPC);
	FTransform meshT = GetMesh()->GetComponentTransform();
	meshT.SetLocation(FVector::Zero());
	meshT.SetRotation(FQuat::Identity);
	meshT.SetScale3D(FVector::One() * ScaleUp);
	GetMesh()->SetWorldTransform(meshT);
}

FName ComponentNameRotMat;
FVector ff;
FVector rr;
FVector uu;
double tempVec3double[3];

void ANPC::setRotationMatrix(FName namecomponent, int id)
{
	ComponentNameRotMat = namecomponent;
	//
	tempVec3double[0] = singletonCache->childBodyRotAxis[id * 9 + 0];
	tempVec3double[1] = singletonCache->childBodyRotAxis[id * 9 + 1];
	tempVec3double[2] = singletonCache->childBodyRotAxis[id * 9 + 2];
	ff = -UEMJU::MJtoUEpos(tempVec3double) * 0.01;
	tempVec3double[0] = singletonCache->childBodyRotAxis[id * 9 + 3];
	tempVec3double[1] = singletonCache->childBodyRotAxis[id * 9 + 4];
	tempVec3double[2] = singletonCache->childBodyRotAxis[id * 9 + 5];
	rr = UEMJU::MJtoUEpos(tempVec3double) * 0.01;
	tempVec3double[0] = singletonCache->childBodyRotAxis[id * 9 + 6];
	tempVec3double[1] = singletonCache->childBodyRotAxis[id * 9 + 7];
	tempVec3double[2] = singletonCache->childBodyRotAxis[id * 9 + 8];
	uu = -UEMJU::MJtoUEpos(tempVec3double) * 0.01;

	ff.Normalize();
	rr.Normalize();
	uu.Normalize();


	FMatrix RotationMatrix2(
		FPlane(ff, 0.0f),
		FPlane(uu, 0.0f),
		FPlane(rr, 0.0f),
		FPlane(0.0f, 0.0f, 0.0f, 1.0f)
	);
	FQuat RotationQuat2(RotationMatrix2);

	// Convert the quaternion to a rotator and return it


	//ModifyComponentTransformrotByName( ComponentNameRotMat, RotationQuat2);
	ModifyComponentTransformrotByCom(SceneComChildBodies[id], RotationQuat2);
}

float ANPC::lerpAngle(float startAngle, float endAngle, float t)
{
	// Normalize startAngle and endAngle to be within the range 0 to 360 degrees
	startAngle = std::fmod(startAngle, 360.0f);
	endAngle = std::fmod(endAngle, 360.0f);

	// Compute the shortest path difference
	float deltaAngle = endAngle - startAngle;
	if (deltaAngle > 180.0f) {
		deltaAngle -= 360.0f; // Go the shorter way around the circle
	}
	else if (deltaAngle < -180.0f) {
		deltaAngle += 360.0f; // Go the shorter way around the circle
	}

	// Perform the linear interpolation using the shortest path
	float result = startAngle + t * deltaAngle;

	// Normalize the result back to the range 0 to 360 degrees
	result = std::fmod(result, 360.0f);
	if (result < 0.0f) {
		result += 360.0f; // Ensure positive result in the range [0, 360)
	}

	return result;
}

