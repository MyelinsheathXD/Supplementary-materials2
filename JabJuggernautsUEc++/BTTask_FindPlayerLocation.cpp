// Fill out your copyright notice in the Description page of Project Settings.


#include "BTTask_FindPlayerLocation.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Character.h"
#include "Kismet/GameplayStatics.h"

class UCameraComponent;

UBTTask_FindPlayerLocation::UBTTask_FindPlayerLocation(FObjectInitializer const& ObjectInitializer):
UBTTask_BlackboardBase(ObjectInitializer)
{
	NodeName=TEXT("Find Player Location");
}

EBTNodeResult::Type UBTTask_FindPlayerLocation::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
		// Get the player character
	ACharacter* PlayerCharacter = UGameplayStatics::GetPlayerCharacter(GetWorld(), 0);
	FVector CharacterLocation;
	
	if (PlayerCharacter)
	{
		// Get the world location of the player character
		CharacterLocation = PlayerCharacter->GetActorLocation();
		UCameraComponent* CameraComponent = PlayerCharacter->FindComponentByClass<UCameraComponent>();
		
		if (CameraComponent && PlayerCharacter->ActorHasTag("VRNPC"))
		{
			// Now you can use the CameraComponent
			CharacterLocation = CameraComponent->GetComponentLocation();

		}
		
		OwnerComp.GetBlackboardComponent()->SetValueAsVector(GetSelectedBlackboardKey(),CharacterLocation);

		///DirectionRotate(CharacterLocation);
		///MovetowPlayer(CharacterLocation);

		FinishLatentTask(OwnerComp,EBTNodeResult::Succeeded);
		return EBTNodeResult::Succeeded;

	}

		


	return EBTNodeResult::Failed;
}
