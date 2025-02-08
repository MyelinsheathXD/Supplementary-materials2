// Fill out your copyright notice in the Description page of Project Settings.


#include "BTService_isPlayerInMeleeRange.h"

#include "NPC.h"
#include "NPC_AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Character.h"
#include "Kismet/GameplayStatics.h"

class UCameraComponent;

UBTService_isPlayerInMeleeRange::UBTService_isPlayerInMeleeRange()
{
	bNotifyBecomeRelevant=true;
	NodeName=TEXT("Is Player In Melee Range");
}

void UBTService_isPlayerInMeleeRange::OnBecomeRelevant(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
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

		//double loc[3] = { 0.0,0.0,0.0 };
		//UEMJU::UEtoMJpos(CharacterLocation, loc);
		//Singleton& singleton = Singleton::getInstance();
		//pelvisPos = UEMJU::MJtoUEpos(singleton.torso);
		///UEMJU::MJtoUEpos(singleton.torso);

		//float angle = atan2(y, x) * 180.0 / M_PI;
		//float angle = atan2(loc[1] - singleton.torso[1], loc[0] - singleton.torso[0]) * 180.0 / 3.14;
		//FVector deltaLoc(loc[1] - singleton.torso[1], loc[0] - singleton.torso[0], loc[2] - singleton.torso[2]);
		//FVector deltaLoc (CharacterLocation- UEMJU::MJtoUEpos(singleton.torso));
		auto const* cont= Cast<ANPC_AIController>(OwnerComp.GetAIOwner());
		auto const* npc= Cast<ANPC>(cont->GetPawn());


		OwnerComp.GetBlackboardComponent()->SetValueAsBool(
			GetSelectedBlackboardKey(),
			(CharacterLocation- npc->GetActorLocation()).Length()<=MeleeRange);

		/*OwnerComp.GetBlackboardComponent()->SetValueAsBool(
	GetSelectedBlackboardKey(),
	(CharacterLocation- UEMJU::MJtoUEpos(singleton.torso)).Length()<=MeleeRange);
	*/
		


	}
}
