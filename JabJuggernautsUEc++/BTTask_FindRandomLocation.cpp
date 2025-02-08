// Fill out your copyright notice in the Description page of Project Settings.


#include "BTTask_FindRandomLocation.h"

#include "NavigationSystem.h"
#include "NPC_AIController.h"
#include "varMJGlobal.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Character.h"
#include "Kismet/GameplayStatics.h"
#include "mjpc/mjmp.h"

class UCameraComponent;

UBTTask_FindRandomLocation::UBTTask_FindRandomLocation(FObjectInitializer const& ObjectInitializer):
UBTTask_BlackboardBase(ObjectInitializer)
{
	NodeName="Find Random Location in NavMesh";
	
}
EBTNodeResult::Type UBTTask_FindRandomLocation::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{

	if (getAppSolo())
	{		
		Singleton& singleton = Singleton::getInstance();
		FVector torsoLoc(singleton.torso[1],  singleton.torso[0], singleton.torso[2]);

		if (auto* const NavSys=UNavigationSystemV1::GetCurrent(GetWorld()))
		{
			FNavLocation Loc;
			if (NavSys->GetRandomPointInNavigableRadius(torsoLoc,SearchRadius,Loc))
			{
				OwnerComp.GetBlackboardComponent()->SetValueAsVector(GetSelectedBlackboardKey(),Loc.Location);
				
			}

			FinishLatentTask(OwnerComp,EBTNodeResult::Succeeded);
			return EBTNodeResult::Succeeded;
		}

	}

	
	return EBTNodeResult::Failed;
}
