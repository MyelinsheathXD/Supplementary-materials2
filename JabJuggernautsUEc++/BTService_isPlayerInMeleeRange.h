// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/Services/BTService_BlackboardBase.h"
#include "BTService_isPlayerInMeleeRange.generated.h"

/**
 * 
 */
UCLASS()
class MPC_API UBTService_isPlayerInMeleeRange : public UBTService_BlackboardBase
{
	GENERATED_BODY()

protected:
	UBTService_isPlayerInMeleeRange();
	virtual void OnBecomeRelevant(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;


private:
	UPROPERTY(EditAnywhere,BlueprintReadWrite,Category=AI,meta=(AllowPrivateAccess="true"))
	float MeleeRange=2.5f;

};
