//// Copyright Epic Games, Inc. All Rights Reserved.
//
#pragma once

#include "CoreMinimal.h"
#include "Math/Quat.h"
#include "Math/Rotator.h"

#include "mujoco/mujoco.h"


namespace UEMJU
{
	

	FVector MJtoUEpos(double* pos);
//{
//	//return FVector(pos[0] * 100.0, pos[1] * 100.0, pos[2] * 100.0);
//	return FVector(pos[1] * 100.0, pos[0] * 100.0, pos[2] * 100.0);
//}


	double* UEtoMJpos(FVector UEpos, double* pos);
//{
//	pos[0] = UEpos.Y * 0.01;
//	pos[1] = UEpos.X * 0.01;
//	pos[2] = UEpos.Z * 0.01;
//	return pos;
//}

	FQuat MJtoUErot(double* rot);

}