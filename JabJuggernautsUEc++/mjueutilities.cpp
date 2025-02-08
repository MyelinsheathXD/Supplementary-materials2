#include "mjueutilities.h"
//// Copyright Epic Games, Inc. All Rights Reserved.
//
#pragma once

#include "mjueutilities.h"



FVector UEMJU::MJtoUEpos(double* pos)
{
	//return FVector(pos[0] * 100.0, pos[1] * 100.0, pos[2] * 100.0);
	// 
	//return FVector(pos[1] * 100.0, pos[0] * 100.0, pos[2] * 100.0);

	//cahtgpt
	//return FVector(pos[0] * 100.0, pos[2] * 100.0, -pos[1] * 100.0);
	//return FVector(-pos[0] * 100.0, pos[1] * 100.0, pos[2] * 100.0);
	return FVector(pos[0] * 100.0, pos[1] * -100.0, pos[2] * 100.0);
}


double*  UEMJU::UEtoMJpos(FVector UEpos,double* pos)
{
	//pos[0] = UEpos.Y * 0.01;
	//pos[1] = UEpos.X * 0.01;
	//pos[2] = UEpos.Z * 0.01;

	//pos[0] = UEpos.X * 0.01;
	//pos[1] = -UEpos.Z * 0.01;
	//pos[2] = UEpos.Y * 0.01;
	// 
	// 
	//pos[0] = -UEpos.X * 0.01;
	//pos[1] = UEpos.Y * 0.01;
	//pos[2] = UEpos.Z * 0.01;

	pos[0] = UEpos.X * 0.01;
	pos[1] = -UEpos.Y * 0.01;
	pos[2] = UEpos.Z * 0.01;
	return pos;
}

FQuat UEMJU::MJtoUErot(double* rot)
{
	//FQuat IdentityQuat = FQuat::Identity;
	//FQuat SomeQuat = FQuat (0.f, 0.f, 0.f, 1.f);
	
	//FQuat SomeQuat(rot[0], -rot[3], rot[1], rot[2]);
	
	//FQuat SomeQuat(rot[1], -rot[0], rot[2], rot[3]);

	//FQuat SomeQuat(-rot[0], rot[1], rot[2], -rot[3]);
	//FQuat SomeQuat(-rot[1], rot[2], rot[3], -rot[0]);
	//FQuat SomeQuat = FQuat(0.f, 0.0f, 0.f, 0.f);
	FQuat SomeQuat = FQuat::Identity;

	//FQuat SomeQuat(rot[0], rot[1], rot[2], rot[3]);
	//FQuat SomeQuat(rot[1], -rot[2], -rot[3], -rot[0]);
	//FQuat SomeQuat(rot[0], -rot[1], -rot[2], -rot[3]);

	//FQuat SomeQuat(-rot[0], -rot[1], rot[2], rot[3]);
	// 
	//SomeQuat.X = -rot[1];
	//SomeQuat.Y = -rot[2];
	//SomeQuat.Z = rot[3];
	//SomeQuat.W = rot[0];

	SomeQuat.X = -rot[1];
	SomeQuat.Y = -rot[2];
	SomeQuat.Z = -rot[3];
	SomeQuat.W = rot[0];

	return SomeQuat;
}

