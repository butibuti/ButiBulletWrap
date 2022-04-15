#include "stdafx.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include"ButiMath/ButiMath.h"
namespace ButiEngine {
Vector3 Vector3::Zero = Vector3();
Vector3 Vector3::XAxis = Vector3(1.0f, 0.0f, 0.0f);
Vector3 Vector3::YAxis = Vector3(0.0f, 1.0f, 0.0f);
Vector3 Vector3::ZAxis = Vector3(0.0f,0.0f,1.0f);
}

BOOL __stdcall DllMain(HMODULE hModule,
    DWORD  ul_reason_for_call,
    LPVOID lpReserved
)
{

    return TRUE;
}
