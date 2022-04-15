
#ifndef BUTIBULLET_H
#define BUTIBULLET_H
#pragma once
#ifdef BUTIBULLETWRAP_EXPORTS
#define BUTIBULLET_API __declspec(dllexport)
#else
#define BUTIBULLET_API __declspec(dllimport)
#endif

namespace ButiBullet {
constexpr float StepSeconds60FPS = 1.0f / 60.0f;

}

#endif // !BUTIBULLET_H
