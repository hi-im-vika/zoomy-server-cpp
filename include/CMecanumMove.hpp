//
// Created by Ronal on 4/28/2024.
//
#pragma once
#include <chrono>
#include <math.h>
#include <spdlog/spdlog.h>
#include "CControlPi.hpp"

enum wheel{NW = 0, NE, SW, SE};
enum relation{GLOBAL = 0, RELATIVE};
// todo doxygen
class CMecanumMove {
private:
    CControlPi* _control;

    float _speedModifier;

    std::vector<int> _wheelSpeed, _wheelVel;

    std::vector<float> _angle;
    float _rotation;

    std::chrono::steady_clock::time_point _deltaTime;

    bool _relation;

    bool _threadExit;

    static void moveThread(CMecanumMove* ptr);

    void driveControl();

public:
    CMecanumMove();

    ~CMecanumMove();

    bool init(CControlPi* control, float speedModifier = 0.5, bool relation = GLOBAL);

    void moveOmni(int x = 0, int y = 0, int ra = 0, int rb = 0);

    void moveTank(int l = 0, int r = 0);

    void setRelation(bool relation);

    void setRotation(int angle);
};
