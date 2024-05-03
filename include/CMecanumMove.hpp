//
// Created by Ronal on 4/28/2024.
//
#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <math.h>
#include "CControlPi.hpp"

enum wheel{NW = 0, NE, SW, SE};
enum mode{ECO = 0, NORMAL, SPORT};
enum relation{GLOBAL = 0, RELATIVE};
// todo doxygen
class CMecanumMove {
private:
    CControlPi* _control;

    float _speedModifier;

    std::vector<int> _wheelSpeed;

    std::vector<int> _wheelVel;

    std::chrono::steady_clock::time_point _deltaTime;

    unsigned int _mode;

    bool _relation;

    bool _threadExit;

    static void moveThread(CMecanumMove* ptr);

    void driveControl();

public:
    CMecanumMove();

    ~CMecanumMove();

    bool init(CControlPi* control, unsigned int mode = 2, bool relation = GLOBAL);

    void moveOmni(int x = 0, int y = 0, int r = 0);

    void moveTank(int l = 0, int r = 0);

    void setRelation(bool relation);
};
