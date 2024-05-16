//
// Created by Ronal on 4/28/2024.
//

#include "../include/CMecanumMove.hpp"

#define ACCELERATION 0.0125
#define ROTATION_SPEED 14

CMecanumMove::CMecanumMove() = default;

CMecanumMove::~CMecanumMove() {
    _threadExit = true;
}

bool CMecanumMove::init(CControlPi* control, float speedModifier, bool relation) {
    _control = control;
    _relation = relation;
    _turn = {0, 0};
    _angle = {0.0, 0.0, 0.0};
    _rotation = 0.0;
    _speedModifier = speedModifier / 8.0;
    _wheelSpeed = {0, 0, 0, 0};
    _wheelVel = {0, 0, 0, 0};
    _deltaTime = std::chrono::steady_clock::now();

    _threadExit = false;
    std::thread t1(&CMecanumMove::moveThread, this);
    t1.detach();

    return true;
}

void CMecanumMove::moveThread(CMecanumMove* ptr) {
    while (!ptr->_threadExit) {
        ptr -> driveControl();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void CMecanumMove::driveControl() {
    if (_relation) {
        // todo relative motion controls
    }
    int delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _deltaTime).count();
    _deltaTime = std::chrono::steady_clock::now();
    for (int i = 0; i < _wheelSpeed.size(); i++) {
        _wheelVel[i] += (_wheelSpeed[i] - _wheelVel[i]) * ACCELERATION * delta;
        _control->pca9685_motor_control(CControlPi::motor(i), _wheelVel[i]);
    }
}

void CMecanumMove::moveOmni(int x, int y, int r) {
    unsigned int speed = _speedModifier * hypot(x, y);
    float theta = atan2(-x, -y);
    int rotation = _speedModifier * r *0.65;
    _wheelSpeed[NW] = speed * (cos(theta) - sin(theta)) - rotation;
    _wheelSpeed[SW] = speed * (cos(theta) + sin(theta)) - rotation;
    _wheelSpeed[NE] = speed * (cos(theta) + sin(theta)) + rotation;
    _wheelSpeed[SE] = speed * (cos(theta) - sin(theta)) + rotation;
}

void CMecanumMove::moveTank(int l, int r) {
    _wheelSpeed[NW] = -l * _speedModifier;
    _wheelSpeed[SW] = _wheelSpeed[NW];
    _wheelSpeed[NE] = -r * _speedModifier;
    _wheelSpeed[SE] = _wheelSpeed[NE];
}

void CMecanumMove::setRelation(bool relation) {
    _relation = relation;
}
