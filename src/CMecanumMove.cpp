//
// Created by Ronal on 4/28/2024.
//

#include "../include/CMecanumMove.hpp"

#define ACCELERATION 0.2

CMecanumMove::CMecanumMove() = default;

CMecanumMove::~CMecanumMove() {
    _threadExit = true;
}

bool CMecanumMove::init(CControlPi* control, unsigned int mode, bool relation) {
    _control = control;
    _mode = mode;
    _relation = relation;
    _wheelSpeed = {0, 0, 0, 0};
    _wheelVel = {0, 0, 0, 0};
    _deltaTime = std::chrono::steady_clock::now();

    switch (_mode) {
        case ECO:
            _speedModifier = 0.2;
        case NORMAL:
            _speedModifier = 0.5;
            break;
        case SPORT:
            _speedModifier = 1.0;
            break;
    }

    _threadExit = false;

    std::thread t1(&CMecanumMove::moveThread, this);
    t1.detach();
}

void CMecanumMove::moveThread(CMecanumMove* ptr) {
    while (!ptr->_threadExit) {
        ptr -> driveControl();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void CMecanumMove::driveControl() {
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
    int rotation = _speedModifier * r / 16;
    _wheelSpeed[NW] = (speed * (cos(theta) - sin(theta)) + rotation);
    _wheelSpeed[SW] = (speed * (cos(theta) + sin(theta)) + rotation);
    _wheelSpeed[NE] = (speed * (cos(theta) + sin(theta)) - rotation);
    _wheelSpeed[SE] = (speed * (cos(theta) - sin(theta)) - rotation);
}

void CMecanumMove::moveTank(int l, int r) {
    _wheelSpeed[NW] = l * _speedModifier;
    _wheelSpeed[SW] = _wheelSpeed[NW];
    _wheelSpeed[NE] = r * _speedModifier;
    _wheelSpeed[SE] = _wheelSpeed[NE];
}

void CMecanumMove::setRelation(bool relation) {
    _relation = relation;
}
