//
// Created by Ronal on 4/28/2024.
//

#include "../include/CMecanumMove.hpp"
CMecanumMove::CMecanumMove() {
    _threadExit = false;
}

CMecanumMove::~CMecanumMove() {
}

void CMecanumMove::setAcceleration(cv::Point3f acceleration) {
    _acceleration = acceleration;
}

void CMecanumMove::setVelocity(cv::Point3f velocity) {
    _velocity = velocity;
}
