//
// Created by Ronal on 4/28/2024.
//
#pragma once
#include <opencv2/opencv.hpp>

enum mode{tank = 0, normal, sport};
// todo doxygen
class CMecanumMove {
private:
    cv::Point3f _acceleration;

    cv::Point3f _velocity;

    bool _threadExit;

    static void moveThread();

public:
    CMecanumMove();

    ~CMecanumMove();

    void setAcceleration(cv::Point3f acceleration);

    void setVelocity(cv::Point3f velocity);

    void move();
};
