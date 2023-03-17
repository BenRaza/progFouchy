#pragma once

#include <cmath>
#include <iostream>
#include <vector>
#include "glm/ext/scalar_constants.hpp"
#include "p6/p6.h"

extern float PI;

class Boids {
private:
    glm::vec2 m_position;
    glm::vec2 m_direction;
    float     m_speed;
    float     m_angle;
    // glm::vec2 acceleration;

public:
    Boids(glm::vec2 position, float speed)
        : m_position(position), m_speed(speed){};

    glm::vec2 getPosition()
    {
        return m_position;
    }
    void setPosition(glm::vec2 newPos, p6::Context ctx);

    glm::vec2 getDirection()
    {
        return m_direction;
    }
    void  setDirection(glm::vec2 newDir);
    float getAngle()
    {
        return m_angle;
    }
    void setAngle(float newAngle)
    {
        m_angle     = std::fmod(newAngle, (2.f * PI));
        m_direction = glm::vec2(m_speed * cos(m_angle), m_speed * sin(m_angle));
    }
    void  useForce(glm::vec2 extForce, float screen);
    float getSpeed()
    {
        return m_speed;
    }
    void setSpeed(float newSpeed)
    {
        m_speed = newSpeed;
    }
    // glm::vec2 getAcceleration()
    // {
    //     return acceleration;
    // }
    glm::vec2 calculateSeparationForce(std::vector<Boids> boids);
};