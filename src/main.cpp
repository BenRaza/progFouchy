#include <cmath>
#include <cstdlib>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <random>
#include <vector>
#include "glm/ext/scalar_constants.hpp"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

auto  ctx = p6::Context{{.title = "Simple-p6-Setup"}};
float PI  = glm::pi<float>();

std::random_device                     rd;
std::mt19937                           mt(rd());
std::uniform_real_distribution<double> randPosition(0, 2 * PI);
std::uniform_real_distribution<double> randHeight(-1, 1);
std::uniform_real_distribution<double> randWidth(-ctx.aspect_ratio(), ctx.aspect_ratio());

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
    void setPosition(glm::vec2 newPos)
    {
        m_position = newPos;
        if (m_position.x > ctx.aspect_ratio())
        {
            m_position.x = ctx.aspect_ratio() - 0.01;
        }
        else if (m_position.x < -ctx.aspect_ratio())
        {
            m_position.x = -ctx.aspect_ratio() + 0.01;
        }
        if (m_position.y > 1)
        {
            m_position.y = 0.99;
        }
        else if (m_position.y < -1)
        {
            m_position.y = -0.99;
        }
    }
    glm::vec2 getDirection()
    {
        return m_direction;
    }
    void setDirection(glm::vec2 newDir)
    {
        if (glm::distance(glm::vec2(0.f, 0.f), newDir) > m_speed || glm::distance(glm::vec2(0.f, 0.f), newDir) < m_speed - .25)
        {
            m_angle     = glm::atan<float>(newDir.x, newDir.y);
            m_direction = glm::vec2(m_speed * glm::cos(m_angle), m_speed * glm::sin(m_angle));
        }
        else
        {
            m_direction = newDir;
        }
    }
    float getAngle()
    {
        return m_angle;
    }
    void setAngle(float newAngle)
    {
        m_angle     = std::fmod(newAngle, (2.f * PI));
        m_direction = glm::vec2(m_speed * cos(m_angle), m_speed * sin(m_angle));
    }
    void useForce(glm::vec2 extForce)
    {
        const glm::vec2 newForce = extForce * .0005f;
        setDirection(getDirection() + newForce);
        if (getPosition().x + getDirection().x > ctx.aspect_ratio() || getPosition().x + getDirection().x < -ctx.aspect_ratio())
        {
            m_direction.x *= -1;
        }
        if (getPosition().y + getDirection().y > 1.f || getPosition().y + getDirection().y < -1.f)
        {
            m_direction.y *= -1;
        }
        m_position += m_direction;
    }
    // glm::vec2 getDirection()
    // {
    //     return direction;
    // }
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
    glm::vec2 calculateSeparationForce(std::vector<Boids> boids)
    {
        glm::vec2 totalForce(0.f, 0.f);
        for (Boids& boid : boids)
        {
            float test = glm::distance(getPosition(), boid.getPosition());
            if (glm::distance(getPosition(), boid.getPosition()) < .15f && glm::distance(getPosition(), boid.getPosition()) > .001f)
            {
                totalForce.x += ((getPosition().x - boid.getPosition().x) / glm::distance(getPosition(), boid.getPosition()));
                totalForce.y += ((getPosition().y - boid.getPosition().y) / glm::distance(getPosition(), boid.getPosition()));
            }
        }
        return totalForce;
    }
};

float RandomFloat(float a, float b)
{
    float random = ((float)rand()) / (float)RAND_MAX;
    float diff   = b - a;
    float r      = random * diff;
    return a + r;
}

float RandomAngle()
{
    return (randPosition(mt));
}

int main(int argc, char* argv[])
{
    { // Run the tests
        if (doctest::Context{}.run() != 0)
        {
            return EXIT_FAILURE;
        }
        // The CI does not have a GPU so it cannot run the rest of the code.
        const bool no_gpu_available = argc >= 2 && strcmp(argv[1], "-nogpu") == 0; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (no_gpu_available)
        {
            return EXIT_SUCCESS;
        }
    }

    // Actual app
    // auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    ctx.maximize_window();
    float        speed           = 0.007;
    unsigned int NUMBER_OF_BOIDS = 50;

    std::vector<Boids> boids;

    for (size_t i = 0; i < NUMBER_OF_BOIDS; i++)
    {
        glm::vec2 nexVec(randWidth(mt), randHeight(mt));
        boids.push_back(Boids(nexVec, speed));
        float angle = RandomAngle();
        boids[i].setAngle(angle);
        boids[i].setDirection(glm::vec2(speed * glm::cos(angle), speed * glm::sin(angle)));
    }

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::PurpleHeart);
        // ctx.circle(
        //     p6::Center{ctx.mouse()},
        //     p6::Radius{0.2f}
        // );

        for (Boids& boid : boids)
        {
            ctx.square(
                p6::Center{boid.getPosition()},
                p6::Radius{0.05f}
            );
            glm::vec2 acc = boid.calculateSeparationForce(boids);
            // // boid.setDirection(boid.getDirection() + acc);
            boid.useForce(acc);
        }
    };

    // Should be done last. It starts the infinite loop.
    ctx.start();

    return 0;
}