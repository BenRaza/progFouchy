#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include "glm/fwd.hpp"
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

class Boids {
private:
    glm::vec2 position;
    glm::vec2 direction;
    glm::vec2 speed;
    glm::vec2 acceleration;

public:
    glm::vec2 getPosition()
    {
        return position;
    }
    glm::vec2 getDirection()
    {
        return direction;
    }
    glm::vec2 getSpeed()
    {
        return speed;
    }
    glm::vec2 getAcceleration()
    {
        return acceleration;
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
    return RandomFloat(0.f, 360.f);
}

glm::vec2 vecSpeed(float speed, float acceleration, float angle)
{
    return {(speed + acceleration) * std::cos(angle), (speed + acceleration) * std::sin(angle)};
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
    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    ctx.maximize_window();
    float howFast = 0.01;

    std::vector<glm::vec2> pos;
    for (size_t i = 0; i < 100; i++)
    {
        glm::vec2 nexVec(RandomFloat(-ctx.aspect_ratio(), ctx.aspect_ratio()), RandomFloat(-1., 1.));
        pos.push_back(nexVec);
    }
    std::vector<glm::vec2> speed;
    for (size_t i = 0; i < 100; i++)
    {
        glm::vec2 newSpeed(RandomFloat(-howFast, howFast), RandomFloat(-howFast, howFast));
        speed.push_back(newSpeed);
    }

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::PurpleHeart);
        ctx.circle(
            p6::Center{ctx.mouse()},
            p6::Radius{0.2f}
        );

        for (int i = 0; i < 100; i++)
        {
            pos[i] += speed[i];
            ctx.square(
                p6::Center{pos[i]},
                p6::Radius{0.05f}
            );
        }
    };

    // Should be done last. It starts the infinite loop.
    ctx.start();

    return 0;
}