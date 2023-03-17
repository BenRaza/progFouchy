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
#include "../include/boids.hpp"
#include "doctest/doctest.h"

std::random_device rd;
std::mt19937       mt(rd());

float RandomFloat(float a, float b)
{
    float random = ((float)rand()) / (float)RAND_MAX;
    float diff   = b - a;
    float r      = random * diff;
    return a + r;
}

int main(int argc, char* argv[])
{
    auto                                   ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    std::uniform_real_distribution<double> randAngle(0, 2 * PI);
    std::uniform_real_distribution<double> randHeight(-1, 1);
    std::uniform_real_distribution<double> randWidth(-ctx.aspect_ratio(), ctx.aspect_ratio());
    float                                  screenRatio = ctx.aspect_ratio();
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
        float angle = randAngle(mt);
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
            boid.useForce(acc, screenRatio);
        }
    };

    // Should be done last. It starts the infinite loop.
    ctx.start();

    return 0;
}