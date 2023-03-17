#include "../include/boids.hpp"

float PI = glm::pi<float>();

void Boids::setPosition(glm::vec2 newPos, p6::Context ctx)
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

void Boids::setDirection(glm::vec2 newDir)
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

void Boids::useForce(glm::vec2 extForce, float screen)
{
    const glm::vec2 newForce = extForce * .0005f;
    setDirection(getDirection() + newForce);
    if (getPosition().x + getDirection().x > screen || getPosition().x + getDirection().x < -screen)
    {
        m_direction.x *= -1;
    }
    if (getPosition().y + getDirection().y > 1.f || getPosition().y + getDirection().y < -1.f)
    {
        m_direction.y *= -1;
    }
    m_position += m_direction;
}

glm::vec2 Boids::calculateSeparationForce(std::vector<Boids> boids)
{
    glm::vec2 totalForce(0.f, 0.f);
    for (Boids& boid : boids)
    {
        float test = glm::distance(getPosition(), boid.getPosition());
        if (test < .15f && test > .001f)
        {
            totalForce.x += ((getPosition().x - boid.getPosition().x) / glm::distance(getPosition(), boid.getPosition()));
            totalForce.y += ((getPosition().y - boid.getPosition().y) / glm::distance(getPosition(), boid.getPosition()));
        }
    }
    return totalForce;
}