#include <iostream>
#include <vector>
#include <bullet/btBulletDynamicsCommon.h>

const int NUM_AGENTS = 1000;
const double AGENT_RADIUS = 0.3;
const double AGENT_HEIGHT = 1.8;
const double AGENT_MASS = 80;
const double AGENT_MAX_FORCE = 100;
const double AGENT_MAX_SPEED = 2;
const double AGENT_NEIGHBOR_DISTANCE = 2;
const double AGENT_AVOIDANCE_FORCE = 100;
const double AGENT_GOAL_FORCE = 100;

class Agent {
public:
    Agent(btDynamicsWorld* world, const btVector3& position) : world_(world) {
        btCapsuleShape* shape = new btCapsuleShape(AGENT_RADIUS, AGENT_HEIGHT);
        btDefaultMotionState* state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), position));
        btRigidBody::btRigidBodyConstructionInfo info(AGENT_MASS, state, shape);
        body_ = new btRigidBody(info);
        world_->addRigidBody(body_);
    }

    void update(const std::vector<Agent>& agents, const btVector3& goal) {
        btVector3 velocity = body_->getLinearVelocity();
        btVector3 acceleration = computeAcceleration(agents, goal);
        velocity += acceleration;
        velocity = velocity.clamped(btVector3(-AGENT_MAX_SPEED, -AGENT_MAX_SPEED, -AGENT_MAX_SPEED), btVector3(AGENT_MAX_SPEED, AGENT_MAX_SPEED, AGENT_MAX_SPEED));
        body_->setLinearVelocity(velocity);
    }

    btVector3 computeAcceleration(const std::vector<Agent
