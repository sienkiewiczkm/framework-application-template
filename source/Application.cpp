#include "Application.hpp"

#include <iostream>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "imgui.h"

#include "fw/Common.hpp"
#include "fw/DebugShapes.hpp"
#include "fw/Resources.hpp"
#include "fw/TextureUtils.hpp"

namespace application
{

Application::Application():
    _enablePhysicsStepping{false},
    _enableCameraRotations{false},
    _cameraRotationSensitivity{0.2, 0.2}
{
}

Application::~Application()
{
}

void Application::onCreate()
{
    ImGuiApplication::onCreate();

    _phongEffect = std::make_shared<fw::TexturedPhongEffect>();
    _phongEffect->create();

    _universalPhongEffect = std::make_shared<fw::UniversalPhongEffect>();

    _cube = fw::createBox({1.0, 1.0, 1.0});
    _grid = std::make_shared<fw::Grid>(
        glm::ivec2{32, 32},
        glm::vec2{0.5f, 0.5f}
    );

    _frameMarker = std::make_shared<fw::FrameMarker>();

    _testTexture = fw::loadTextureFromFile(
        fw::getFrameworkResourcePath("textures/checker-base.png")
    );

    _camera.rotate(fw::pi()/4, -3.0*fw::pi()/4);
    _camera.setDist(3.0f);

    updateProjectionMatrix();

    createBullet();
    createPhysicsScene();
}

void Application::onDestroy()
{
    destroyBullet();
    ImGuiApplication::onDestroy();
}

void Application::onUpdate(
    const std::chrono::high_resolution_clock::duration& deltaTime
)
{
    std::chrono::duration<double> deltaTimeSeconds{deltaTime};
    ImGuiApplication::onUpdate(deltaTime);

    ImGui::Checkbox("Enable physics", &_enablePhysicsStepping);

    if (_enablePhysicsStepping)
    {
        _dynamicsWorld->stepSimulation(
            static_cast<float>(deltaTimeSeconds.count()),
            10
        );
    }
}

void Application::onRender()
{
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    float mtx[16];
    btTransform cubeTransform;
    _cubeRigidBody->getMotionState()->getWorldTransform(cubeTransform);
    cubeTransform.getOpenGLMatrix(mtx);
    glm::mat4 cubeWorldMatrix = glm::make_mat4(mtx) * glm::scale(
        glm::mat4{},
        glm::vec3{2.0f, 2.0f, 2.0f}
    );

    _phongEffect->begin();
    _phongEffect->setProjectionMatrix(_projectionMatrix);
    _phongEffect->setViewMatrix(_camera.getViewMatrix());
    _phongEffect->setModelMatrix(cubeWorldMatrix);
    _phongEffect->setTexture(_testTexture);
    _cube->render();
    _phongEffect->setModelMatrix({});
    _grid->render();
    _phongEffect->end();

    for (const auto& chunk: _frameMarker->getGeometryChunks())
    {
        _universalPhongEffect->setLightDirection({-1, 1, 0});
        _universalPhongEffect->setSolidColor(
            chunk.getMaterial()->getBaseAlbedoColor()
        );

        _universalPhongEffect->begin();
        // todo: standarize a way to change uniforms
        _universalPhongEffect->setProjectionMatrix(_projectionMatrix);
        _universalPhongEffect->setViewMatrix(_camera.getViewMatrix());
        _universalPhongEffect->setModelMatrix(chunk.getModelMatrix());
        chunk.getMesh()->render();
        _universalPhongEffect->end();
    }

    ImGuiApplication::onRender();
}

bool Application::onMouseButton(int button, int action, int mods)
{
    if (ImGuiApplication::onMouseButton(button, action, mods)) { return true; }

    if (GLFW_MOUSE_BUTTON_LEFT == button)
    {
        _enableCameraRotations = GLFW_PRESS == action;
    }

    return true;
}

bool Application::onMouseMove(glm::dvec2 newPosition)
{
    if (ImGuiApplication::onMouseMove(newPosition)) { return true; }

    if (_enableCameraRotations)
    {
        auto movement = getMouseMovement() * _cameraRotationSensitivity;
        _camera.rotate(movement.y, movement.x);
    }

    return true;
}

bool Application::onScroll(double xoffset, double yoffset)
{
    if (fw::ImGuiApplication::onScroll(xoffset, yoffset))
        return true;

    const double cMinimumDistance = 1.0;
    const double cMaximumDistance = 10.0;
    const double cZoomStep = 0.5;
    auto currentDistance = _camera.getDist();
    _camera.setDist(
        std::min(
            std::max(currentDistance + cZoomStep * yoffset, cMinimumDistance),
            cMaximumDistance
        )
    );

    return true;
}

bool Application::onResize()
{
    updateProjectionMatrix();
    return true;
}

void Application::updateProjectionMatrix()
{
    auto windowSize = getWindowSize();
    auto aspectRatio = static_cast<float>(windowSize.x) / windowSize.y;
    _projectionMatrix = glm::perspective(45.0f, aspectRatio, 0.5f, 100.0f);
}

void Application::createBullet()
{
    _collisionConfiguration = new btDefaultCollisionConfiguration();
    _collisionDispatcher = new btCollisionDispatcher(_collisionConfiguration);
    _overlappingPairCache = new btDbvtBroadphase();
    _constraintSolver = new btSequentialImpulseConstraintSolver;

    _dynamicsWorld = new btDiscreteDynamicsWorld(
        _collisionDispatcher,
        _overlappingPairCache,
        _constraintSolver,
        _collisionConfiguration
    );

    _dynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void Application::createPhysicsScene()
{
    auto groundShape = new btBoxShape(
        btVector3(btScalar(50.), btScalar(50.), btScalar(50.))
    );

    auto cubeShape = new btBoxShape(
        btVector3(btScalar(1.f), btScalar(1.), btScalar(1.))
    );

    _collisionShapes.push_back(groundShape);
    _collisionShapes.push_back(cubeShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3{0.f, -50.f, 0.f});
    _groundRigidBody = createRigidBody(groundShape, 0.f, groundTransform);

    btTransform cubeTransform;
    cubeTransform.setIdentity();
    cubeTransform.setOrigin(btVector3{0.f, 5.f, 0.f});

    _cubeRigidBody = createRigidBody(cubeShape, 0.25f, cubeTransform);
}

void Application::destroyBullet()
{
    delete _dynamicsWorld;
    _dynamicsWorld = nullptr;

	delete _constraintSolver;
    _constraintSolver = nullptr;

	delete _overlappingPairCache;
    _overlappingPairCache = nullptr;

	delete _collisionDispatcher;
    _collisionDispatcher = nullptr;

	delete _collisionConfiguration;
    _collisionConfiguration = nullptr;
}

btRigidBody* Application::createRigidBody(
    btCollisionShape* shape,
    float mass,
    const btTransform& transform
)
{
    bool isDynamic = mass != 0.0f;
    btVector3 localInertia{0.0f, 0.0f, 0.0f};

    if (isDynamic)
    {
        shape->calculateLocalInertia(mass, localInertia);
    }

    auto motionState = new btDefaultMotionState(transform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(
        mass,
        motionState,
        shape,
        localInertia
    );

    auto rigidBody = new btRigidBody(cInfo);
    rigidBody->setUserIndex(-1);
    _dynamicsWorld->addRigidBody(rigidBody);

    return rigidBody;
}

}
