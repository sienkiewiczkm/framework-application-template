#pragma once

#include <memory>

#include "glm/glm.hpp"

#include "fw/FrameMarker.hpp"
#include "fw/Grid.hpp"
#include "fw/ImGuiApplication.hpp"
#include "fw/Mesh.hpp"
#include "fw/OrbitingCamera.hpp"
#include "fw/TexturedPhongEffect.hpp"
#include "fw/UniversalPhongEffect.hpp"
#include "fw/Vertices.hpp"

#include "bullet/btBulletDynamicsCommon.h"

namespace application
{

class Application:
    public fw::ImGuiApplication
{
public:
    Application();
    virtual ~Application();

protected:
    virtual void onCreate() override;
    virtual void onDestroy() override;
    virtual void onUpdate(
        const std::chrono::high_resolution_clock::duration& deltaTime
    ) override;
    virtual void onRender() override;

    virtual bool onMouseButton(int button, int action, int mods) override;
    virtual bool onMouseMove(glm::dvec2 newPosition) override;
    virtual bool onScroll(double xoffset, double yoffset) override;
    virtual bool onResize() override;

    void updateProjectionMatrix();

    void createBullet();
    void createPhysicsScene();
    void destroyBullet();

    btRigidBody* createRigidBody(
        btCollisionShape* shape,
        float mass,
        const btTransform& transform
    );

private:
    std::shared_ptr<fw::TexturedPhongEffect> _phongEffect;
    std::shared_ptr<fw::UniversalPhongEffect> _universalPhongEffect;

    std::shared_ptr<fw::Mesh<fw::VertexNormalTexCoords>> _cube;

    std::shared_ptr<fw::Grid> _grid;
    std::shared_ptr<fw::FrameMarker> _frameMarker;

    fw::OrbitingCamera _camera;
    glm::mat4 _projectionMatrix;
    bool _enableCameraRotations;

    glm::dvec2 _cameraRotationSensitivity;
    GLuint _testTexture;

    bool _enablePhysicsStepping;

    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _collisionDispatcher;
    btBroadphaseInterface* _overlappingPairCache;
    btSequentialImpulseConstraintSolver* _constraintSolver;
    btDiscreteDynamicsWorld* _dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> _collisionShapes;
    btRigidBody* _groundRigidBody;
    btRigidBody* _cubeRigidBody;
};

}
