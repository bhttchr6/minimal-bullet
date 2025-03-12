#include <iostream>
#include <vector>
#include <../third/glm/glm.hpp>
#include <../third/glm/gtc/matrix_transform.hpp>
#include <../third/glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include "BulletDynamics/btBulletDynamicsCommon.h"

// Camera class for handling movement
class Camera {
public:
    float x, y, z;
    Camera() : x(0), y(5), z(10) {}
    void move(float dx, float dy, float dz) {
        x += dx;
        y += dy;
        z += dz;
    }
};

// Global variables
Camera camera;
btDiscreteDynamicsWorld* dynamicsWorld;
std::vector<btRigidBody*> bodies;

void initPhysics() {
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    dynamicsWorld->setGravity(btVector3(0, -9.8, 0));

    // Ground plane
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
    btDefaultMotionState* groundMotion = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotion, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundBody);
}

void addSphere(float radius, float x, float y, float z, float mass) {
    btCollisionShape* sphereShape = new btSphereShape(radius);
    btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, y, z)));
    
    btVector3 inertia(0, 0, 0);
    if (mass > 0)
        sphereShape->calculateLocalInertia(mass, inertia);

    btRigidBody::btRigidBodyConstructionInfo sphereRigidBodyCI(mass, motionState, sphereShape, inertia);
    btRigidBody* sphereBody = new btRigidBody(sphereRigidBodyCI);
    dynamicsWorld->addRigidBody(sphereBody);
    bodies.push_back(sphereBody);
}

void updatePhysics(float deltaTime) {
    dynamicsWorld->stepSimulation(deltaTime, 10);
}

void renderSphere(float radius, int slices, int stacks) {
    for (int i = 0; i <= stacks; ++i) {
        float lat0 = M_PI * (-0.5 + (float)(i - 1) / stacks);
        float z0  = radius * sin(lat0);
        float zr0 = radius * cos(lat0);

        float lat1 = M_PI * (-0.5 + (float)i / stacks);
        float z1 = radius * sin(lat1);
        float zr1 = radius * cos(lat1);

        glBegin(GL_TRIANGLE_STRIP);
        for (int j = 0; j <= slices; ++j) {
            float lng = 2 * M_PI * (float)(j - 1) / slices;
            float x = cos(lng);
            float y = sin(lng);

            glVertex3f(x * zr0, y * zr0, z0);
            glVertex3f(x * zr1, y * zr1, z1);
        }
        glEnd();
    }
}

void renderScene() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    glm::mat4 view = glm::lookAt(glm::vec3(camera.x, camera.y, camera.z), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    
    // Render ground plane
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_QUADS);
        glVertex3f(-10.0f, -1.0f, -10.0f);
        glVertex3f(10.0f, -1.0f, -10.0f);
        glVertex3f(10.0f, -1.0f, 10.0f);
        glVertex3f(-10.0f, -1.0f, 10.0f);
    glEnd();
    
    // Render spheres
    glColor3f(1.0f, 0.0f, 0.0f);
    for (btRigidBody* body : bodies) {
        btTransform transform;
        body->getMotionState()->getWorldTransform(transform);
        btVector3 pos = transform.getOrigin();

        glPushMatrix();
        glTranslatef(pos.x(), pos.y(), pos.z());
        renderSphere(1.0f, 20, 20);
        glPopMatrix();
    }
}

void cleanup() {
    for (btRigidBody* body : bodies) {
        delete body->getMotionState();
        delete body->getCollisionShape();
        dynamicsWorld->removeRigidBody(body);
        delete body;
    }
    delete dynamicsWorld;
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return -1;
    }
    
    GLFWwindow* window = glfwCreateWindow(800, 600, "Physics Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glEnable(GL_DEPTH_TEST);
    
    initPhysics();
    addSphere(1.0f, 0, 10, 0, 1.0f);
    
    while (!glfwWindowShouldClose(window)) {
        updatePhysics(1.0f / 60.0f);
        renderScene();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    cleanup();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
