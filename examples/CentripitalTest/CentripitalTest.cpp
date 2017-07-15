
#include "CentripitalTest.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 2 // 3+ on my machine are unacceptably slow
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"


#include <vector>
#include <stdexcept>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI // The above two lines fail to define M_PI
#define M_PI 3.14159265358979323846
#endif



struct CentriptialTest : public CommonRigidBodyBase
{
	CentriptialTest (struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~CentriptialTest(){}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation(float deltaTime);
	void resetCamera()
	{
		float dist = 10;
		float pitch = -25;
		//float pitch = 10;
		//static float yaw = 360;
		//float yaw = 52;
		float yaw = 110;
		//static float yaw = 280;
		//static float yaw = 90;
		const float yawstep = 1;
		yaw += yawstep;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}

	// TODO generateTubeMeshX should not rely on member vectors; this is a temporary hack while I think about the design
	void generateTubeMeshX(float i_radius, float o_radius, float length, unsigned int subSteps, btTriangleIndexVertexArray& mesh);
	std::vector<btVector3> gerbilWheelVertices;
	std::vector<unsigned int> gerbilWheelIndices;
	btTriangleIndexVertexArray gerbilWheelArray;
	btBvhTriangleMeshShape* gerbilWheelShape;
	btTransform* gerbilWheelTransform;

};

class WheelMotionState : public btMotionState {
public: 
	WheelMotionState(btScalar angVelocity, btVector3& axis);
	virtual void getWorldTransform(btTransform& worldTrans) const;
	virtual void setWorldTransform(const btTransform& worldTrans);

private:
	btScalar angVelocity;
	btVector3 axis;
	btTransform worldTransform;

};

WheelMotionState::WheelMotionState(btScalar angVelocity, btVector3& axis) :
	// TODO add a stepSimulation(float deltaTime) to use angVelocity and axis
	angVelocity(angVelocity),
	axis(axis) {
	worldTransform.setIdentity(); 
}

void WheelMotionState::getWorldTransform(btTransform& worldTrans) const {
	worldTrans = worldTransform;
}

void WheelMotionState::setWorldTransform(const btTransform& worldTrans) {
	worldTransform = worldTrans;
}

void CentriptialTest::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		// TODO move this into WheelMotionState. Add a stepSimulation(float deltaTime) to the motionstate.
		btTransform tr;
		dynamic_cast<btRigidBody*>(m_dynamicsWorld->getCollisionObjectArray()[0])->getMotionState()->getWorldTransform(tr);
		static float angle = 0.f;
		angle += 0.01f;
		tr.setRotation(btQuaternion(btVector3(1, 0, 0), angle));
		dynamic_cast<btRigidBody*>(m_dynamicsWorld->getCollisionObjectArray()[0])->getMotionState()->setWorldTransform(tr);

		// TODO is there a way to keep cubes from going through the tube without changing the fixed timestep?
		m_dynamicsWorld->stepSimulation(deltaTime, 30, 1./120.);
	}
}

void CentriptialTest::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0, -10., 0));
	//m_dynamicsWorld->setGravity(btVector3(0,-.51,0));

	// TODO gravity below .5 allows cubes to get stuck (activation state? but velocity is increasing...)
	//m_dynamicsWorld->setGravity(btVector3(0,-.4,0));
	//m_dynamicsWorld->setGravity(btVector3(0, -.39, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	
	generateTubeMeshX(4.75, 5.0, 6., 4, gerbilWheelArray);
	gerbilWheelShape = new btBvhTriangleMeshShape(&gerbilWheelArray, true, true);
	m_collisionShapes.push_back(gerbilWheelShape);

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-51,0));

	{
		btScalar mass(0.);
		//createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}

	gerbilWheelTransform = new btTransform();
	gerbilWheelTransform->setIdentity();
	gerbilWheelTransform->setOrigin(btVector3(0, 0, 0));
	{
		btScalar mass(0.);
		btRigidBody* gerbilWheelRigidBody = createRigidBody(mass, *gerbilWheelTransform, gerbilWheelShape,btVector4(1, 0, 0, 1), 5.0);
		
		// flag as kinimatic and prevent deactivation
		gerbilWheelRigidBody->setCollisionFlags(gerbilWheelRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		gerbilWheelRigidBody->setActivationState(DISABLE_DEACTIVATION);

		// KINEMATIC_OBJECTs need to provide their own motion state
		gerbilWheelRigidBody->setMotionState(new WheelMotionState(1., btVector3(1., 0., 0.)));
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1,.1,.1));
		//btBoxShape* colShape = createBoxShape(btVector3(.2, .2, .2));

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										btScalar(0.2*i),
										btScalar(2+.2*k),
										btScalar(0.2*j)));

			
					createRigidBody(mass,startTransform,colShape, btVector4(1, 0, 0, 1), 10.0);
					

				}
			}
		}
	}

	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
}


void CentriptialTest::renderScene()
{
	resetCamera();
	CommonRigidBodyBase::renderScene();
	
}

void generateCircleVertices(float radius, float x, float angleStep, unsigned int vertInCircle, std::vector<btVector3>& vertex) {
	const float M_HALF_PI = M_PI / 2.0;
	const float M_PI_AND_HALF = M_PI + M_HALF_PI;
	const float M_TWO_PI = 2.0 * M_PI;

	float angle = 0.0;
	for (unsigned int index = 0; index < vertInCircle; ++index) {
		vertex.push_back(btVector3(x, radius * sin(angle), radius * cos(angle)));
		angle += angleStep;
	}
}

void 
CentriptialTest::generateTubeMeshX(float i_radius, float o_radius, float length, unsigned int subSteps, btTriangleIndexVertexArray& mesh) {
	const unsigned int MAX_SUB_STEPS = 20;
	if (subSteps > MAX_SUB_STEPS) {
		throw std::invalid_argument("subSteps > MAX_SUB_STEPS");
	}
	unsigned int vertInCircle = (3 << subSteps);
	unsigned int totalVertices = 4 * vertInCircle;

	float halfLength = length / 2.0;
	float angleStep = 2.0 * M_PI / static_cast<float>(vertInCircle);

	// first circle, outer
	generateCircleVertices(o_radius, halfLength, angleStep, vertInCircle, gerbilWheelVertices);

	unsigned int innerCircle1Base = gerbilWheelVertices.size();

	// first circle, inner
	generateCircleVertices(i_radius, halfLength, angleStep, vertInCircle, gerbilWheelVertices);

	unsigned int outerCircle2Base = gerbilWheelVertices.size();

	// second circle, outer
	generateCircleVertices(o_radius, -halfLength, angleStep, vertInCircle, gerbilWheelVertices);

	unsigned int innerCircle2Base = gerbilWheelVertices.size();

	// second circle, inner
	generateCircleVertices(i_radius, -halfLength, angleStep, vertInCircle, gerbilWheelVertices);

	unsigned int outer1 = 0;
	unsigned int outer2 = outerCircle2Base;
	unsigned int inner1 = innerCircle1Base;
	unsigned int inner2 = innerCircle2Base;
	unsigned int limit = vertInCircle - 1;
	
	// indexes for first circle end cap
	for (; outer1 < limit; ++outer1, ++inner1) {
		gerbilWheelIndices.push_back(outer1);
		gerbilWheelIndices.push_back(inner1);
		gerbilWheelIndices.push_back(inner1 + 1);

		gerbilWheelIndices.push_back(outer1);
		gerbilWheelIndices.push_back(inner1 + 1);
		gerbilWheelIndices.push_back(outer1 + 1);
	}
	gerbilWheelIndices.push_back(outer1);
	gerbilWheelIndices.push_back(inner1);
	gerbilWheelIndices.push_back(innerCircle1Base);

	gerbilWheelIndices.push_back(outer1);
	gerbilWheelIndices.push_back(innerCircle1Base);
	gerbilWheelIndices.push_back(0);

	
	// indexes for second circle end cap
	limit = vertInCircle - 1 + outerCircle2Base;
	for (; outer2 < limit; ++outer2, ++inner2) {
		gerbilWheelIndices.push_back(outer2);
		gerbilWheelIndices.push_back(inner2 + 1);
		gerbilWheelIndices.push_back(inner2);

		gerbilWheelIndices.push_back(outer2);
		gerbilWheelIndices.push_back(outer2 + 1);
		gerbilWheelIndices.push_back(inner2 + 1);
	}
	gerbilWheelIndices.push_back(outer2);
	gerbilWheelIndices.push_back(innerCircle1Base);
	gerbilWheelIndices.push_back(inner2);

	gerbilWheelIndices.push_back(outer2);
	gerbilWheelIndices.push_back(outerCircle2Base);
	gerbilWheelIndices.push_back(innerCircle2Base);

	// indexes for outer skin
	outer1 = 0;
	outer2 = outerCircle2Base;
	for (; outer1 < vertInCircle - 1; ++outer1, ++outer2) {
		gerbilWheelIndices.push_back(outer1);
		gerbilWheelIndices.push_back(outer1 + 1);
		gerbilWheelIndices.push_back(outer2);

		gerbilWheelIndices.push_back(outer2);
		gerbilWheelIndices.push_back(outer1 + 1);
		gerbilWheelIndices.push_back(outer2 + 1);
	}
	gerbilWheelIndices.push_back(outer1);
	gerbilWheelIndices.push_back(0);
	gerbilWheelIndices.push_back(outer2);
	
	gerbilWheelIndices.push_back(outer2);
	gerbilWheelIndices.push_back(0);
	gerbilWheelIndices.push_back(outerCircle2Base);
    


	// indexes for inner skin
	inner1 = innerCircle1Base;
	inner2 = innerCircle2Base;
	limit = innerCircle1Base + vertInCircle - 1;
	for (; inner1 < limit; ++inner1, ++inner2) {
		gerbilWheelIndices.push_back(inner1);
		gerbilWheelIndices.push_back(inner2);
		gerbilWheelIndices.push_back(inner1 + 1);

		gerbilWheelIndices.push_back(inner2);
		gerbilWheelIndices.push_back(inner2 + 1);
		gerbilWheelIndices.push_back(inner1 + 1);
	}
	gerbilWheelIndices.push_back(inner1);
	gerbilWheelIndices.push_back(inner2);
	gerbilWheelIndices.push_back(innerCircle1Base);
	
	gerbilWheelIndices.push_back(inner2);
	gerbilWheelIndices.push_back(innerCircle2Base);
	gerbilWheelIndices.push_back(innerCircle1Base);
	

	unsigned int* ind = new unsigned int[gerbilWheelIndices.size()];
	float* vert = new float[gerbilWheelVertices.size() * 3];

	for (unsigned int i = 0; i < gerbilWheelIndices.size(); ++i) {
		ind[i] = gerbilWheelIndices[i];
	}

	unsigned int vertIndex = 0;
	for (unsigned int i = 0; i < gerbilWheelVertices.size(); ++i) {
		vert[vertIndex] = gerbilWheelVertices[i].x();
		vert[vertIndex + 1] = gerbilWheelVertices[i].y();
		vert[vertIndex + 2] = gerbilWheelVertices[i].z();
		vertIndex += 3;
	}


	// TODO this is horrible code
	// - would like to use vector to avoid having to calculate the number of tris
	// - but it would also be nice not to allocate twice
	// - where should the vector memory live?
	// - who owns and will delete the arrays?
	btIndexedMesh m;
	m.m_numTriangles = gerbilWheelIndices.size() / 3;
	m.m_numVertices = gerbilWheelVertices.size();
	m.m_triangleIndexBase = reinterpret_cast<unsigned char*>(ind);
	m.m_triangleIndexStride = 3 * sizeof(unsigned int);
	m.m_vertexBase = reinterpret_cast<unsigned char*>(vert);
	m.m_vertexStride = 3 * sizeof(float);
	m.m_vertexType = PHY_ScalarType::PHY_FLOAT;
	mesh.addIndexedMesh(m);

}




CommonExampleInterface*    CentripitalTestCreateFunc(CommonExampleOptions& options)
{
	return new CentriptialTest(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(CentriptialTestCreateFunc)



