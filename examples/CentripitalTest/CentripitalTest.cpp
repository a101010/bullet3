
#include "CentripitalTest.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

struct CentriptialTest : public CommonRigidBodyBase
{
	CentriptialTest (struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~CentriptialTest(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		//float pitch = -35;
		float pitch = 10;
		float yaw = 230;
		//float yaw = 52;
		//static float yaw = 90;
		//const float yawstep = 1;
		//yaw += yawstep;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
};

void CentriptialTest::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-.51,0));
	//m_dynamicsWorld->setGravity(btVector3(0,-.4,0));
	//m_dynamicsWorld->setGravity(btVector3(0, -.39, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	
	btTriangleMesh* gerbilWheel = new btTriangleMesh();
	float scale = 1.;
	/*btVector3 quad[] = {
		btVector3(.0, 1.0, .0) * scale,
		btVector3(1.0, .0, .0) * scale,
		btVector3(1.0, 1.0, .0) * scale,
		btVector3(.0, .0, .0) * scale
	};*/

	btVector3 quad[] = {
		btVector3(.0, .0, 1.0) * scale,
		btVector3(1.0, .0, .0) * scale,
		btVector3(1.0, .0, 1.0) * scale,
		btVector3(.0, .0, .0) * scale
	};
	gerbilWheel->addTriangle(quad[3], quad[2], quad[0], false);
	gerbilWheel->addTriangle(quad[1], quad[2], quad[3], false);
	btBvhTriangleMeshShape* gerbilWheelCollider = new btBvhTriangleMeshShape(gerbilWheel, true, true);
	m_collisionShapes.push_back(gerbilWheelCollider);

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-51,0));

	{
		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}

	btTransform gerbilWheelTransform;
	gerbilWheelTransform.setIdentity();
	gerbilWheelTransform.setOrigin(btVector3(0, 0, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, gerbilWheelTransform, gerbilWheelCollider);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1,.1,.1));
		

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

			
					createRigidBody(mass,startTransform,colShape);
					

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







CommonExampleInterface*    CentripitalTestCreateFunc(CommonExampleOptions& options)
{
	return new CentriptialTest(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(CentriptialTestCreateFunc)



