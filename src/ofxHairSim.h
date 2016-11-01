#ifndef _OFX_HAIR_SIM_
#define _OFX_HAIR_SIM_

#include "ofMain.h"
#include "ofxHairModel.h"
#include "ofxHairBoundary.h"
#include "ofxHairCollision.h"
#include "ofxNearestNeighbour.h"
#include "ofxHairUtil.h"

// C++11
#include <thread>
#include <atomic>

enum CollisionType {
	ClosestPointOnTriangle = 1,
	ClosestPointOnTriangleFromVertex = 2,
	ClosestPointOnTriangleKdTree = 3,
};

struct weightInfo {
	int index;
	vector<int> guideIndices;
	vector<float> weight;
};

class ofxHairSim
{
public:
	ofxHairSim();
	~ofxHairSim();
	void init(ofxHairModel &model);
	void initGuide(ofxHairModel &model);
	void update(ofMatrix4x4 mRotate);
	void reset();

	/*  setting */
	void setTimeStep(const float timestep) { m_timestep = timestep; }
	void setDamping(const float damping) { m_damping = damping; }
	void setCollisionType(CollisionType type) { m_collisionType = type; }
	void setSupportRadius(float radius) { m_collision.setRadius(radius); }
	void setFloorHeight(float Ypos) { bFloorContact = true; m_floor_Y = Ypos; }

	/* get data */
	float getTimeStep() { return m_timestep; }
	float getDamping() { return m_damping; }

	/* for collision */
	bool loadBoundaryOBJ(string filename);
	void updateBoundary(vector<ofPoint>  &points);
	std::vector<ofPoint*> getParticlePosition() { return m_particlesPos; }
	CollisionType getCollisionType() { return m_collisionType; }

	/* utility */
	void modelHairFitting(ofxHairModel& hair, float radius);

	/* skinning */
	vector<weightInfo> m_weightInfo;
	map<int, int> mapIdx; // number of guide index -> all index
	void deformation(ofxHairModel &model, ofxHairModel &guide);
	void calcSkinWeight(int linkNum, ofxHairModel &model);
	void calcSkinWeight2(int linkNum, ofxHairModel &model);
	bool loadSkinWeight(const string filename);
	void initSkeleton();
	void updateSkeleton();
	
	//void drawAxis();

private:

	/* simulation parameter */
	float m_timestep;
	float m_damping;
	float m_floor_Y;
	bool bFloorContact;
	ofVec3f m_gravity;

	ofxHairModel *hair;
	ofxHairBoundary boundary;
	ofxHairCollision m_collision;
	CollisionType m_collisionType;

	std::vector<ofxHairParticle*> m_particles;
	std::vector<ofVec3f*> m_particlesPos;
	std::vector<ofVec3f> m_particlesPosition;

	/* simulation function */
	void updateForce();
	void updateVelocity();
	void solveCollision();
	void solveConstraint(ofMatrix4x4 mRotate);
		void applyLocalShapeConstrain(ofMatrix4x4 headRot);
		void applyLocalShapeConstrain2(ofMatrix4x4 headRot);
		void applyConstrainHARADA12(ofMatrix4x4 mRotate);
		void applyConstrainCEIG15(float Sg, float Sl, ofMatrix4x4 mRotate);
	void updatePositionAndVelocity();
};
#endif