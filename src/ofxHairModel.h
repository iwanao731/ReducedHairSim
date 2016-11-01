#ifndef _OFX_HAIR_MODEL_
#define _OFX_HAIR_MODEL_

#include "ofMain.h"
#include "ofxHairStrand.h"
#include "ofxHairParticle.h"

struct VertexIntermediate {
	ofVec3f mPosition;
	ofVec3f mColor;
	GLfloat mArBlendingIndex[4];
	GLfloat mArBlendingWeight[4];
	GLfloat mVertIndex;
};

struct JointIntermediate {
	int mParentIndex;
	ofVec3f mPos;
	ofQuaternion mRotate;
	ofVec4f mTranslation;
	ofQuaternion mRotate0;
	ofVec4f mTranslation0;
};

struct ParticleIntermediate {
	ofVec3f position;
	ofVec3f velocity;
	float lestLength;
	int indices;
	ofVec3f position0;
	ofVec3f tmp_position;
	ofVec3f forces;
	ofVec3f collisionF;
	ofVec3f d;

	float mass;
	float inv_mass;
	bool enabled;

	/* GEIG2015 */
	ofVec3f posPrev;
	ofVec3f dGlobal;
	ofVec3f dLocal;
	ofVec3f dInex;
};

class ofxHairModel
{
public:
	/* new */
	vector<VertexIntermediate> mNormalVertices;		// Normal Particles;
	vector<JointIntermediate> mGuideJoints;			// Guide Particles
	vector<ParticleIntermediate> mSimParticles;		// Simulation Particles as Guide Particle

	/* old */
	std::vector<ofxHairStrand> strands;

	void addHairStrand(const ofVec3f position, const ofVec3f normal, const float length, const int resolution);
	int getNumParticles() { return m_numParticles; };
	int getNumStrand() { return m_numStrands; }
	bool loadHairModel(string filename);
	bool exportHairModel(string filename);
	bool loadHairModelAsText(string filename);
	bool loadGuideHair(string filename);
	bool exportHairModelAsText(string filename);
	void buildJointHierarchy();
	void buildJointMatrix();
	void updateJointMatrix();
	void resetGuideAndNormal();

	bool exportGuideHairModel(string filename);


private:
	int m_numStrands;
	int m_numParticles;
};

#endif