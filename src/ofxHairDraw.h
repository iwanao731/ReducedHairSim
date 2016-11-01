#ifndef _OFX_HAIR_DRAW_
#define _OFX_HAIR_DRAW_

#include "ofMain.h"
#include "ofxHairModel.h""

class ofxHairDraw
{
public:
	ofxHairDraw(ofxHairModel &model);
	void init(string filename);
	void update(ofxHairModel &model, ofEasyCam &cam);
	void draw(ofxHairModel &model, ofEasyCam &cam);
	void drawOld(ofxHairModel &model);
	ofxHairDraw& setDrawHairColor(bool v);
	ofxHairDraw& setDrawHairParticles(bool v);
	ofxHairDraw& setDrawHairEdges(bool v);
	ofxHairDraw& setDrawHairNormal(bool v);
	ofxHairDraw& setDrawHairGuide(bool v);

	/* Hair Rendering */
	bool hasSkin() const { return static_cast<bool>(skinFbo); }
	ofFbo& getSkinFbo() const { return *skinFbo.get(); }
	std::vector<ofVec3f> getSkinVertices() const;

private:
	ofxHairModel& m_model;
	bool bColor;
	bool bNode;
	bool bEdge;
	bool bNormalHair;
	bool bGuideHair;

	ofShader shader;
	unsigned int mVbo;

	int locate_pos;
	int locate_rot;

	int posID;
	int rotID;
	int boneNum;

	ofVec4f *translations;
	ofQuaternion *rotations;

	std::unique_ptr<ofFbo> skinFbo;
};

#endif