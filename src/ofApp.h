#pragma once

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxHEMesh.h"
#include "ofxHEMeshDraw.h"
#include "ofxHairModel.h"
#include "ofxHairSim.h"
#include "ofxMeshUtil.h"
#include "ofxHairDraw.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void reset();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		ofxUISuperCanvas *gui1, *gui2, *gui3, *gui4;
		void setGUI1();
		void guiEvent(ofxUIEventArgs &e);
		string originalFileExtension;

		ofxHairModel hairModel, initModel;
		ofxHairModel simModel;
		ofxHairSim sim;
	
		// adaptive guide hair
		vector<int> adaptive_guide_indices;

		ofxHEMesh hem, initHem;
		ofEasyCam cam;

		ofxHairDraw *viewer;

		// load graph
		void hairFitting();

		ofLight light;
};
//