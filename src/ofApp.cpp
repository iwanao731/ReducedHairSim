#include "ofApp.h"

// important factor
float searchRadius = 4.8; // 4.5
int resolution = 25;
int frameNum = 1000;
int cutGroupNum = 1000; // 800

// simulation parameter
float supportRadius = 7.0f;
float timestep = 0.05f;
float damping = 0.7;

// count
int frameCount = 0;

// bool
bool SkinningMethod = false;
bool bDrawMesh = true;
bool bRotation = true;
bool bPlay = false;
bool bHairColor = true;
bool bFitHeadAndHair = false;
bool bSelectGuideHair = false;
bool bDrawGuide = true;
bool bDrawSkinWeight = false;
bool bDrawNormal = true;
bool bSkinning = false;
bool bDeformation = true;
bool bGLSL = true;

//--------------------------------------------------------------
void ofApp::setup(){

	//string hairname = "E:/Database/USC-HairSalon/strands00010_.data";
	string hairname = "data/hair_example.data";
	string filename = "../../../../../apps/sharedData/hair/hairstyles/head_model.obj";

	hairModel.loadHairModel(hairname);
	//hairModel.exportHairModel("data/exported.data");
	hem.loadOBJModel(filename);
	initHem= hem;
	
	/* read guide hair */
	string guidename = "data/test_7000.guide";
	hairModel.loadGuideHair(guidename);

	/* simulation setting */
	ofxHairSim test;
	ofxHairModel m = hairModel;
	test.init(m);
	//test.loadSkinWeight("");
	test.calcSkinWeight2(4, hairModel);

	cout << "loading shader" << endl;
	viewer = new ofxHairDraw(hairModel);
	viewer->init("Shaders/HairSkinning");
	cout << "loading shader end" << endl;
	
	hairModel.exportGuideHairModel("data/guidehair.hair");
	simModel.loadHairModel("data/guidehair.hair");
	//simModel.exportHairModel("data/guidehair2.hair");
	float radius = 10.0f;
	
	sim.setTimeStep(timestep);
	sim.setDamping(damping);
	sim.setSupportRadius(supportRadius);

	sim.loadBoundaryOBJ(filename);
	sim.modelHairFitting(simModel, 10.0f);
	sim.setCollisionType(CollisionType::ClosestPointOnTriangleKdTree);
	sim.setSupportRadius(supportRadius);
	sim.init(simModel);

	setGUI1();
	sim.m_weightInfo = test.m_weightInfo;
}

//--------------------------------------------------------------
void ofApp::update(){

	sim.setTimeStep(timestep);
	sim.setDamping(damping);
	sim.setSupportRadius(supportRadius);

	if (bPlay) {
		ofMatrix4x4 mat; mat.isIdentity();

		if (bRotation) {
			cam.getGlobalTransformMatrix();
			if (bGLSL == false)
				mat.rotateRad(sin(ofGetElapsedTimef()*0.3f), 0, 1, 0);
			else
				mat.rotateRad(sin(ofGetElapsedTimef()*5.0f), 0, 1, 0);

			ofxHEMeshVertexIterator vit = hem.verticesBegin();
			ofxHEMeshVertexIterator vite = hem.verticesEnd();

			int i = 0;
			vector<ofPoint> points;
			for (; vit != vite; vit++, i++) {
				ofVec3f pos = mat * initHem.vertexPoint(vit.v);
				points.push_back(pos);
				hem.vertexMoveTo(vit.v, pos);
			}
			sim.updateBoundary(points);
		}
		sim.update(mat);
		sim.updateSkeleton();
		frameCount++;

		if (bGLSL == false)
			sim.deformation(hairModel, simModel);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(0);
	ofEnableDepthTest();
	ofEnableLighting();

	cam.begin();

	ofPushMatrix();
	//cam.setGlobalPosition(0, 500, 200);
	//ofTranslate(0, -500, 400);

	//light.setPosition(cam.getPosition());
	//light.enable();

	ofDisableLighting();
	if(bDrawMesh){
		ofSetColor(255);
		hem.setTopologyDirty(true);
		ofxHEMeshDraw hedraw(hem);
		hedraw.setDrawEdges(true);
		hedraw.setDrawFaces(true);
		hedraw.draw(cam);
	}
	
	if (bGLSL == false) {
		viewer->setDrawHairColor(bHairColor);
		viewer->setDrawHairParticles(false);
		viewer->setDrawHairEdges(true);
		viewer->setDrawHairNormal(bDrawNormal);
		viewer->setDrawHairGuide(bDrawGuide);
		viewer->drawOld(simModel);
		viewer->drawOld(hairModel);
	}
	else {
		viewer->draw(simModel, cam);
	}

	ofPopMatrix();
	cam.end();
	
	ofSetColor(255);
	ofDisableDepthTest();
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key)
	{
		case 'f':
			ofToggleFullscreen();
			break;
		case 'd':
			bDeformation = !bDeformation;
			hairModel.resetGuideAndNormal();
			break;
		case 'r':
		{
			//reset();
			break;
		}
		case 'g':
			gui1->toggleVisible();
			break;
		case 'b':
			bGLSL = !bGLSL;
			break;
		case ' ':
			bPlay = !bPlay;
			break;
	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::reset()
{
	hairModel = initModel;
	hem = initHem;
	vector<ofPoint> points;
	ofxHEMeshVertexIterator vit = hem.verticesBegin();
	ofxHEMeshVertexIterator vite = hem.verticesEnd();
	for( ; vit != vite; vit++ ){
		ofVec3f pos = hem.vertexPoint(vit.v);
		points.push_back(pos);
	}
	sim.updateBoundary(points);
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::setGUI1()
{
	gui1 = new ofxUISuperCanvas("1: HAIR SIM");
   	gui1->setWidgetFontSize(OFX_UI_FONT_MEDIUM);
	
    gui1->addSpacer();
	gui1->addSlider("supportRadius", 6.0f, 15.0f, supportRadius);
	gui1->addSlider("timestep", 0.0f, 1.4f, timestep);
	gui1->addSlider("damping", 0.0f, 1.0f, damping);

	gui1->addSpacer();
	gui1->addLabelToggle("Fit obj and hair", false);
	gui1->addSpacer();
	gui1->addLabelToggle("Simulation", false);

    gui1->addSpacer();
	gui1->addToggle("Rotation", bRotation);
	gui1->addToggle("Deformation", bDeformation);
	gui1->addToggle("Draw Guide Hair", bDrawGuide);
	gui1->addToggle("Draw Normal Hair", bDrawNormal);
	gui1->addToggle("Draw Hair Color", bHairColor);
	gui1->addToggle("Draw OBJ Model", bDrawMesh);

    gui1->setPosition(0, 0);
	gui1->autoSizeToFitWidgets();
	ofAddListener(gui1->newGUIEvent, this, &ofApp::guiEvent);
}

void ofApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.getName();
	int kind = e.getKind();
	cout << "got event from: " << name << endl;

	if(name == "Fit obj and hair")
	{
		hairFitting();

	}else if(name == "Simulation")
	{
		ofxUIToggle *toggle = (ofxUIToggle *) e.getToggle();
		bPlay = toggle->getValue();
	}else if(name == "Draw Normal Hair")
	{
		bDrawNormal = !bDrawNormal;
	}
	else if(name == "Draw Guide Hair")
	{
		bDrawGuide = !bDrawGuide;
	}else if(name == "Draw Hair Color")
	{
		bHairColor = !bHairColor;
	}else if(name == "Draw OBJ Model")
	{
		bDrawMesh = !bDrawMesh;
	}
	else if (name == "Deformation") {
		bDeformation = !bDeformation;
	}
	else if (name == "timestep")
	{
		ofxUISlider *slider = (ofxUISlider *)e.getSlider();
		timestep = slider->getValue();
		sim.setTimeStep(timestep);
	}else if (name == "damping")
	{
		ofxUISlider *slider = (ofxUISlider *)e.getSlider();
		damping = slider->getValue();
	}else if(name == "Rotation")
	{
		bRotation = !bRotation;
	}else if(name == "RADIUS"){
		ofxUISlider *slider = (ofxUISlider *) e.getSlider();
		searchRadius = slider->getValue();
	}
}

void ofApp::hairFitting()
{
	cout << "Fitting Hair and Model...";
	float radius = 10.0f;
	sim.modelHairFitting(hairModel, radius);
	//initModel = hairModel;
	bFitHeadAndHair = true;
	cout << "done..." << endl;
}
