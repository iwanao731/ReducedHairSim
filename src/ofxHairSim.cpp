#include "ofxHairSim.h"

ofxHairSim::ofxHairSim()
{
	m_gravity.set(0.0, -9.8f, 0.0);
	m_damping = 0.99;
	m_timestep = 0.05;
	m_collisionType = ClosestPointOnTriangleKdTree;
	bFloorContact = false;
	m_floor_Y = -1000;
}

ofxHairSim::~ofxHairSim()
{
	//m_particles.clear();
	//m_particlesPos.clear();
	//delete hair;
}

void ofxHairSim::init(ofxHairModel &model)
{
	hair = &model; // link

	m_particles.clear();
	int num = model.getNumParticles();
	m_particles.resize(num);
	m_particlesPos.resize(num);
	m_particlesPosition.resize(num);

	for(int i=0; i<model.getNumStrand(); i++){
		for(int j=0; j<model.strands[i].getResolution(); j++){
			int k = i*model.strands[i].m_resolution + j;
			m_particles[k] = &model.strands[i].m_particles[j];
			m_particlesPos[k] = &m_particles[k]->tmp_position;
			m_particlesPosition[k] = m_particles[k]->tmp_position;
		}
	}
	initSkeleton();
}

void ofxHairSim::initGuide(ofxHairModel &model)
{
	hair = &model; // link

	m_particles.clear();
	int num = hair->getNumParticles();
	m_particles.resize(num);
	m_particlesPos.resize(num);
	m_particlesPosition.resize(num);

	for (int i = 0; i<hair->getNumStrand(); i++) {
		for (int j = 0; j<hair->strands[i].getResolution(); j++) {
			int k = i*hair->strands[i].m_resolution + j;
			m_particles[k] = &hair->strands[i].m_particles[j];
			m_particlesPos[k] = &m_particles[k]->tmp_position;
			m_particlesPosition[k] = m_particles[k]->tmp_position;
		}
	}
}

bool ofxHairSim::loadBoundaryOBJ(string filename)
{
	return boundary.load(filename);
}

void ofxHairSim::updateBoundary(vector<ofPoint> &points)
{
	boundary.setPosition(points);

#if 0
	for(int i=0; i<hair->getNumStrand(); i++){
		hair->strands[i].m_particles[0].position = points[i];
	}
#else
	// changing root position
	for(int i=0; i<hair->getNumStrand(); i++){
		hair->strands[i].m_particles[0].position = 
			  points[hair->strands[i].root_ratio.idx[0]] * hair->strands[i].root_ratio.s
			+ points[hair->strands[i].root_ratio.idx[1]] * hair->strands[i].root_ratio.t
			+ points[hair->strands[i].root_ratio.idx[2]] * hair->strands[i].root_ratio.u;
	}
#endif
}

void ofxHairSim::modelHairFitting(ofxHairModel& _hair, float radius)
{
	hair = &_hair;

	ofxNearestNeighbour3D nn;
	nn.buildIndex(boundary.m_points);

	for(auto& s : _hair.strands)
	{	
		ofPoint root_pos = s.m_particles[0].position;
	    vector<pair<NNIndex, float> > indices;
	    nn.findPointsWithinRadius(root_pos, radius, indices);	// the result is depended by radius parameter

		// calculate closest point
		float closest_dist = 1000000;
		Triangle closest_triangle;
		ofPoint closest_point;
		float weight[3];
		for(auto idx : indices){
			for(int i = 0; i < boundary.m_numNeighborTriangle[idx.first]; i++){
				Triangle t = boundary.m_neighborTriangle[idx.first][i];
				ofPoint collision_point;
				float w[3];
				float dist = ofxMeshUtil::point_triangle_distance2(root_pos, boundary.m_points[t.p1], boundary.m_points[t.p2], boundary.m_points[t.p3], collision_point, w[0], w[1], w[2]);
				if(dist < closest_dist){
					closest_dist = dist;
					closest_point = collision_point;
					closest_triangle = t;
					weight[0] = w[0];
					weight[1] = w[1];
					weight[2] = w[2];
				}
			}
		}

		// set
		s.root_ratio.s = weight[0];
		s.root_ratio.t = weight[1];
		s.root_ratio.u = weight[2];
		s.root_ratio.idx[0] = closest_triangle.p1;
		s.root_ratio.idx[1] = closest_triangle.p2;
		s.root_ratio.idx[2] = closest_triangle.p3;

		// translate
		ofVec3f trans(closest_point - root_pos);
		for(auto& p : s.m_particles)
		{
			p.position += trans;
			p.position0 = p.position;
		}
	}
}

void ofxHairSim::reset()
{
}

void ofxHairSim::update(ofMatrix4x4 mRotate)
{
	updateForce();
	updateVelocity();
	solveConstraint(mRotate);
	solveCollision();
	updatePositionAndVelocity();
}

void ofxHairSim::updateForce()
{
	for(auto& p : m_particles) {
		p->forces += m_gravity;
		//p->forces += ofVec3f(0.0, 0.0, ofNoise(ofGetElapsedTimef() * 0.75, 100.0) * 50); /* wind */
	}
}

void ofxHairSim::updateVelocity()
{
	float dt = m_timestep;

	for(auto p : m_particles) {

		if(!p->enabled) {
			p->tmp_position = p->position;
			continue;
		}

		p->velocity = p->velocity + dt * (p->forces * p->inv_mass);
		p->tmp_position += (p->velocity * dt);
		p->forces = ofVec3f(0.0);
		p->velocity *= 0.99;
	}
}

void ofxHairSim::solveCollision()
{
	switch (m_collisionType)
	{
		case ClosestPointOnTriangle:
			ofxHairCollision::collisionClosestPointOnTriangle(m_particles, boundary.m_points, boundary.m_normal, boundary.m_triangles);
			break;
		case ClosestPointOnTriangleFromVertex:
			ofxHairCollision::collisionClosestPointOnTriangleFromNearestVertex(m_particles, boundary.m_points, boundary.m_normal, boundary.m_numNeighborTriangle, boundary.m_neighborTriangle);
			break;
		case ClosestPointOnTriangleKdTree:
			ofxHairCollision::collisionClosestPointOnTriangle_ke_tree(m_particles, boundary.m_points, boundary.m_normal, boundary.m_numNeighborTriangle, boundary.m_neighborTriangle, m_collision.getRadius() );
			break;
		default:
			break;
	}
}

void ofxHairSim::solveConstraint(ofMatrix4x4 mRotate)
{
	//applyConstrainHARADA12(mRotate);

	float Sg = 0.2;
	float Sl = 0.1;
	applyConstrainCEIG15(Sg, Sl, mRotate);
}

void ofxHairSim::applyLocalShapeConstrain(ofMatrix4x4 headRot)
{
	const int numIteration =  4;
	float SlRoot = 1.0;
	float SlTip = 0.6;
	
	for(int sidx=0; sidx<hair->strands.size(); sidx++)
	{
		for(int iter = 0; iter < numIteration; iter++)
		{
			ofQuaternion rotGlobal = hair->strands[sidx].m_particles[0].local_rotation0;

			for(int i=0; i<hair->strands[sidx].getResolution()-1; i++)
			{
				ofVec3f pos = hair->strands[sidx].m_particles[i].tmp_position;
				float t = i / (float)hair->strands[sidx].getResolution();
				float Sl = (1.0f - t) * SlRoot + t * SlTip; // Linear interpolation
				ofVec3f pos_plus_one = hair->strands[sidx].m_particles[i+1].tmp_position;
				ofQuaternion rotGlobalWorld = headRot.getRotate() * rotGlobal;
				ofVec3f pos0_i_plus_1_local = hair->strands[sidx].m_particles[i+1].local_trans0;
				ofVec3f pos0_i_plus_1_world = rotGlobal * pos0_i_plus_1_local + pos;
				ofVec3f delta = 0.5f * Sl * (pos0_i_plus_1_world - pos_plus_one);

				if (hair->strands[sidx].m_particles[i].mass > 0.0f) {
					pos -= delta;
					hair->strands[sidx].m_particles[i].d += delta;
				}

				if (hair->strands[sidx].m_particles[i+1].mass > 0.0f) {
					pos_plus_one += delta;
					hair->strands[sidx].m_particles[i+1].d += delta;
				}

				// Update local / global frame
				ofQuaternion invRotGlobalWorld = ofMatrix4x4(rotGlobalWorld).getInverse().getRotate();
				ofVec3f vec = (pos_plus_one - pos).normalized();
				ofVec3f x_i_plus_1_frame_i = (invRotGlobalWorld * vec).normalized();
				ofVec3f e = ofVec3f(1.0f, 0.0f, 0.0f);
				ofVec3f rotAxis = e.crossed(x_i_plus_1_frame_i);

				if (rotAxis.length() > 0.001f) {
					float angleInRadian = acos(e.dot(x_i_plus_1_frame_i));
					rotAxis = rotAxis.normalized();
					ofQuaternion localRot = ofQuaternion(angleInRadian, rotAxis);
					rotGlobal = rotGlobal * localRot;
				}

				hair->strands[sidx].m_particles[i].tmp_position = pos;
				hair->strands[sidx].m_particles[i+1].tmp_position = pos_plus_one;
				pos = pos_plus_one;
			}
		}
	}
}


void ofxHairSim::applyLocalShapeConstrain2(ofMatrix4x4 headRot)
{
	const int numIteration = 1;
	float SlRoot = 1.0f;
	float SlTip = 0.4f;
	
	for(int iter = 0; iter < numIteration; iter++)
	{
		for(int sidx=0; sidx<hair->strands.size(); sidx++)
		{
			ofQuaternion rotGlobal = hair->strands[sidx].m_particles[0].local_rotation0;
			ofVec3f pos = hair->strands[sidx].m_particles[1].tmp_position;

			for(int i=1; i<hair->strands[sidx].getResolution()-1; i++)
			{
				float t = i / (float)hair->strands[sidx].getResolution();
				float Sl = (1.0f - t) * SlRoot + t * SlTip; // Linear interpolation

				ofVec3f delta = hair->strands[sidx].m_particles[i].position0 - hair->strands[sidx].m_particles[i].position;
				hair->strands[sidx].m_particles[i].tmp_position -= 0.5f * Sl * delta;
				hair->strands[sidx].m_particles[i+1].tmp_position += 0.5 * Sl * delta;
			}
		}
	}
}

void ofxHairSim::applyConstrainHARADA12(ofMatrix4x4 mRotate)
{
	// Han and Harada 2012
	for(int i=0; i<hair->getNumStrand(); i++) {
		for(int j=0; j<hair->strands[i].getResolution(); j++) {
			if(j==0){
				ofxHairParticle* pc = &hair->strands[i].m_particles[0];
				ofVec3f delta_x_i_global = mRotate * pc->position0 - pc->tmp_position;
				pc->tmp_position += delta_x_i_global;
			}else{
				float t = j / (float)hair->strands[i].getResolution();
				ofxHairParticle* pc = &hair->strands[i].m_particles[j];

				// global shape constraint
				float SgRoot = 0.3;	
				float SgTip = 0.1;
				float Sg = (1.0f - t) * SgRoot + t * SgTip; // Linear interpolation
				ofVec3f delta_x_i_global = Sg * (mRotate * pc->position0 - pc->tmp_position);
				pc->tmp_position += delta_x_i_global;
			}
		}
	}

	//applyLocalShapeConstrain2(mRotate);


	for(int i=0; i<hair->getNumStrand(); i++) {
		for(int j=1; j<hair->strands[i].getResolution(); j++) {
			// distance constraint
			ofVec3f dir;
			ofVec3f curr_pos;
			ofxHairParticle* pa = &hair->strands[i].m_particles[j-1];
			ofxHairParticle* pb = &hair->strands[i].m_particles[j];

			curr_pos = pb->tmp_position;
			dir = pb->tmp_position - pa->tmp_position;
			dir.normalize();

			pb->tmp_position = pa->tmp_position + dir * hair->strands[i].getLength(j);
						
			// floor contact
			if(bFloorContact){
				if(pb->tmp_position.y < m_floor_Y)
					pb->tmp_position.y = m_floor_Y;
			}

			pb->d = curr_pos - pb->tmp_position;
		}
	}
}

void ofxHairSim::updatePositionAndVelocity()
{
	float dt = m_timestep;

	for(int i=0; i<hair->getNumStrand(); i++) {
		for(int j=1; j<hair->strands[i].getResolution(); j++) {
			ofxHairParticle* pa = &hair->strands[i].m_particles[j-1];
			ofxHairParticle* pb = &hair->strands[i].m_particles[j];
			if(!pa->enabled) {
				continue;
			}
			
			/* Han and Harada */
			//pa->velocity = ((pa->tmp_position - pa->position) / dt) + m_damping *  (pb->d / dt);

			/* GEIG2015 */
			pa->velocity = ((pa->tmp_position - pa->position) / dt) - m_damping * (pb->dGlobal + pb->dLocal + pb->dInex) / dt;
			pa->posPrev = pa->position;
			pa->position = pa->tmp_position;
		}
		if(hair->strands[i].m_particles.back().enabled) {
			ofxHairParticle* last = &hair->strands[i].m_particles.back();
			last->position = last->tmp_position;
		}
	}
}

void ofxHairSim::applyConstrainCEIG15(float Sg, float Sl, ofMatrix4x4 mRotate)
{
	float SgRoot = 1.0;	
	float SgTip = 0.3;

	/* paralel computing using C++11 */

	std::vector<std::thread> workers;
	std::atomic<int> i(0);

	int numThreads = std::thread::hardware_concurrency();
	numThreads = 1; // (numThreads < 1) ? 1 : numThreads;

	for (auto t = 0; t < numThreads; t++) {
		workers.push_back(std::thread([&, t]() {
			int index = 0;

			/* loop number of strands */
			while ((index = i++) < hair->strands.size()) {
				int sidx = i - 1;
				//cout << sidx << endl;
				/* Real-time Inextensible Hair with Volume and Shape */
				for (int pidx = 1; pidx<hair->strands[sidx].getResolution(); pidx++)
				{
					float t = pidx / (float)hair->strands[sidx].getResolution();
					ofxHairParticle* pc = &hair->strands[sidx].m_particles[pidx];
					float _Sg = (1.0f - t) * SgRoot + t * SgTip; // Linear interpolation

					 /* global */
					ofVec3f dGlobal = Sg * (mRotate * hair->strands[sidx].m_particles[pidx].position0 - hair->strands[sidx].m_particles[pidx].tmp_position);
					hair->strands[sidx].m_particles[pidx].tmp_position += _Sg * dGlobal;

					/* local */
					ofVec3f dLocal = Sl * (hair->strands[sidx].m_particles[pidx].posPrev - hair->strands[sidx].m_particles[pidx].tmp_position);
					hair->strands[sidx].m_particles[pidx].tmp_position += _Sg * dLocal;

					/* inextensibility */
					ofxHairParticle* pa = &hair->strands[sidx].m_particles[pidx - 1];
					ofxHairParticle* pb = &hair->strands[sidx].m_particles[pidx];
					float restDist = hair->strands[sidx].getLength(pidx);
					ofVec3f dInex = (1.0f - restDist / (pa->tmp_position - pb->tmp_position).length()) * (pa->tmp_position - pb->tmp_position);
					hair->strands[sidx].m_particles[pidx].tmp_position += dInex;

					pb->dGlobal = _Sg * dGlobal;
					pb->dLocal = _Sg * dLocal;
					pb->dInex = dInex;
				}
			}
		}));
	}

	for (auto &t : workers) {
		t.join();
	}
}

void ofxHairSim::calcSkinWeight(int linkNum, ofxHairModel &model)
{
	ofxNearestNeighbour3D nn;
	//map<int, int> mapIdx; // number of guide index -> all index
	
	// construct the search information
	vector<ofVec3f> points;
	int allNum = 0;
	int guideNum = 0;
	for (auto &s : model.strands) {
		for (auto &p : s.m_particles) {
			if (s.bGuideHair) {
				points.push_back(p.position);
				mapIdx.insert(map<int, int>::value_type(guideNum, allNum));
				guideNum++;
			}
			allNum++;
		}
	}

	nn.buildIndex(points);

	// search to find closest point
	int particleIdx = 0;
	for (auto &s : model.strands) {
		for (auto &p : s.m_particles) {
			if (!s.bGuideHair) {
				vector<NNIndex> indices2;
				vector<float> distance;
				nn.findNClosestPoints(p.position, linkNum, indices2, distance);

				weightInfo weight;
				weight.index = particleIdx;
				for (unsigned i = 0; i < indices2.size(); ++i)
				{
					if (model.strands[mapIdx[indices2[i]] / s.getResolution()].bGuideHair) {
						weight.guideIndices.push_back(mapIdx[indices2[i]]);
						float d = distance[i];
						weight.weight.push_back(pow(d + 1, -8));
					}
				}

				// normalize
				float sum = 0.0f;
				for (auto &w : weight.weight) {
					sum += w;
				}
				for (auto &w : weight.weight) {
					w /= sum;
				}
				m_weightInfo.push_back(weight);
			}
			particleIdx++;
		}
	}

	for (int i = 0; i < m_weightInfo.size(); i++) {
		for (int j = 0; j < m_weightInfo[i].guideIndices.size(); j++) {
			model.mNormalVertices[i].mArBlendingIndex[j] = m_weightInfo[i].guideIndices[j];
			model.mNormalVertices[i].mArBlendingWeight[j] = m_weightInfo[i].weight[j];
		}
	}
	//initSkeleton();
}

void ofxHairSim::calcSkinWeight2(int linkNum, ofxHairModel &model)
{
	cout << "calc skinning weight" << endl;

	ofxNearestNeighbour3D nn;

	int allNum = 0;
	int guideNum = 0;
	for (auto &s : model.strands) {
		for (auto &p : s.m_particles) {
			if (!s.bGuideHair) {
				mapIdx.insert(map<int, int>::value_type(allNum, guideNum));
				guideNum++;
			}
			allNum++;
		}
	}

	// construct the search information
	vector<ofVec3f> points;
	for (auto &p : model.mGuideJoints) {
		points.push_back(p.mPos);
	}

	nn.buildIndex(points);

	// search to find closest point
	int particleIdx = 0;
	for (auto &p : model.mNormalVertices) {
		vector<NNIndex> indices2;
		vector<float> distance;
		nn.findNClosestPoints(p.mPosition, linkNum, indices2, distance);

		weightInfo weight;
		weight.index = particleIdx;
		for (unsigned i = 0; i < indices2.size(); ++i)
		{
			weight.guideIndices.push_back(indices2[i]);
			float d = distance[i];
			weight.weight.push_back(pow(d + 1, -8));
		}

		// normalize
		float sum = 0.0f;
		for (auto &w : weight.weight) {
			sum += w;
		}
		for (auto &w : weight.weight) {
			w /= sum;
		}
		m_weightInfo.push_back(weight);
		particleIdx++;
	}

	for (int i = 0; i < m_weightInfo.size(); i++) {
		for (int j = 0; j < m_weightInfo[i].guideIndices.size(); j++) {
			model.mNormalVertices[i].mArBlendingIndex[j] = m_weightInfo[i].guideIndices[j];
			model.mNormalVertices[i].mArBlendingWeight[j] = m_weightInfo[i].weight[j];
		}
	}
	cout << "finished calc skinning weight" << endl;
}

bool ofxHairSim::loadSkinWeight(const string filename)
{
	ifstream ofs(filename);
	if (ofs.fail()) {
		std::cerr << "failed loading weight" << std::endl;
		return false;
	}

	std::string str;

	// header
	getline(ofs, str);
	m_weightInfo.resize(ofToInt(str));

	// others
	int i = 0;
	while (getline(ofs, str)) {
		vector<string> indices = ofxHairUtil::split(str, ' ');
		int k = 0;
		for (int j = 0; j < indices.size(); j+=2) {
			if (j == 0) {
				m_weightInfo[i].index = ofToInt(indices[0]);
				m_weightInfo[i].guideIndices.resize(ofToInt(indices[1]));
				m_weightInfo[i].weight.resize(ofToInt(indices[1]));
			}else {
				m_weightInfo[i].guideIndices[k] = ofToInt(indices[j]);
				m_weightInfo[i].weight[k] = ofToFloat(indices[j+1]);
				k++;
			}
		}
		i++;
	}

	initSkeleton();

	return true;
}

ofMatrix4x4 CreateTransformFromQuaternionTransform(ofQuaternion quaternion, ofVec4f translation)
{
	ofQuaternion q = quaternion;
	float ww = q.w() * q.w() - 0.5f;

	ofVec3f v00 = ofVec3f(ww, q.x() * q.y(), q.x() * q.z());
	ofVec3f v01 = ofVec3f(q.x() * q.x(), q.w() * q.z(), -q.w() * q.y());
	ofVec3f v10 = ofVec3f(q.x() * q.y(), ww, q.y() * q.z());
	ofVec3f v11 = ofVec3f(-q.w() * q.z(), q.y() * q.y(), q.w() * q.x());
	ofVec3f v20 = ofVec3f(q.x() * q.z(), q.y() * q.z(), ww);
	ofVec3f v21 = ofVec3f(q.w() * q.y(), -q.w() * q.x(), q.z() * q.z());

	ofVec3f vv0 = 2.0f * (v00 + v01);
	ofVec3f vv1 = 2.0f * (v10 + v11);
	ofVec3f vv2 = 2.0f * (v20 + v21);

	return ofMatrix4x4(
		vv0.x, vv0.y, vv0.z, 0,
		vv1.x, vv1.y, vv1.z, 0,
		vv2.x, vv2.y, vv2.z, 0,
		translation.x, translation.y, translation.z, 1
	);
}


ofMatrix4x4 CreateTransformFromQuaternionTransform2(ofQuaternion quaternion, ofVec4f translation)
{
	ofQuaternion q = quaternion;

	float val[16];
	val[0]  = 1.0f - 2.0f*(q.y() * q.y() + q.z() * q.z());
	val[1]  = 2.0f * (q.x() * q.y() + q.z() * q.w());
	val[2]  = 2.0f * (q.z() * q.x() - q.y() * q.w());
	val[3]  = 0.0f;
	val[4]  = 2.0f * (q.x() * q.y() - q.z() * q.w());
	val[5]  = 1.0f - 2.0f*(q.z() * q.z() + q.x() * q.x());
	val[6]  = 2.0f * (q.y() * q.z() + q.x() * q.w());
	val[7]  = 0.0f;
	val[8]  = 2.0f * (q.z() * q.x() + q.y() * q.w());
	val[9]  = 2.0f * (q.y() * q.z() - q.x() * q.w());
	val[10] = 1.0f - 2.0f*(q.x() * q.x() + q.y() * q.y());
	val[11] = 0.0f;
	val[12] = translation.x;
	val[13] = translation.y;
	val[14] = translation.z;
	val[15] = 1.0f;

	return ofMatrix4x4(val);
}

ofMatrix4x4 CreateTransformFromQuaternionTransform3(ofQuaternion quaternion, ofVec4f translation)
{
	ofQuaternion q = quaternion;

	float val[16];
	val[0] = q.x() * q.x() + q.y() * q.y() - q.z() * q.z() - q.w() * q.w();
	val[1] = 2.0f * (q.y() * q.z() - q.x() * q.w());
	val[2] = 2.0f * (q.x() * q.z() + q.y() * q.w());
	val[3] = 0.0f;
	val[4] = 2.0f * (q.x() * q.w() + q.y() * q.z());
	val[5] = q.x() * q.x() - q.y() * q.y() + q.z() * q.z() - q.w() * q.w();
	val[6] = 2.0f * ( -q.x() * q.y() + q.z() * q.w());
	val[7] = 0.0f;
	val[8] = 2.0f * (q.y() * q.w() - q.x() * q.z());
	val[9] = 2.0f * (q.z() * q.w() + q.x() * q.y());
	val[10] = q.x() * q.x() - q.y() * q.y() - q.z() * q.z() + q.w() * q.w();
	val[11] = 0.0f;
	val[12] = translation.x;
	val[13] = translation.y;
	val[14] = translation.z;
	val[15] = 1.0f;

	return ofMatrix4x4(val);
}

void ofxHairSim::deformation(ofxHairModel &model, ofxHairModel &guide)
{
	int allIdx = 0;
	int guideIdx = 0;
	for (auto &s : model.strands) {
		for (auto &p : s.m_particles) {
			ofVec3f pos;
			if (!s.bGuideHair) {
				int m_size = m_weightInfo[guideIdx].guideIndices.size();
				for (int j = 0; j < m_size; j++) {
					int idx = m_weightInfo[guideIdx].guideIndices[j];
					int sIdx = idx / 100;
					int pIdx = idx % 100;
					ofxHairParticle pp = guide.strands[sIdx].m_particles[pIdx];

					// for test
					ofMatrix4x4 mm = pp.gMat0.getInverse() * pp.gMat;
					ofQuaternion qq = mm.getRotate();
					ofVec3f tt = mm.getTranslation();

					ofMatrix4x4 mmm;
					mmm.setRotate(qq);
					mmm.setTranslation(tt);
					pos += p.position0 * mmm * m_weightInfo[guideIdx].weight[j];
				}
				guideIdx++;
			}
			allIdx++;
			p.position = pos;
		}
	}
}

void ofxHairSim::initSkeleton()
{
	ofVec3f v0(0.0, 1.0, 0.0);
	for (auto &s : hair->strands) {
		int count = 0;
		for (int i = 0; i<s.m_particles.size()-1; i++) {
			ofVec3f v1 = s.m_particles[i + 1].position - s.m_particles[i].position;

			ofMatrix4x4 mat;
			mat.makeIdentityMatrix();

			// rotate
			ofQuaternion q;
			q.makeRotate(v0, v1);

			mat.glTranslate(s.m_particles[i].position);
			mat.glRotate(q);
			s.m_particles[i].gMat0 = mat;
			s.m_particles[i].gMat = mat;
			count++;
		}
		s.m_particles[count].gMat0 = s.m_particles[count-1].gMat0;
		s.m_particles[count].gMat = s.m_particles[count-1].gMat;
	}
}

void ofxHairSim::updateSkeleton()
{
	ofVec3f v0(0.0, 1.0, 0.0);

	int countAll = 0;
	for (auto &s : hair->strands) {
		int count = 0;
		for (int i = 0; i < s.m_particles.size()-1; i++) {
			ofVec3f v1 = s.m_particles[i + 1].position - s.m_particles[i].position;

			ofMatrix4x4 mat;
			mat.makeIdentityMatrix();

			// rotate
			ofQuaternion q;
			q.makeRotate(v0, v1);

			mat.glTranslate(s.m_particles[i].position);
			mat.glRotate(q);
			s.m_particles[i].gMat = mat;
			//s.m_particles[i].gMat.makeRotationMatrix(30, 0, 1, 0);

			count++;
		}
		countAll++;
		s.m_particles[count].gMat = s.m_particles[count-1].gMat;
		//s.m_particles[count].gMat.makeRotationMatrix(30, 0, 1, 0);
	}
}
