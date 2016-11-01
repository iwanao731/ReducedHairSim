#include "ofxHairDraw.h"
#define BUFFER_OFFSET(bytes) ((GLubyte *)NULL + (bytes))

ofxHairDraw::ofxHairDraw(ofxHairModel &model)
:	m_model(model)
{
}

void ofxHairDraw::init(string filename)
{
	glClear(GL_COLOR_BUFFER_BIT);

	if (!shader.load(filename)) {
		cout << "failed loading shader" << endl;
	};

	// generate OpenGL Vertex-Buffer-Object & Index-Buffer-Object
	if (mVbo)  glDeleteBuffers(1, &mVbo);
	glGenBuffers(1, &mVbo);
	glBindBuffer(GL_ARRAY_BUFFER, mVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(VertexIntermediate) * m_model.mNormalVertices.size(), &m_model.mNormalVertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	/* Texture */
	/* I refered from this website to implement "Vertex Texture Fetch"
		OpenGLでGPUスキニング【頂点テクスチャ編】
		http://nullorempry.jimdo.com/2012/07/02/opengl%E3%81%A7gpu%E3%82%B9%E3%82%AD%E3%83%8B%E3%83%B3%E3%82%B0-%E9%A0%82%E7%82%B9%E3%83%86%E3%82%AF%E3%82%B9%E3%83%81%E3%83%A3%E7%B7%A8/
	*/

	#define TEXTURE_SIZE 2
	boneNum = m_model.mGuideJoints.size();

	cout << "bone Num : " << boneNum << endl;

	unsigned int ids[TEXTURE_SIZE] = { 0, 0 }; // テクスチャ領域
	ofVec4f *init_data = new ofVec4f [boneNum]; // ボーンの初期データ配列
	translations = new ofVec4f[boneNum];
	rotations = new ofQuaternion[boneNum];
	
	// 座標の初期化
	for (int i = 0; i < boneNum; i++)
		init_data[i].set(ofVec4f(0.0f, 0.0f, 0.0f, 1.0f));

	for (int i = 0; i < TEXTURE_SIZE; i++)
	{
		if (ids[i])  glDeleteBuffers(1, &ids[i]);

		if (i == 0)
			glActiveTexture(GL_TEXTURE0);
		else if (i == 1)
			glActiveTexture(GL_TEXTURE1);

		glGenTextures(1, &ids[i]);
		glBindTexture(GL_TEXTURE_2D, ids[i]);

		/* フィルタは補間をかけない && ミップマップもかけない */
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (int)GL_NEAREST);

		/* ラッピングはClampモード(0~1以外は端っこを引き延ばす) */
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, (int)GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, (int)GL_CLAMP_TO_EDGE);

		/* ミップマップをつくらない */
		glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);

		/* データ形式の設定 */
		glTexImage2D(
			GL_TEXTURE_2D,	// テクスチャターゲット
			0,				// ミップマップレベル
			GL_RGBA32F,		// テクスチャのピクセルフォーマット
			boneNum, 1,		// テクスチャの縦横幅(データの要素数)
			0,				// ボーダー
			GL_RGBA,		// ピクセルの配列形式
			GL_FLOAT,		// 1ピクセルのデータ形式
			init_data);		// ピクセルデータは空
							
		/* Uniform番号の取得 */
		if (i == 0) {
			locate_pos = glGetUniformLocation(shader.getProgram(), "BoneTranslationTexture");	// ボーン移動テクスチャのUniform変数の番号
			if (locate_pos >= 0)
				glUniform1i(locate_pos, 0);
			else
				fprintf(stderr, "ユニフォーム変数 BoneTranslationTexture が見つかりません\n");
		}
		else if (i == 1) {
			locate_rot = glGetUniformLocation(shader.getProgram(), "BoneRotationTexture");		// ボーン回転テクスチャのUniform変数の番号
			if (locate_rot >= 0)
				glUniform1i(locate_rot, 1);
			else
				fprintf(stderr, "ユニフォーム変数 BoneRotationTexture が見つかりません\n");
		}
	}

	posID = ids[0]; // ボーン平行移動値のテクスチャ
	rotID = ids[1]; // ボーン回転値のテクスチャ
	cout << "Texture Trans location : " << locate_pos << " , " << posID << endl;
	cout << "Texture Rotate location : " << locate_rot << " , " << rotID << endl;

	delete init_data;
}

void ofxHairDraw::update(ofxHairModel &model, ofEasyCam &cam)
{
}

void ofxHairDraw::draw(ofxHairModel &model, ofEasyCam &cam)
{
	//glClearColor(0.0, 0.0, 0.0, 1.0);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* layout update */
	const int iPositionSlot = 0;
	const int iColorSlot = 1;
	const int iBoneIndicesSlot = 2;
	const int iBoneWeightsSlot = 3;

	/* uniform変数をセットアップする前に呼ばれる必要がある */
	glUseProgram(shader.getProgram());

	glEnableVertexAttribArray(iPositionSlot);
	glEnableVertexAttribArray(iColorSlot);
	glEnableVertexAttribArray(iBoneIndicesSlot);
	glEnableVertexAttribArray(iBoneWeightsSlot);

	// bind vertex buffer
	glBindBuffer(GL_ARRAY_BUFFER, mVbo);

	glVertexAttribPointer(iPositionSlot, 3, GL_FLOAT, GL_FALSE, sizeof(VertexIntermediate), 0);
	glVertexAttribPointer(iColorSlot, 3, GL_FLOAT, GL_FALSE, sizeof(VertexIntermediate), (void*)(sizeof(ofVec3f)));
	glVertexAttribPointer(iBoneIndicesSlot, 4, GL_FLOAT, GL_FALSE, sizeof(VertexIntermediate), (void*)(6 * sizeof(GLfloat)));
	glVertexAttribPointer(iBoneWeightsSlot, 4, GL_FLOAT, GL_FALSE, sizeof(VertexIntermediate), (void*)(10 * sizeof(GLfloat)));
	shader.setUniformMatrix4f("ModelViewProjectionMatrix", cam.getModelViewProjectionMatrix());

	/* For Rendering */
	if (this->hasSkin()) {
		shader.begin();
		this->getSkinFbo().begin();
		this->getSkinFbo().activateAllDrawBuffers();
	}

	/* 移動情報の更新 */
	int i = 0;
	for (auto &s : model.strands) {
		for (auto &p : s.m_particles) {
			ofMatrix4x4 m = p.gMat0.getInverse() * p.gMat;
			translations[i] = m.getTranslation();
			rotations[i] = m.getRotate();
			i++;
		}
	}

	// テクスチャユニットの切り替え（平行移動テクスチャはTextureUnit1に登録）
	glActiveTexture(GL_TEXTURE1);
	if (locate_pos != -1) {
		glUniform1i(locate_pos, GL_TEXTURE1 - GL_TEXTURE0);

		// 現在のユニットテクスチャを有効化する
		glEnable(GL_TEXTURE_2D);

		/* ボーン平行移動テクスチャの更新 */
		glBindTexture(GL_TEXTURE_2D, posID);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, boneNum, 1, 0, GL_RGBA, GL_FLOAT, translations);

		// 現在のユニットテクスチャを無効化する
		glDisable(GL_TEXTURE_2D);
	}

	// テクスチャユニットの切り替え (回転テクスチャはTextureUnit2に登録)
	glActiveTexture(GL_TEXTURE2);
	if (locate_rot != -1) {
		glUniform1i(locate_rot, GL_TEXTURE2 - GL_TEXTURE0);

		/* 現在のユニットテクスチャを有効化する */
		glEnable(GL_TEXTURE_2D);

		/* ボーン回転テクスチャの更新 */
		glBindTexture(GL_TEXTURE_2D, rotID);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, boneNum, 1, 0, GL_RGBA, GL_FLOAT, rotations);

		// 現在のユニットテクスチャを無効化する
		glDisable(GL_TEXTURE_2D);
	}

	/* ボーンテクスチャのサイズを渡す */
	shader.setUniform2i("BoneTextureSize", boneNum, 1);

	/* drawing */
	int res = 100;
	glEnableClientState(GL_VERTEX_ARRAY); //有効化
	for (int i = 0; i < m_model.mNormalVertices.size() / res; i++) {
		glDrawArrays(GL_LINE_STRIP, i*res, res);
	}
	glDisableClientState(GL_VERTEX_ARRAY);  //無効化


	/* Hair Simulation */
	shader.end();

	/* clean */
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(iPositionSlot);
	glDisableVertexAttribArray(iColorSlot);
	glDisableVertexAttribArray(iBoneIndicesSlot);
	glDisableVertexAttribArray(iBoneWeightsSlot);

	glUseProgram(0);
}

void ofxHairDraw::drawOld(ofxHairModel &model)
{
	if (bEdge) {
		glLineWidth(0.1f);
		for (auto s : model.strands) {
			if (s.bGuideHair && bGuideHair) {
				glBegin(GL_LINE_STRIP);
				for (auto p : s.m_particles) {
					if (bColor) {
						ofSetColor(p.color);
					}
					else {
						ofSetColor(p.collision_color);
					}
					glVertex3f(p.position.x, p.position.y, p.position.z);
				}
				glEnd();
			}

			if (!s.bGuideHair && bNormalHair) {
				glBegin(GL_LINE_STRIP);
				for (auto p : s.m_particles) {
					if (bColor) {
						ofSetColor(p.color);
					}
					else {
						ofSetColor(p.collision_color);
					}
					glVertex3f(p.position.x, p.position.y, p.position.z);
				}
				glEnd();
			}
		}

		if (bNode) {
			ofSetColor(255);
			glPointSize(3.0);
			glBegin(GL_POINTS);
			for (auto s : model.strands) {
				for (auto p : s.m_particles) {
					ofPoint pos = p.position;
					ofSetColor(p.color);
					if (bColor) {
						ofSetColor(p.collision_color);
					}
					glVertex3f(pos.x, pos.y, pos.z);
				}
			}
			glEnd();
		}
	}
}

ofxHairDraw& ofxHairDraw::setDrawHairParticles(bool v)
{
	bNode = v;
	return *this;
}

ofxHairDraw& ofxHairDraw::setDrawHairEdges(bool v)
{
	bEdge = v;
	return *this;
}

ofxHairDraw& ofxHairDraw::setDrawHairColor(bool v)
{
	bColor = v;
	return *this;
}

ofxHairDraw& ofxHairDraw::setDrawHairNormal(bool v)
{
	bNormalHair = v;
	return *this;
}


ofxHairDraw& ofxHairDraw::setDrawHairGuide(bool v)
{
	bGuideHair = v;
	return *this;
}

std::vector<ofVec3f> ofxHairDraw::getSkinVertices() const {
	ofFloatPixels pixels;
	skinFbo->readToPixels(pixels, 0);

	std::vector<ofVec3f> data(boneNum);
	std::memcpy(&data[0], pixels.getData(), sizeof(ofVec3f) * boneNum);

	return std::move(data);
}
