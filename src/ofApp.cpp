#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(0);
    blackMagic.setup(1920, 1080, 30);
    background.allocate(CAM_WIDTH, CAM_HEIGHT);
    diff.allocate(CAM_WIDTH, CAM_HEIGHT);
    
    threashold = 60;
    initializeSourceArea();
    destination_corners = {cv::Point2f(0, 0), cv::Point2f(CAM_WIDTH, 0), cv::Point2f(CAM_WIDTH, CAM_HEIGHT), cv::Point2f(0, CAM_HEIGHT)};
    homography_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
    selectedCorner    = -1;
    autoFall          = false;
    learnBackground   = true;
    debug             = true;
    whiteScreen       = false;
    showGui           = false;
    scene             = Scene::Setup;

    for (int i = 0; i < 3; i++)
    {
        temari_images[i].load("temari"+ofToString(i)+".png");
        temari_fbos[i].allocate(temari_images[i].getWidth(), temari_images[i].getHeight());
        temari_fbos[i].begin();
        ofClear(0, 0, 0);
        temari_images[i].draw(0, 0);
        temari_fbos[i].end();
    }

    ofDisableArbTex();
    ofClear(0, 0, 0);
    sprite.load("image.jpg");
    ofEnableArbTex();
    
    box2d.init();
    box2d.enableEvents();
    box2d.setGravity(0, 3);
    box2d.registerGrabbing();
    box2d.setFPS(60);
    ofAddListener(box2d.contactStartEvents, this, &ofApp::contactStart);
    
    spkSystem.setup();
    spkGroup.setup(spkSystem);
    spkGroup.setColor(ofxSPK::Range<ofFloatColor>(ofColor(254, 240, 192, 255), ofColor(225, 211, 172, 255)),
                      ofxSPK::Range<ofFloatColor>(ofColor(246, 168, 24, 0), ofColor(250, 239, 74, 0)));
    spkGroup.setLifeTime(1, 2);
    spkGroup.setFriction(0.1);
    spkGroup.setSize(3, ofxSPK::RangeF(50, 80));
    spkGroup.setGravity(ofVec2f(0, 10.0));
    spkGroup.setMass(0.1, 1);
    spkGroup.reserve(10000);
    spkEmitter.setup(ofxSPK::Emitter::$Random::create(), spkGroup);
    spkEmitter.setForce(10, 50);
//    spkMod.setPosition(0, -300, 0);
    
    spkBackGroup.setup(spkSystem);
    spkBackGroup.setColor(ofxSPK::RangeC(ofColor(255, 255, 0, 255), ofColor(255, 0, 0, 255)),
                          ofxSPK::RangeC(ofColor(255, 0, 255, 0), ofColor(255, 255, 0, 0)));
    spkBackGroup.setLifeTime(10, 20);
    spkBackGroup.setFriction(0.01);
    spkBackGroup.setSize(ofxSPK::RangeF(0, 0.1), ofxSPK::RangeF(40, 130));
    spkBackGroup.setGravity(ofVec3f(0, -1, 4));
    spkBackGroup.setMass(0.1, 1);
    spkBackGroup.reserve(10000);
    
    spkBackEmitter.setup(ofxSPK::Emitter::$Random::create(), spkBackGroup);
    spkBackEmitter.setForce(10, 50);
    spkBackMod.setPosition(0, -300, 0);
    
    gui.setup();
    radius.set("R", 50, 10, 200);
    gui.add(radius);
}

//--------------------------------------------------------------
void ofApp::initializeSourceArea(){
    
    cv::Point2f center = cv::Point2f(CAM_WIDTH/2, CAM_HEIGHT/2);
    const int rectSize = 100;
    source_corners     = {center + cv::Point2f(-rectSize, -rectSize), center + cv::Point2f(rectSize, -rectSize), center + cv::Point2f(rectSize, rectSize), center + cv::Point2f(-rectSize, rectSize)};
        
}

//--------------------------------------------------------------
void ofApp::update(){

    
    if(autoFall)
    {
        if (ofRandom(1.0) < 0.02)
        {
            b2Temari circle;
            float r = radius;
            circles.push_back(shared_ptr<b2Temari>(new b2Temari));
            circles.back().get()->temari_image_id = (int)ofRandom(3.0 - FLT_EPSILON);
            circles.back().get()->angle = ofRandom(0, 360);
            circles.back().get()->setVelocity(ofRandom(-10, 10), 10);
            circles.back().get()->setPhysics(3.0, 1.0, 0.1);
            circles.back().get()->setup(box2d.getWorld(), ofRandom(0, ofGetWidth()), -2*r, r);
            
            b2Vec2 force = b2Vec2(ofRandom(-1, 1), 0);
            circles.back().get()->body->ApplyForce(force, b2Vec2(ofGetWidth()/2, ofGetHeight()/2), true);
        }
    }
    
    ofRemove(circles, sholdRemove);
    
    for (int i = 0; i < circles.size(); i++)
    {
        for (int j = 0; j < polylines.size(); j++)
        {
            if (polylines[j].inside(circles[i].get()->getPosition()))
            {
                circles[i].get()->destroy();
                circles.erase(circles.begin() + i);
            }
        }
    }
    
    box2d.update();
    
    for (int i = 0; i < 5; i++) {
        ofVec3f p = ofVec3f(ofRandom(-ofGetWidth(), ofGetWidth()), ofRandom(-ofGetWidth(), ofGetWidth()), ofRandom(0, ofGetWidth()));
        spkBackGroup.emitSpheric(10, p, ofxSPK::RangeF(10, 40), p.getCrossed(ofVec3f(1, 0, 0)), ofxSPK::RangeF(0, 0.2));
    }
    spkSystem.update();

    
    if (blackMagic.update())
    {
        original.setFromPixels(blackMagic.getGrayPixels());
        
        if (learnBackground) {
            background = original;
            learnBackground = false;
        }
        
        diff.absDiff(background, original);
        diff.threshold(threashold);
        contourFinder.findContours(diff, 1000, 1920*1080/4, 10, true);
        
        if (homography_matrix.at<float>(2, 2) != 0)
        {
            for (int i = 0; i < MAX_BLOBS; i++)
            {
                if (polygons[i].isBody())
                {
                    polylines[i].clear();
                    polygons[i].destroy();
                }
            }
            
            for (int i = 0; i < contourFinder.blobs.size(); i++)
            {
                for (int j = 0; j < contourFinder.blobs[i].pts.size(); j++)
                {
                    cv::Point3f point = ofxCv::toCv( contourFinder.blobs[i].pts[j] );
                    point.z = 1.0f;
                    cv::Mat source_mat = cv::Mat(point, CV_32FC1);
                    cv::Mat t = cv::Mat::zeros(3, 3, CV_32FC1);
                    t = homography_matrix * source_mat;
                    t /= t.at<float>(0, 2);
                    polylines[i].addVertex(t.at<float>(0, 0), t.at<float>(0, 1));
                }
                polylines[i].setClosed(false);
                polylines[i].simplify(3);
                
                polygons[i].addVertexes(polylines[i]);
                polygons[i].setPhysics(0.0, 0.5, 0.5);
                polygons[i].create(box2d.getWorld());

            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    if (scene == Scene::Setup)
    {
        if (!whiteScreen)
        {
            diff.draw(0, 0);
            contourFinder.draw(0, 0);
        } else {
            ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
        }
        
        ofPushStyle();
        ofSetLineWidth(3);
        ofNoFill();
        ofSetColor(ofColor::green);
        for (auto p : source_corners)
        {
            ofDrawCircle(p.x, p.y, 10);
        }
        ofPopStyle();
    }
    
    else if (scene == Scene::Play)
    {
        ofPushStyle();
        ofColor backColor;
        backColor.setHsb(ofMap(sin(ofGetElapsedTimef()/100), -1, 1, 19, 160), 10, 255);
        ofBackgroundGradient(ofColor::white, backColor, OF_GRADIENT_LINEAR);
//        ofBackgroundGradient(ofColor::white, ofColor(254, 181, 141), OF_GRADIENT_LINEAR);
        ofPopStyle();
        
        ofPushStyle();
        
        for (int i = 0; i < circles.size(); i++)
        {
            ofPushMatrix();
            ofTranslate(circles[i].get()->getPosition().x, circles[i].get()->getPosition().y);
            ofRotate(circles[i].get()->getRotation() + circles[i].get()->angle);
            ofSetRectMode(ofRectMode::OF_RECTMODE_CENTER);
            temari_fbos[circles[i]->temari_image_id].draw(0, 0, circles[i].get()->getRadius()*2, circles[i].get()->getRadius()*2);
            ofPopMatrix();
        }
        
        ofPopStyle();
        
        ofPushStyle();

        ofEnableBlendMode(OF_BLENDMODE_ADD);
        sprite.bind();
        ofEnablePointSprites();
        spkSystem.draw();
        ofDisablePointSprites();
        sprite.unbind();
        
        ofPopStyle();
    }
    
    if (debug)
    {
        if (showGui)
            gui.draw();
        
        ofPushStyle();
        for (int i = 0; i < contourFinder.blobs.size(); i++)
        {
            ofSetLineWidth(5);
            ofSetColor(ofColor::purple);
            polygons[i].draw();
        }
        ofPopStyle();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    switch (key) {
        case ' ':
            learnBackground = true;
            break;
            
        case OF_KEY_LEFT:
            threashold -= 5;
            break;
        
        case OF_KEY_RIGHT:
            threashold += 5;
            break;
        
        case OF_KEY_BACKSPACE:
            initializeSourceArea();
            break;
        
        case 'a':
            autoFall = !autoFall;
            break;
            
        case OF_KEY_RETURN:
            scene = scene == Scene::Setup ? Scene::Play : Scene::Setup;
            break;
            
        case 'c':
        {
            b2Temari circle;
            float r = 120;
            circles.push_back(shared_ptr<b2Temari>(new b2Temari));
            circles.back().get()->setPhysics(3.0, 0.53, 0.1);
            circles.back().get()->setup(box2d.getWorld(), mouseX, mouseY, r);
            break;
        }
        
        case 'f':
            ofToggleFullscreen();
            break;
            
        case 'd':
            debug = !debug;
            break;
            
        case 'w':
            whiteScreen = !whiteScreen;
            break;
            
        case 'g':
            showGui = !showGui;
            break;
            
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

    if (selectedCorner != -1)
    {
        source_corners[selectedCorner].x = ofClamp(x, 0, ofGetWidth());
        source_corners[selectedCorner].y = ofClamp(y, 0, ofGetHeight());
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

    for (int i = 0; i < source_corners.size(); i++)
    {
        if (ofPoint(x, y).distance(ofPoint(source_corners[i].x, source_corners[i].y)) < 10)
        {
            selectedCorner = i;
            break;
        }
        
        selectedCorner = -1;
    }
    
    if (ofGetKeyPressed('0'))
    {
        cv::findHomography(source_corners, destination_corners).convertTo(homography_matrix, CV_32FC1);
        cout << homography_matrix << endl;
    }
}

//--------------------------------------------------------------
void ofApp::exit(ofEventArgs &args){
    blackMagic.close();
}

//--------------------------------------------------------------
bool ofApp::sholdRemove(shared_ptr<ofxBox2dBaseShape> shape){
    return (shape->getPosition().y > ofGetHeight() + 1000);
//    return !ofRectangle(-100, -1000, ofGetWidth()+100, ofGetHeight()+100).inside(shape->getPosition());
}

//--------------------------------------------------------------
void ofApp::contactStart(ofxBox2dContactArgs &e){
//    if (e.b->GetType() == b2Shape::e_circle && e.a->GetType() == b2Shape::e_edge)
//    {
    
        float   theta     = e.b->GetBody()->GetAngle();
        float   velocity  = e.b->GetBody()->GetLinearVelocity().Length();
        ofVec3f direction = ofVec3f(velocity * cos(theta), velocity * sin(theta));
        ofVec3f position  = ofVec3f(e.b->GetBody()->GetPosition().x * OFX_BOX2D_SCALE  + radius * cos(theta), e.b->GetBody()->GetPosition().y * OFX_BOX2D_SCALE + radius * sin(theta), -10);
        
        spkGroup.emitSpheric(10, position, ofxSPK::RangeF(10, 40), -direction, ofxSPK::RangeF(0, 0.2));
        spkSystem.update();
//    }
}