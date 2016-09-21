#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxBlackMagic.h"
#include "ofxBox2d.h"
#include "ofxSPK.h"
#include "ofxGui.h"
#include "b2Temari.hpp"

#define MAX_BLOBS 10
#define CAM_WIDTH 1920
#define CAM_HEIGHT 1080

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    
        void initializeSourceArea();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
        void exit(ofEventArgs &args);
    
        void contactStart(ofxBox2dContactArgs &e);
    
        static bool sholdRemove(shared_ptr<ofxBox2dBaseShape> shape);
    
        enum Scene {
            Setup,
            Play
        };
        Scene scene;
		
        ofxBlackMagic blackMagic;
        ofxCvGrayscaleImage original, background, diff;
        ofxCvContourFinder contourFinder;
        int threashold;
        std::vector<cv::Point2f> source_corners;
        std::vector<cv::Point2f> destination_corners;
        int selectedCorner;
        cv::Mat homography_matrix;
    
        ofxBox2d box2d;
        std::array<ofPolyline, MAX_BLOBS> polylines;
        std::array<ofxBox2dEdge, MAX_BLOBS> polygons;
        std::vector<shared_ptr<b2Temari> > circles;
        std::array<ofImage, 3> temari_images;
        std::array<ofFbo, 3> temari_fbos;
        ofImage sprite;
    
        bool learnBackground;
        bool debug;
        bool autoFall;
        bool whiteScreen;
        bool showGui;
    
        ofxSPK::System   spkSystem;
    
        ofxSPK::Group    spkGroup;
        ofxSPK::Modifier spkMod;
        ofxSPK::Emitter  spkEmitter;
    
        ofxSPK::Group    spkBackGroup;
        ofxSPK::Modifier spkBackMod;
        ofxSPK::Emitter  spkBackEmitter;
    
        ofxPanel gui;
        ofParameter<float> radius;
};
