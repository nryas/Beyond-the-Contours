//
//  b2Temari.hpp
//  Beyond the Contours
//
//  Created by KoshinoLab on 2016/09/15.
//
//

#pragma once

#include "ofMain.h"
#include "ofxBox2d.h"

class b2Temari : public ofxBox2dCircle {
    public:
        int temari_image_id;
        float angle;
};