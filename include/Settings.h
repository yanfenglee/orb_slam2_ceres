//
// Created by lyfpcy on 2019-07-31.
//

#ifndef ORB_SLAM2_SETTINGS_H
#define ORB_SLAM2_SETTINGS_H

namespace ORB_SLAM2 {
    class Settings {
    public:
        double fx;
        double fy;
        double cx;
        double cy;
    };

extern Settings settings;

#define sfx settings.fx
#define sfy settings.fy
#define scx settings.cx
#define scy settings.cy

}



#endif //ORB_SLAM2_SETTINGS_H
