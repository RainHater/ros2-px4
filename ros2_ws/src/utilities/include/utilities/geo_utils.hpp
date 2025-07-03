#ifndef _GEO_UTILS_H
#define _GEO_UTILS_H

#include <math.h>

namespace geo_utils {
//角度转弧度
inline double deg2rad(double deg){
    return deg * M_PI / 180.0;
}
//GPS → ENU 简易转换函数（参考点 + 当前点 → x, y）
inline void gps_to_local(double lat0, double lon0, 
                double lat, double lon, 
                double &x, double &y){
    //简化的球面近似方法（ENU坐标）
    //地球半径，单位米
    double R = 6378137.0; 
    double dLat = deg2rad(lat - lat0);
    double dLon = deg2rad(lon - lon0);

    //东向（x）
    x = R * dLon * cos(deg2rad(lat0)); 
    //北向（y）
    y = R * dLat;     
}
}

#endif
