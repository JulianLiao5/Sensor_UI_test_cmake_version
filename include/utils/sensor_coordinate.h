//
// Created by mikaa-mi on 18-4-19.
//

#ifndef CONTROL_SENSOR_COORDINATE_H
#define CONTROL_SENSOR_COORDINATE_H

#include <eigen3/Eigen/Core>

#include <sensor/Radar77.h>

namespace PIAUTO {
    namespace sensor {

        enum SensorType {
            RADAR77 = 0
        };

        class SensorCoordinate {
        public:
            SensorCoordinate();

            Eigen::Vector3d Radar77Obj2Coordinate(const chassis::ObjectInfo_77 &obj, int index);

        private:
            Eigen::Matrix<double, 3, 3> world_to_map_radar[RADAR77_NUM];
        };

    }
}
#endif //CONTROL_SENSOR_COORDINATE_H
