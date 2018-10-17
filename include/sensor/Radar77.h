//
// Created by mikaa on 17-12-4.
//

#ifndef CANTEST_RADAR77_H
#define CANTEST_RADAR77_H

#include <can/CanNode.h>
#include <can/CanTransmitter.h>
#include <utils/PITime.h>
#include <utils/CircleBuffer.h>

namespace PIAUTO {
    namespace chassis {

        #define RADAR77_NUM    1
        #define DEFAULT_BUFFER_SIZE    20    // 20 means 20 * 50ms = 1000ms
        #define TIME_INTERVAL_500 150
        #define DEBUG false

        ///radar77 frame analytical structure
        struct record500 {
            uint64_t Range:15;
            uint64_t :1;
            uint64_t RadialVelocity:14;
            uint64_t :2;
            uint64_t RadialAcc:10;
            uint64_t Angle:11;
            uint64_t Power:10;
            uint64_t :1;
        };

        ///radar77 buffer
        struct Radar_77Attributes {
            record500 _500[64] = {{0}};
            ///frame timer
            time::Timer Timer_500;
        };

        ///radar77 object
        struct ObjectInfo_77 {
            int index;
            float Range;
            float RadialVelocity;
            float RadialAcc;
            float Azimuth;
            float Power;
        };

        std::ostream &operator<<(std::ostream &os, const ObjectInfo_77 &obj);

        /**
         * @class Radar_77
         * @brief Radar77 class derived from CanNode, a kind of can node which store
         *          radar77 data and provide corresponding interfaces to get them.
         */
        class Radar_77 : public CanNode {
          public:
            /**
             * @brief Constructor and destructor
             * @param id Radar ID
             * @param c Cantransmitter used to send some config info(not used)
             */
            Radar_77(int id, CanTransmitter *c);

            ~Radar_77() override;

            bool UpdateAttributes(VCI_CAN_OBJ &) override;

            bool VerifyFrameTimer() override;

            /**
             * @brief Get all objects received once.
             * @return Objects.
             */
            std::vector<ObjectInfo_77> GetObjectInfo();

            /**
             * @brief Get all objects received sevaral times.
             * @param[out] _buff Objects returned.
             * @param times Specified times.
             * @return True if times < buff_size.
             */
            bool GetObjectInfoByTimes(std::vector<ObjectInfo_77> *_buff, int times);

          private:
            int ID;
            CanTransmitter *ct;
            std::vector<ObjectInfo_77> objs;
            CircleBuffer<std::vector<ObjectInfo_77>> radar_buffer;
            int obj_index=0;
            Radar_77Attributes *attri;
        };
    }
}


#endif //CANTEST_RADAR77_H
