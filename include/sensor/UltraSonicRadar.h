//
// Created by mikaa on 17-12-4.
//

#ifndef CANTEST_ULTRASONICRADAR_H
#define CANTEST_ULTRASONICRADAR_H

#include <can/CanNode.h>
#include <can/CanTransmitter.h>
#include <utils/PITime.h>
#include <utils/CircleBuffer.h>

namespace PIAUTO {
    namespace chassis {

        #define USRADAR_NUM    8
        #define DEFAULT_SONAR_BUFFER_SIZE    10    // 10 means 10 * 100ms = 1000ms
        #define TIME_INTERVAL_800 150
        #define DEBUG_SONAR false

        ///Ultrasonic radar frame analytical structure
        struct record800 {
            uint64_t Channel:8;
            uint64_t Range:16;
            uint64_t :40;
        };

        ///Ultrasonic radar buffer
        struct USRadarAttributes {
            record800 _800[USRADAR_NUM] = {
                {0, UINT16_MAX},
                {0, UINT16_MAX},
                {0, UINT16_MAX},
                {0, UINT16_MAX},
                {0, UINT16_MAX},
                {0, UINT16_MAX},
                {0, UINT16_MAX},
                {0, UINT16_MAX},
            };

            ///frame timers related to all Ultrasonic nodes
            time::Timer Timer_800[USRADAR_NUM];
        };

        ///Ultrasonic radar objects received once
        struct SonarData {
            unsigned short left_front;
            unsigned short right_front;

            unsigned short leftside_front;
            unsigned short rightside_front;

            unsigned short leftside_rear;
            unsigned short rightside_rear;

            unsigned short left_rear;
            unsigned short right_rear;
        };

        std::ostream &operator<<(std::ostream &os, const SonarData &obj);

        /**
         * @class UltraSonicRadar 
         * @brief UltraSonic radar class derived from CanNode, a kind of can node which store
         *          UltraSonic radar data and provide corresponding interfaces to get them.
         */
        class UltraSonicRadar : public CanNode {
          public:
            /**
             * @brief Constructor and destructor
             * @param c Cantransmitter used to send some config info(not used)
             */
            UltraSonicRadar(CanTransmitter *c);

            ~UltraSonicRadar() override;

            bool UpdateAttributes(VCI_CAN_OBJ &) override;

            bool VerifyFrameTimer() override;

            // [BEGIN] -- Getters, object range returned by specified radar
            USHORT GetRange_Front_Left() {
                return GetRangeByIndex(0);
            }

            USHORT GetRange_Front_Right() {
                return GetRangeByIndex(1);
            }

            USHORT GetRange_Back_Left() {
                return GetRangeByIndex(5);
            }

            USHORT GetRange_Back_Right() {
                return GetRangeByIndex(4);
            }

            USHORT GetRange_Left_Front() {
                return GetRangeByIndex(7);
            }

            USHORT GetRange_Left_Back() {
                return GetRangeByIndex(6);
            }

            USHORT GetRange_Right_Front() {
                return GetRangeByIndex(2);
            }

            USHORT GetRange_Right_Back() {
                return GetRangeByIndex(3);
            }
            // [END] -- Getters, object range returned by specified radar

            USHORT GetRangeByIndex(int index) {
                if (index < 0 || index > 7) {
                    LOG(WARNING) << "Failed to get USRADAR_Range:bounds error";
                    return 0;
                }
                std::shared_lock<std::shared_timed_mutex> lg(_mt);
                return ReverseHLValue(attri->_800[index].Range);
            }

            /**
             * @brief Get all objects received sevaral times.
             * @param[out] _buff Objects returned.
             * @param times Specified times.
             * @return True if times < buff_size.
             */
            bool GetObjectInfoByTimes(SonarData *_buff, int times);

          private:
            CanTransmitter *ct;
            CircleBuffer<SonarData> sonar_buffer;
            USRadarAttributes *attri;
        };
    }
}


#endif //CANTEST_ULTRASONICRADAR_H
