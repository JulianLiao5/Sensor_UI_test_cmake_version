//
// Created by mikaa on 17-12-29.
//
#pragma once
#ifndef CANTEST_CANNODE_H
#define CANTEST_CANNODE_H

#include "controlcan.h"

#include <fstream>
#include <shared_mutex>    // shared_timed_mutex
#include <string>

using std::string;

namespace PIAUTO {
    namespace chassis {

        /**
         * @class CanNode
         * @brief Base class of all can node class
         */
        class CanNode {
        public:
            /**
             * @brief Analyze and extract frame data to can node object's buffer.
             * @param frame Frame to be analyzed.
             * @return True If frame is belong to this kind of can node.
             */
            virtual bool UpdateAttributes(VCI_CAN_OBJ &frame)=0;

            /**
             * @brief Verify frames' interval to judge if the node is healthy.
             * @return True If time is less than limited time.
             */
            virtual bool VerifyFrameTimer()=0;

            virtual ~CanNode();

            /**
             * @brief Constructor, open a specified file if needed.
             * @param file_name File to be opened.
             */
            CanNode(const string &file_name);

            /**
             * frame record files path
             */
            static string FRAME_RECORD_PATH ;

        protected:
            std::fstream logFile;
            static int logFileCount;
            std::shared_timed_mutex _mt;
        };
    }
}
#endif //CANTEST_CANNODE_H
