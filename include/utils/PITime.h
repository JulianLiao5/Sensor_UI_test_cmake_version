//
// Created by mikaa on 17-12-28.
//

#ifndef PITIMER_H
#define PITIMER_H

#include <iostream>

namespace PIAUTO {
    namespace time {
        ///Get timestamps by millseconds
        uint64_t GetCurrentTimeMilliSec();

        ///Get timestamps by milliseconds
        uint64_t getCurrentTicks();

        ///Get timestamps(yyyy-mm-dd hh:mm:ss:us)
        std::string getTimeStamps();

        /**
         * @class Timer
         * @brief A custom timer.
         */
        class Timer {
        public:
            /**
             * @brief Reset time and restart timing.
             */
            void Reset() {
                oldtime = GetCurrentTimeMilliSec();
            }

            /**
             * @brief Get time since last reset.
             * @return Time(milliseconds).
             */
            uint64_t GetTime() {
                return GetCurrentTimeMilliSec() - oldtime;
            }

        private:
            uint64_t oldtime = 0;
        };
    }
}

#endif //PITIMER_H
