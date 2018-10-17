#pragma once

#include <condition_variable>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <utils/PITime.h>
#include "controlcan.h"
#include "CanNode.h"

#include <glog/logging.h>

#define LOGPATH "../log"

using std::endl;
using std::cout;
using std::string;
using std::thread;

namespace PIAUTO {
    namespace chassis {


        /**
       *  A custom semaphore implemented by condition_variable .
       */
        class Semaphore {
        public:
            Semaphore(long count = 0)
                    : count_(count) {
            }

            /**
             * @brief Increase the count and inform a waiting thread
             */
            void Signal() {
                std::unique_lock<std::mutex> lock(mutex_);
                ++count_;
                cv_.notify_one();
            }

            /**
             * @brief Waiting for a signal and decrease the count.
             */
            void Wait() {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [=] { return count_ > 0; });
                --count_;
            }

            /**
             * @brief Reset the semaphore and set the count to 0.
             */
            void Reset() {
                std::unique_lock<std::mutex> lock(mutex_);
                count_ = 0;
            }

        private:
            std::mutex mutex_;
            std::condition_variable cv_;
            long count_;
        };

        /**
         * @brief Convert FrameData to string.
         * @param frame Frame to be converted.
         * @return String converted from the frame.
         */
        string Frame2Str(VCI_CAN_OBJ &frame);

        /**
         * @brief Record frameId and data to a file.
         * @param file File to store the data
         * @param frame Frame to be recorded.
         */
        //
        void recordFrame(std::fstream &file, VCI_CAN_OBJ &frame);

        /**
         * @brief Convert char(0-f) to int(0-15)
         * @param chr Char to be converted.
         * @param[out] cint Int converted from char.
         * @return True If chr is between 0-f or 0-F.
         */
        bool CharToInt(unsigned char chr, unsigned char *cint);

        /**
         * @brief Convert hex string to char array:
         *        Convert "21" to 0x21(1 byte) ,"2121" to 0x2121(2 byte) ,"21 21" to 0x2121 if flag=1
         * @param str String to be converted.
         * @param[out] data Char array converted from string.
         * @param len Pair number in str("21" means a pair).
         * @param flag 1 means space exists between pairs.
         * @return True If chr is between 0-f or 0-F.
         */
        bool StrToData(unsigned char *str, unsigned char *data, int len, int flag);

        /**
         * @brief Insert val into raw data from begin to begin+lenth index
         * @param raw Data to be changed.
         * @param val Value to be inserted into raw.
         * @param begin Begin index.
         * @param length Bytes' number the val occupied.
         */
        void InsertDataOnBytes(BYTE *raw, int val, int begin, int length);

        /**
         * @brief Verify frame by checksum byte
         * @param frame Frame to be verified
         * @return True If test passed.
         */
        bool VerifyFrame(VCI_CAN_OBJ *frame);

        /**
         * @brief Reverse high-Low Byte
         * @param val Value to be reversed.
         * @return result.
         */
        USHORT ReverseHLValue(USHORT val);

        /**
         * @class CanTransmitter
         * @brief A transmitter used to initialize Can device, generate and send frame,
         * receive frames and distribute them to various Cannodes' buffers.
         */
        class CanTransmitter {
        public:

            typedef std::function<bool (VCI_CAN_OBJ &)> CanParse;

            /**
             * @brief Constructor
             * @param dt Device type.
             * @param idx Device index.
             * @param cn Can index(One device may has multiple can bus)
             * @param code Code.
             * @param mask Mask(used to filter some frames combined with code)
             * @param filterType Filter mode.
             * @param mode 0 for normal and 1 for listen only.
             * @param timing0 Frame rate timer 0.
             * @param timing1 Frame rate timer 1(determine the frame rate combined with timer 0).
             */
            CanTransmitter(
                    DWORD dt,
                    DWORD idx,
                    DWORD cn,
                    DWORD code,
                    DWORD mask,
                    UCHAR filterType,
                    UCHAR mode,
                    UCHAR timing0,
                    UCHAR timing1
            );

            /**
             * @brief Destructor.
             */
            ~CanTransmitter();

            /**
             * @brief Initialization of the can device.
             * @param code Same as Constructor.
             * @param mask -
             * @param filterType -
             * @param mode -
             * @param timing0 -
             * @param timing1 -
             */
            void Init(DWORD code,
                      DWORD mask,
                      UCHAR filterType,
                      UCHAR mode,
                      UCHAR timing0,
                      UCHAR timing1);

            /**
             * @brief Make up a frame with some string
             * @param ID Frame ID.
             * @param Data Frame data(8 bytes).
             * @param SendType 0 for normal, 1 for once, 2 for self-test, 3 for self-test once.
             * @param ExternFlag 1 for extended frame.
             * @param RomoteFlag 1 for remote frame.
             * @return Frame generated.
             */
            static VCI_CAN_OBJ
            GenerateFrame(const string &ID, const string &Data, BYTE SendType = 0, BYTE ExternFlag = 0,
                          BYTE RomoteFlag = 0);

            /**
             * @brief Generate checksum byte for a frame.
             * @param frame Frame to be handled.
             */
            static void GenerateChecksum(VCI_CAN_OBJ *frame);

            /**
             * @brief Send frame through can bus.
             * @return True If no error occur.
             */
            bool SendData(VCI_CAN_OBJ *);

            /**
             * @brief Receive frames thread.
             */
            void ReceiveData();

            void FrameRateMonitor() {
                std::thread::id frame_rate_monitor_id = std::this_thread::get_id();
                cout << "FrameRateMonitor thread(" << frame_rate_monitor_id << ") begin!" << endl;
                unsigned timer = 0;
                int sendFrameNum_temp = 0;
                int recvFrameNum_temp = 0;
                int sendFramRate = 0;
                int recvFramRate = 0;
                while (m_connect) {
                    usleep(1000 * 1000);
                    ++timer;
                    sendFramRate = sendFrameNum - sendFrameNum_temp;
                    sendFrameNum_temp = sendFrameNum;
                    if (sendFramRate > maxSendFrameRate)
                        maxSendFrameRate = sendFramRate;
                    recvFramRate = recvFrameNum - recvFrameNum_temp;
                    recvFrameNum_temp = recvFrameNum;
                    if (recvFramRate > maxRecvFrameRate)
                        maxRecvFrameRate = recvFramRate;
                    if (sendFramRate + recvFramRate > maxFrameRate)
                        maxFrameRate = sendFramRate + recvFramRate;
                }

                FrameRate = (sendFrameNum + recvFrameNum) / timer;
                cout << "RunTime       ： " << std::dec << timer << "s" << endl;
                cout << "SendFrameNum： " << sendFrameNum << endl;
                cout << "RecvFrameNum： " << recvFrameNum << endl;
                cout << "MaxSendFPS  : " << maxSendFrameRate << " fps" << endl;
                cout << "MaxRecvFPS  : " << maxRecvFrameRate << " fps" << endl;
                cout << "MaxFPS       : " << maxFrameRate << " fps" << endl;
                cout << "AveFPS       ： " << FrameRate << " fps" << endl;

                cout << "FrameRateMonitor Thread Exit" << endl;
            };

            void registerCallbacks(CanParse &canParse);
            void unregisterCallbacks(unsigned int &index);

        private:
            std::thread *monitor_Thread;
            int sendFrameNum = 0;
            int recvFrameNum = 0;
            int FrameRate = 0;
            int maxFrameRate = 0;
            int maxSendFrameRate = 0;
            int maxRecvFrameRate = 0;
            ///connect status
            bool m_connect = false;
            std::thread *ReceiveThread;
            std::mutex _mt;
            DWORD devtype;
            DWORD index;
            DWORD cannum;
            std::mutex mutexParses;
            std::shared_ptr<std::vector<CanParse>> parses_;
        };
    }
}
