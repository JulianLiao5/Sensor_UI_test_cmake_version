//
// Created by mikaa-mi on 18-4-24.
//

#ifndef CONTROL_CIRCLEBUFFER_H
#define CONTROL_CIRCLEBUFFER_H

#endif //CONTROL_CIRCLEBUFFER_H
#include <glog/logging.h>

namespace PIAUTO {
    namespace chassis {


        /**
         * A custom circle buffer used to store sensor data without pop interface
         * therefore, the oldest element will be replaced when a new element is pushed
         */
        template<typename T>
        class CircleBuffer {
        public:
            /**
            * @brief Constructor.
            * @param _buf_size Max size of the buffer.
            */
            CircleBuffer(int _bufSize);

            /**
            * @brief Destructor.
            */
            ~CircleBuffer();

            /**
            * @brief Push a new element to buffer.
            * @param _b Element to be pushed
            */
            void Push(T _b);

            /**
            * @brief Push all elements stored in _buf to buffer.
            * @param _buf Store elements to be pushed.
            * @param _len Number of elements to be pushed.
            */
            void Push(const T *_buf, int _len);

            /**
            * @brief Retrieve data from the buffer.
            * @param[out] _buf Store all elements retrieved from the buffer.
            * @param _len Number of elements to be retrieved.
            * @return True if _len is less than buff size.
            */
            bool GetData(T *_buf, int _len);

            /**
            * @brief Access the kth new element.
            * @param k Index of the times.
            * @return Element at the specified position in the buffer.
             * @throw std::ut_of_range If k > bufSize.
            */
            const T &at(int k);

            /**
             * @brief Return buffSize of the buffer.
             */
            int DataLength();

        private:
            T *buf;
            int bufSize;
            /// head point to the start of current data, tail point to the end data's next.
            int head;
            int tail;
        };

        template<typename T>
        CircleBuffer<T>::CircleBuffer(int _bufSize) {
            buf = new T[_bufSize];
            bufSize = _bufSize;
            head = 0;
            tail = 0;
        }

        template<typename T>
        CircleBuffer<T>::~CircleBuffer() {
            if (buf) {
                delete[] buf;
                buf = nullptr;
            }
        }

        template<typename T>
        int CircleBuffer<T>::DataLength() {
            /*if (tail >= head) {
                return tail - head;
            } else {
                return tail + bufSize - head;
            }*/
            return bufSize;
        }

        template<typename T>
        void CircleBuffer<T>::Push(const T *_buf, int _len) {
            for (int i = 0; i < _len; i++) {
                buf[tail] = _buf[i];
                tail++;

                if (tail >= bufSize) {
                    tail -= bufSize;
                }

                if (tail == head)
                    ++head;

                if (head >= bufSize) {
                    head -= bufSize;
                }
            }
        }

        template<typename T>
        void CircleBuffer<T>::Push(T _b) { Push(&_b, 1); }

        template<typename T>
        bool CircleBuffer<T>::GetData(T *_buf, int _len) {
            if (0 == head && 0 == tail) {
                LOG(ERROR) << "No data, the CircleBuffer is empty!";
                return false;
            }
            int dataLen = DataLength();
            if (_len > dataLen) {
                LOG(ERROR) << "get data too much: " << _len << " " << dataLen;
                return false;
            }

            #if 0
                auto temp_tail = tail - 1;
                for (int i = _len - 1; i >= 0; --i) {
                    if (temp_tail < 0) {
                        temp_tail += bufSize;
                    }
                    _buf[i] = buf[temp_tail];
                    --temp_tail;
                }
            #else
                for (int i = _len - 1; i >= 0; --i) {
                    if (--tail < 0) {
                        // if (1 == head) {    // tail = 0 and head = 1 means "Data is full"
                            tail += bufSize;
                            head = 0;
                        // }
                        /**else if (0 == head) {  // tail = 0 and head = 0 means "Data is empty"
                            tail = 0;
                            LOG(ERROR) << "No data, the CircleBuffer is empty!";
                            return false;
                        }**/
                    }
                    _buf[i] = buf[tail];
                }
            #endif
            return true;
        }

        template<typename T>
        const T &CircleBuffer<T>::at(int k) {
            if (k >= bufSize) {
                throw std::out_of_range("bound error in circle buffer!");
            }

            k = tail - 1 - k;

            if (k < 0)
                k += bufSize;

            return buf[k];
        }
    }
}
