/*************************************************************************
	> File Name: test_ogm.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月28日 星期二 14时41分45秒
 ************************************************************************/

#include<functional>
#include <math.h>
#include<iostream>
#include<fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <utils/occupancy_status_grid_map.h>
#include <sensor/UltraSonicRadar.h>

using namespace std;

#define TIME_0    0x00
#define TIME_1    0x1C

int main(int argc, char *argv[]) {
  PIAUTO::chassis::CanTransmitter ct(VCI_USBCAN2, 0, 0, 0, 0, 0, 0, TIME_0, TIME_1);
  PIAUTO::chassis::UltraSonicRadar* sonar = new PIAUTO::chassis::UltraSonicRadar(&ct);
  PIAUTO::chassis::CanTransmitter::CanParse sonarParse = std::bind(&PIAUTO::chassis::UltraSonicRadar::UpdateAttributes, sonar, std::placeholders::_1);
  ct.registerCallbacks(sonarParse);

  // sonar data buffer
    PIAUTO::chassis::SonarData sonarObjs[DEFAULT_SONAR_BUFFER_SIZE] = {{
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX
    }};

  cv::Size map_size(1000, 1000);
  double size_per_pixel = 0.01;
  OccupancyStatusGridMap *OGM = new OccupancyStatusGridMap(size_per_pixel, map_size);

  cv::Mat car;
  cv::Mat temp_car = cv::imread("../../res/biro.png", CV_LOAD_IMAGE_COLOR);
  if (temp_car.data != nullptr) {
    double scale = (1.7 / OGM->cell_size()) / (temp_car.size().height);
    cv::resize(temp_car, car, cv::Size(), scale, scale, cv::INTER_LINEAR);
  }

  ofstream sonarFile;
  sonarFile.open("./sonar_data.txt", ios::out);

  while (1) {
    #if DEBUG
    std::thread::id main_id = std::this_thread::get_id();
    cout << "[" << __func__ << "] in thread(" << main_id << ")" << endl;
    #endif

    OGM->Reset();

    OGM->DrawLineInMap(cv::Point2d(0, 5), cv::Point2d(0, -5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(-5, 0), cv::Point2d(5, 0), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(1, -5), cv::Point2d(1, 5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(2, -5), cv::Point2d(2, 5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(3, -5), cv::Point2d(3, 5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(4, -5), cv::Point2d(4, 5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(5, -5), cv::Point2d(5, 5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(0, 0), cv::Point2d(5, 5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(0, 0), cv::Point2d(5, -5), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(0, 0), cv::Point2d(5, 5 * tan(30 * M_PI / 180)), CellStatus::UNKNOWN);
    OGM->DrawLineInMap(cv::Point2d(0, 0), cv::Point2d(5, -5 * tan(30 * M_PI / 180)), CellStatus::UNKNOWN);

    for (int i = 0; i <= 5; i += 2) {
      for (int j = -3; j <= 3; j += 1) {
        OGM->DrawRectInMap(Rect(i, j, 0.07, 0.8), CellStatus::UNKNOWN);
      }
    }

    OGM->DrawRectInMap(Rect(1, 0, 0.2, 0.07), CellStatus::UNKNOWN);
    OGM->DrawRectInMap(Rect(2, 0, 0.2, 0.07), CellStatus::UNKNOWN);
    OGM->DrawRectInMap(Rect(3, 0, 0.2, 0.07), CellStatus::UNKNOWN);
    OGM->DrawRectInMap(Rect(4, 0, 0.2, 0.07), CellStatus::UNKNOWN);
    OGM->DrawRectInMap(Rect(0, 0, 0.4, 0.14), CellStatus::UNKNOWN);
    OGM->DrawRectInMap(Rect(5, 0, 0.4, 0.14), CellStatus::UNKNOWN);

    sonar->GetObjectInfoByTimes(sonarObjs, DEFAULT_SONAR_BUFFER_SIZE);
    for (int j = 0; j != DEFAULT_SONAR_BUFFER_SIZE; ++j) {
        #if DEBUG
        printf("j: %d, FB[left_front] object distance: %f, FC[right_front] object distance: %f\n", j, sonarObjs[j].left_front / 1000.0f, sonarObjs[j].right_front / 1000.0f);
        #endif
        OGM->DrawRectInMap(Rect(sonarObjs[j].left_front / 1000.0f, 1.0, 0.2, 0.07));
        OGM->DrawRectInMap(Rect(sonarObjs[j].right_front / 1000.0f, -1.0, 0.2, 0.07));
    }

    cv::Mat m = OGM->Visualize();
    // cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    cv::QtFont font = cv::fontQt("Times");
    cv::addText(m, "0", cv::Point(504,498), font);
    cv::addText(m, "1", cv::Point(504,398), font);
    cv::addText(m, "2", cv::Point(504,298), font);
    cv::addText(m, "3", cv::Point(504,198), font);
    cv::addText(m, "4", cv::Point(504,98), font);
    cv::addText(m, "-3", cv::Point(204,504), font);
    cv::addText(m, "-2", cv::Point(304,504), font);
    cv::addText(m, "-1", cv::Point(404,504), font);
    cv::addText(m, "FB[left_front]", cv::Point(360,560), font);
    cv::addText(m, "1", cv::Point(604,504), font);
    cv::addText(m, "FC[right_front]", cv::Point(560,560), font);
    cv::addText(m, "2", cv::Point(704,504), font);
    cv::addText(m, "3", cv::Point(804,504), font);
    cv::addText(m, "-30deg", cv::Point(220,42), font);
    cv::addText(m, "-45deg", cv::Point(50,52), font);
    cv::addText(m, "30deg", cv::Point(740,42), font);
    cv::addText(m, "45deg", cv::Point(930,52), font);
    // cv::namedWindow("map", cv::WINDOW_NORMAL);
    /* if (car.data != nullptr) {
      cv::Rect roi_rect(OGM->grid_size().width / 2 - car.cols / 2, OGM->grid_size().height / 2 - car.rows / 2, car.cols, car.rows);
      car.copyTo(m(roi_rect));
    } else {
      OGM->DrawRectInMap(Rect(0, 0, 1, 1.6));
    } */
    cv::imshow("map", m);
    if ('q' == cv::waitKey(20)) {
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  sonarFile.close();

  return 0;
}
