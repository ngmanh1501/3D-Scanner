#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "app_config.h"   // AppPaths

// ================================================================
//  pipeline_io.h
//  ----------------------------------------------------------------
//  4 ham nay TRUOC DAY la "static" ben trong main.cpp. Ca console
//  (main.cpp) va GUI (gui_panels.cpp: DrawTriPanel/DrawTriMultiPanel/
//  DrawAutoCalibPanel/DrawBuildModelPanel/DrawLocatePanel) deu can
//  load calib + patterns theo DUNG 1 cach — chuyen ra day de dung
//  chung, khong copy logic lan 2.
// ================================================================

bool loadCameraCalib(const std::string& path, cv::Mat& Kc, cv::Mat& dist_c);

bool loadProjectorCalib(const std::string& path,
                         cv::Mat& Kp, cv::Mat& dist_p, cv::Mat& R, cv::Mat& T);

// folder/pattern_0.png .. pattern_{count-1}.png + white.png + black.png
bool loadPatterns(const std::string& folder, int count, std::vector<cv::Mat>& patterns);

// Goi ca loadCameraCalib + loadProjectorCalib, in debug Kc/Kp/R/T ra cout.
bool loadAllCalib(const AppPaths& paths,
                   cv::Mat& Kc, cv::Mat& dist_c,
                   cv::Mat& Kp, cv::Mat& dist_p,
                   cv::Mat& R,  cv::Mat& T);
