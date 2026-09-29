#ifndef CaMERA_CALIB_H
#define CaMERA_CALIB_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <filesystem>
#include <fstream>


using namespace cv;
using namespace std;
namespace fs = std::filesystem;



void captureImages(int url, string folder);
void calibrateAndSave(string folder, const string& outputFile);
void runcalibration(int url, string folder, const string& outputFile);


#endif