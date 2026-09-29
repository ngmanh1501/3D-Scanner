#include "pipeline_io.h"
#include "decode.h"     // sl::loadCalibration
#include <iostream>

using namespace std;
using namespace cv;

bool loadCameraCalib(const string& path, Mat& Kc, Mat& dist_c)
{
    sl::loadCalibration(path, Kc, dist_c);
    if (Kc.empty()) { cerr << "[ERROR] " << path << " invalid\n"; return false; }
    return true;
}

bool loadProjectorCalib(const string& path,
                         Mat& Kp, Mat& dist_p, Mat& R, Mat& T)
{
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened()) { cerr << "[ERROR] Cannot open " << path << "\n"; return false; }
    fs["cameraMatrix"] >> Kp;
    fs["distCoeffs"]   >> dist_p;
    fs["R"]            >> R;
    fs["T"]            >> T;
    return true;
}

bool loadPatterns(const string& folder, int count, vector<Mat>& patterns)
{
    patterns.clear();
    for (int i = 0; i < count; i++)
    {
        Mat img = imread(folder + "/pattern_" + to_string(i) + ".png", IMREAD_GRAYSCALE);
        if (img.empty()) { cerr << "[ERROR] Missing pattern_" << i << "\n"; return false; }
        patterns.push_back(img);
    }
    Mat white = imread(folder + "/white.png", IMREAD_GRAYSCALE);
    Mat black = imread(folder + "/black.png", IMREAD_GRAYSCALE);
    if (white.empty() || black.empty()) { cerr << "[ERROR] Missing white/black\n"; return false; }
    patterns.push_back(white);
    patterns.push_back(black);
    cout << "[INFO] Loaded " << patterns.size() << " images\n";
    return true;
}

bool loadAllCalib(const AppPaths& paths,
                   Mat& Kc, Mat& dist_c,
                   Mat& Kp, Mat& dist_p,
                   Mat& R,  Mat& T)
{
    if (!loadCameraCalib(paths.cameraCalibFile, Kc, dist_c))    return false;
    if (!loadProjectorCalib(paths.projectorCalibFile, Kp, dist_p, R, T)) return false;
    cout << "[DEBUG] Kc:\n" << Kc << "\n[DEBUG] Kp:\n" << Kp << "\n"
         << "[DEBUG] R:\n"  << R  << "\n[DEBUG] T:\n"  << T  << "\n";
    return true;
}
