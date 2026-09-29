#pragma once
 
#include <opencv2/opencv.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
 
// ============================================================
//  ProjectorCalibConfig
//  Tất cả các hằng số liên quan đến calib projector
// ============================================================
struct ProjectorCalibConfig
{
    // --- Projector resolution (projector chiếu full-screen đúng độ phân giải này) ---
    int proj_w = 1024;
    int proj_h = 768;
 
    // --- Checkerboard ---
    cv::Size boardSize  = {9, 7};
    float    squareSize = 23.0f;   // mm
 
    // --- Decode ---
    int threshold   = 35;           // ngưỡng sáng/tối để nhận bit
    int patchRadius = 2;           // bán kính patch decode quanh corner
    int minDecodePoints = 15;      // số điểm tối thiểu để chấp nhận 1 pose

    // --- Số pose tối thiểu để computeProjectorCalib() chịu tính calib ---
    // (trước đây là const int MIN_POSES = 5; hard-code trong .cpp)
    int minPoses = 5;
 
    // --- Projector intrinsic guess ---
    // Projector throw ratio thường cho FOV rộng hơn camera thông thường,
    // nên hệ số < 1.0 hợp lý hơn làm điểm khởi đầu cho solver
    // (0.6–1.0 tuỳ throw ratio thực tế của máy chiếu).
    double focalGuess() const { return proj_w * 0.8; }
 
    // --- Capture ---
    int warmupFrames   = 5;        // bỏ qua N frame trước khi lấy ảnh
    int patternDelayMs = 300;      // delay sau khi hiện pattern
 
    // --- Output file ---
    std::string outputJson = "projector_calibration.json";
};
 
 
// ============================================================
//  PatternSet
//  Gói gọn tập ảnh chụp 1 pose (patterns + white + black)
// ============================================================
struct PatternSet
{
    std::vector<cv::Mat> images;   // [0..N-1] = Gray-code patterns
                                   // [N-2]    = white
                                   // [N-1]    = black
};
 
 
// ============================================================
//  Hàm decode 1 pose (ảnh → điểm camera ↔ điểm projector)
//
//  Trả về true nếu tìm đủ điểm hợp lệ.
//  Tọa độ proj_valid trả ra ở hệ Gray-code/projector gốc [0..proj_w, 0..proj_h]
//  — cũng chính là hệ mà Kp/dist_p được calib (projector chiếu full-screen).
// ============================================================
bool decodePose(
    const PatternSet&            ps,
    const ProjectorCalibConfig&  cfg,
    std::vector<cv::Point3f>&    obj_valid,
    std::vector<cv::Point2f>&    cam_valid,
    std::vector<cv::Point2f>&    proj_valid);
 
 
// ============================================================
//  Hàm chụp 1 pose từ camera + projector
// ============================================================
bool captureCalibPose(
    cv::VideoCapture&            cap,
    const std::vector<cv::Mat>&  patterns,
    const ProjectorCalibConfig&  cfg,
    const std::string&           root_path,
    int                          pose_id,
    std::vector<cv::Point3f>&    obj_valid,
    std::vector<cv::Point2f>&    cam_valid,
    std::vector<cv::Point2f>&    proj_valid);
 
 
// ============================================================
//  Tính calib projector + stereo từ dữ liệu điểm đã có.
//  Kết quả Kp, dist_p, R, T lưu vào cfg.outputJson.
// ============================================================
bool computeProjectorCalib(
    const std::vector<std::vector<cv::Point3f>>& all_obj,
    const std::vector<std::vector<cv::Point2f>>& all_cam,
    const std::vector<std::vector<cv::Point2f>>& all_proj,
    const cv::Mat&                Kc,
    const cv::Mat&                dist_c,
    const ProjectorCalibConfig&   cfg);
 
 
// ============================================================
//  Load lại ảnh từ folder đã lưu, decode rồi calib lại
// ============================================================
bool recalibrateFromFolder(
    const std::string&           root_path,
    const cv::Mat&               Kc,
    const cv::Mat&               dist_c,
    const ProjectorCalibConfig&  cfg);
 
 
// ============================================================
//  Hàm chính: vòng lặp tương tác calib projector
// ============================================================
void runProjectorCalibration(
    const ProjectorCalibConfig& cfg,
    int                         camera_url,
    const std::string&          cameraCalibFile);