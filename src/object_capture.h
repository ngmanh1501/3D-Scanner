#pragma once
 
#include <opencv2/opencv.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
 
 
// ============================================================
//  ObjectCaptureConfig
//  Cấu hình cho việc chụp object (tách hoàn toàn khỏi calib)
// ============================================================
struct ObjectCaptureConfig
{
    // --- Projector resolution ---
    int proj_w = 1024;
    int proj_h = 768;
 
    // --- Capture ---
    int warmupFrames   = 5;       // bỏ qua N frame trước khi lấy ảnh
    int patternDelayMs = 250;     // delay sau khi hiện pattern (ms)
 
    // --- Preview window ---
    int previewW = 500;
    int previewH = 375;
    int previewX = 700;
    int previewY = 290;
 
    // --- Root folder chứa các object sessions ---
    std::string rootPath = "Object_data";
};
 
 
// ============================================================
//  Chụp 1 session object:
//  - Chiếu toàn bộ Gray-code patterns
//  - Chụp ảnh tương ứng từ camera
//  - Lưu vào:  rootPath/<session_name>/
//               pattern_0.png … pattern_N.png
//               white.png
//               black.png
//
//  session_name ví dụ: "obj_cup", "obj_box_pose2", ...
//
//  overwriteConfirmed: TRƯỚC ĐÂY hàm này tự hỏi "y/n" qua cin khi
//  session_path đã tồn tại — điều đó chặn cứng nếu gọi từ 1 luồng
//  không có console tương tác (GUI). Giờ việc HỎI người dùng chuyển
//  ra ngoài cho nơi gọi (console: runObjectCapture() vẫn hỏi qua cin
//  như cũ; GUI: hiện popup ImGui rồi gọi lại với true) — hàm này chỉ
//  còn kiểm tra: nếu path đã tồn tại VÀ overwriteConfirmed=false thì
//  trả về false ngay (không chụp), không tự in prompt/chờ input nữa.
//
//  Trả về true nếu chụp thành công.
// ============================================================
// Tra ve pose_id ke tiep (max hien co + 1) cho 1 object_path — dung o
// ca console (runObjectCapture) va GUI (panel Capture) de khong lap
// lai logic quet thu muc "pose_N" 2 noi.
int getNextPoseID(const std::string& object_path);

bool captureObjectSession(
    cv::VideoCapture& cap,
    const std::vector<cv::Mat>& patterns,
    const ObjectCaptureConfig& cfg,
    const std::string& object_name,
    int pose_id,
    bool overwriteConfirmed = false);
 
 
// ============================================================
//  Vòng lặp tương tác chụp object
//
//  Người dùng nhập tên session trước mỗi lần chụp,
//  có thể chụp nhiều object / nhiều pose khác nhau.
//
//  Keys:
//    n : nhập tên và chụp session mới
//    l : liệt kê các session đã chụp
//    q : thoát
// ============================================================
void runObjectCapture(
    const ObjectCaptureConfig& cfg,
    int                        camera_url);
 







