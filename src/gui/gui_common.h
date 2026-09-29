#pragma once

#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <iostream>

// ================================================================
//  GuiLogSink
//  ----------------------------------------------------------------
//  Thay the viec doc truc tiep tu console: redirect std::cout vao
//  buffer nay 1 lan luc khoi dong GUI (xem gui_main.cpp), roi panel
//  Log chi can goi snapshot() moi frame de ve len ImGui::TextUnformatted.
//
//  Nho vay TOAN BO code cu (camera_calib.cpp, object_capture.cpp,
//  triangulator.cpp, multipose_triangulator.cpp...) khong can sua 1
//  dong "cout <<" nao ca — chung van in nhu cu, chi khac noi hien thi.
// ================================================================
class GuiLogSink : public std::streambuf
{
public:
    explicit GuiLogSink(size_t maxLines = 3000) : maxLines_(maxLines) {}

    std::string snapshot() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return buffer_;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        buffer_.clear();
        lineCount_ = 0;
    }

protected:
    int overflow(int c) override
    {
        if (c == EOF) return c;
        std::lock_guard<std::mutex> lk(mtx_);
        buffer_.push_back(static_cast<char>(c));
        if (c == '\n') { lineCount_++; trimIfNeeded(); }
        return c;
    }

    std::streamsize xsputn(const char* s, std::streamsize n) override
    {
        std::lock_guard<std::mutex> lk(mtx_);
        buffer_.append(s, static_cast<size_t>(n));
        for (std::streamsize i = 0; i < n; i++)
            if (s[i] == '\n') lineCount_++;
        trimIfNeeded();
        return n;
    }

private:
    // Gioi han so dong de log khong phinh vo han trong 1 session chay dai
    void trimIfNeeded()
    {
        while (lineCount_ > maxLines_)
        {
            size_t pos = buffer_.find('\n');
            if (pos == std::string::npos) break;
            buffer_.erase(0, pos + 1);
            lineCount_--;
        }
    }

    mutable std::mutex mtx_;
    std::string buffer_;
    size_t lineCount_ = 0;
    size_t maxLines_;
};


// ================================================================
//  BackgroundJob
//  ----------------------------------------------------------------
//  Chay 1 tac vu dai (calibrateAndSave, reconstruct, icpAlign,
//  captureObjectSession...) tren thread rieng, KHONG tren thread render
//  ImGui — tranh app "dung hinh" khi dang xu ly.
//
//  Moi panel nen co 1 BackgroundJob RIENG (xem GuiState ben duoi) de
//  bam qua tab khac trong luc dang chay khong bi anh huong — nhung
//  can luu y: cac ham dung chung 1 cv::VideoCapture / cv::namedWindow
//  thi KHONG duoc chay 2 job cung luc (xem canh bao trong tung panel).
// ================================================================
class BackgroundJob
{
public:
    ~BackgroundJob() { join(); }

    bool isBusy() const { return busy_.load(); }

    // Tra false neu job truoc van dang chay (goi lai sau).
    bool start(std::function<void()> fn)
    {
        if (busy_.load()) return false;
        join();               // don thread cu (da xong viec) truoc khi tao moi
        lastError_.clear();
        busy_ = true;
        thread_ = std::thread([this, fn]()
        {
            try { fn(); }
            catch (const std::exception& e) { lastError_ = e.what(); }
            catch (...) { lastError_ = "Loi khong xac dinh trong background job"; }
            busy_ = false;
        });
        return true;
    }

    // Doc + xoa loi gan nhat (goi 1 lan tren main thread moi frame la du)
    std::string takeLastError()
    {
        std::string e = lastError_;
        lastError_.clear();
        return e;
    }

private:
    void join() { if (thread_.joinable()) thread_.join(); }

    std::thread thread_;
    std::atomic<bool> busy_{false};
    std::string lastError_;
};


// ================================================================
//  GuiState
//  ----------------------------------------------------------------
//  State UI chia se giua cac panel, thay cho cac bien local trong
//  main.cpp/vong lap cin cu (pickFromList, getline ten session...).
//  1 instance duy nhat duoc tao trong gui_main.cpp va truyen tham
//  chieu vao moi ham DrawXxxPanel().
//
//  cfgPtr duoc ep kieu AppConfig* ngay trong gui_panels.cpp (khong
//  include app_config.h o day de tranh include vong / phinh file
//  header dung chung cho moi panel).
// ================================================================
struct GuiState
{
    void* cfgPtr = nullptr;

    int activePanel = 0;   // index muc dang chon o sidebar

    // ---- Lua chon object/pose/group/model (thay pickFromList cin) ----
    int selectedObject = -1;
    int selectedPose   = -1;
    int selectedGroup  = -1;
    int selectedModel  = -1;

    // ---- Text input (thay getline nhap ten session/group) ----
    char newSessionName[128] = "";
    char newGroupName[128]   = "";
    char faceLabel[64]       = "face_0";

    // ---- Job nen: MOI panel 1 job rieng ----
    BackgroundJob calibJob;     // camera + projector calib
    BackgroundJob captureJob;   // chup object
    BackgroundJob triJob;       // tri / tri_multi / autocalib
    BackgroundJob modelJob;     // build_model / locate

    // ---- Popup xac nhan ghi de (thay cin >> confirm 'y'/'n') ----
    bool showOverwriteConfirm = false;
    std::function<void()> onConfirmOverwrite;

    // ---- Log ----
    GuiLogSink* log = nullptr;
};
