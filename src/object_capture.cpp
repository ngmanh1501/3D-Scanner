#include "object_capture.h"
 
using namespace cv;
using namespace std;
namespace fs = std::filesystem;
 
 
// ============================================================
//  Internal helper: chụp 1 frame ổn định
// ============================================================
static Mat grabFrame(VideoCapture& cap, int warmup)
{
    Mat frame;
    for (int k = 0; k < warmup; k++)
        cap >> frame;
    return frame;
}
 
 
// Bo "static": GUI (gui_panels.cpp) can goi lai ham nay de hien so pose
// tiep theo trong panel Capture, khong the tao ban sao logic thu 2.
int getNextPoseID(
    const string& object_path)
{
    if (!fs::exists(object_path))
        return 0;

    int maxPose = -1;

    for (const auto& entry :
         fs::directory_iterator(object_path))
    {
        if (!entry.is_directory())
            continue;

        string name =
            entry.path().filename().string();

        if (name.rfind("pose_",0)!=0)
            continue;

        int id =
            stoi(name.substr(5));

        maxPose = max(maxPose,id);
    }

    return maxPose + 1;
}

// ============================================================
//  confirmOverwriteConsole
//  Y/N qua cin — TÁCH RA khỏi captureObjectSession() để hàm đó không
//  còn phụ thuộc console (xem lý do trong object_capture.h). Chỉ dùng
//  ở runObjectCapture() (chế độ console); GUI có popup riêng, KHÔNG
//  gọi hàm này.
// ============================================================
static bool confirmOverwriteConsole(
    const ObjectCaptureConfig& cfg,
    const string& session_name,
    int pose_id)
{
    string session_path =
        cfg.rootPath + "/" + session_name + "/pose_" + to_string(pose_id);

    if (!fs::exists(session_path))
        return true;   // chưa tồn tại -> không cần hỏi

    cout << "[WARN] Session \"" << session_name << "\" đã tồn tại. Ghi đè? (y/n): ";
    char confirm;
    cin >> confirm;
    if (confirm != 'y' && confirm != 'Y')
    {
        cout << "[SKIP] Bỏ qua session.\n";
        return false;
    }
    return true;
}


// ============================================================
//  captureObjectSession
// ============================================================
bool captureObjectSession(
    VideoCapture&            cap,
    const vector<Mat>&       patterns,
    const ObjectCaptureConfig& cfg,
    const string&            session_name,
    int                       pose_id,
    bool                      overwriteConfirmed)
{
    // Tạo thư mục lưu ảnh
    string session_path =
        cfg.rootPath + "/" +
        session_name +
        "/pose_" +
        to_string(pose_id);
 
    // Việc HỎI người dùng "ghi đè? y/n" đã chuyển ra ngoài (xem
    // object_capture.h) — ở đây chỉ còn kiểm tra cờ đã xác nhận.
    // Console (runObjectCapture) tự hỏi qua cin như cũ rồi gọi lại
    // với overwriteConfirmed=true; GUI hiện popup rồi làm tương tự.
    if (fs::exists(session_path) && !overwriteConfirmed)
    {
        cout << "[SKIP] Session \"" << session_name
             << "\" đã tồn tại, chưa xác nhận ghi đè.\n";
        return false;
    }
 
    fs::create_directories(session_path);
    cout << "\n=== CAPTURE OBJECT SESSION: " << session_name << " ===\n";
    cout << "[INFO] Saving to: " << session_path << "\n";
    cout << "[INFO] Patterns to project: " << patterns.size() << "\n";
 
    // ---- Gray-code patterns ----
    for (int i = 0; i < (int)patterns.size(); i++)
    {
        imshow("projector", patterns[i]);
        waitKey(cfg.patternDelayMs);
 
        Mat frame = grabFrame(cap, cfg.warmupFrames);
        if (frame.empty())
        {
            cout << "[ERROR] Empty frame at pattern " << i << "\n";
            return false;
        }
 
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
 
        string file = session_path + "/pattern_" + to_string(i) + ".png";
        imwrite(file, gray);
        cout << "  [" << i + 1 << "/" << patterns.size() << "] pattern_" << i << ".png\n";
    }
 
    // ---- White ----
    {
        Mat white_pat(cfg.proj_h, cfg.proj_w, CV_8U, Scalar(255));
        imshow("projector", white_pat);
        waitKey(cfg.patternDelayMs);
 
        Mat frame = grabFrame(cap, cfg.warmupFrames);
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        imwrite(session_path + "/white.png", gray);
        cout << "  Captured white.png\n";
    }
 
    // ---- Black ----
    {
        Mat black_pat(cfg.proj_h, cfg.proj_w, CV_8U, Scalar(30));
        imshow("projector", black_pat);
        waitKey(cfg.patternDelayMs);
 
        Mat frame = grabFrame(cap, cfg.warmupFrames);
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        imwrite(session_path + "/black.png", gray);
        cout << "  Captured black.png\n";
    }
 
    cout << "[DONE] Session \"" << session_name << "\" saved.\n";
    return true;
}
 
 
// ============================================================
//  runObjectCapture  –  vòng lặp tương tác
// ============================================================
void runObjectCapture(const ObjectCaptureConfig& cfg, int camera_url)
{
    cout << "\n====== OBJECT CAPTURE ======\n"
         << "  n : chup new session (nhập tên object/pose)\n"
         << "  c : chup tiep pose moi cho object hien tai\n"
         << "  l : liet ke sessions da chup\n"
         << "  q : thoat\n\n";
 
    fs::create_directories(cfg.rootPath);
    string currentObject;

    // ---- Tạo Gray-code patterns ----
    auto graygen = structured_light::GrayCodePattern::create(cfg.proj_w, cfg.proj_h);
    vector<Mat> patterns;
    graygen->generate(patterns);
    cout << "[INFO] Pattern count = " << patterns.size() << "\n";
 
    // ---- Mở camera ----
    VideoCapture cap(camera_url);
    if (!cap.isOpened()) { cout << "[ERROR] Cannot open camera\n"; return; }
 
    // ---- Windows ----
    namedWindow("projector", WINDOW_NORMAL);
    setWindowProperty("projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    // resizeWindow("projector", cfg.previewW, cfg.previewH);
    // moveWindow("projector", cfg.previewX, cfg.previewY);
    namedWindow("camera", WINDOW_NORMAL);
    resizeWindow("camera", 320*2, 240*2);
    moveWindow("camera", 200, 200);
 
    // ---- Main loop ----
    while (true)
    {
        char key = (char)waitKey(1);
 
        // Preview camera liên tục
        Mat frame;
        cap >> frame;
        if (!frame.empty())
            imshow("camera", frame);
 
        if (key == 'n')
        {
            // Flush stdin phòng trường hợp còn newline
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
 
            cout << "\nNhap ten session (vd: cup_pose1, box_top, ...): ";
            string name;
            getline(cin, name);
 
            // Trim khoảng trắng đầu cuối
            auto trim = [](string& s) {
                s.erase(0, s.find_first_not_of(" \t\r\n"));
                s.erase(s.find_last_not_of(" \t\r\n") + 1);
            };
            trim(name);
 
            if (name.empty())
            {
                cout << "[WARN] Tên rỗng, bỏ qua.\n";
                continue;
            }
            
            currentObject = name;
            string object_path =
                cfg.rootPath + "/" + name;

            int poseID =
                getNextPoseID(object_path);

            waitKey(cfg.patternDelayMs*10);

            // Console: hoi "y/n" TAI DAY (thay vi trong captureObjectSession)
            // roi truyen ket qua qua overwriteConfirmed — giu dung UX cu.
            bool overwriteOk = confirmOverwriteConsole(cfg, name, poseID);
            captureObjectSession(
                cap,
                patterns,
                cfg,
                name,
                poseID,
                overwriteOk);
        }
        else if (key == 'c')
        {
            if (currentObject.empty())
            {
                cout << "Hay tao object moi bang phim n\n";
                continue;
            }

            string object_path =
                cfg.rootPath + "/" + currentObject;

            int poseID =
                getNextPoseID(object_path);

            bool overwriteOk = confirmOverwriteConsole(cfg, currentObject, poseID);
            captureObjectSession(
                cap,
                patterns,
                cfg,
                currentObject,
                poseID,
                overwriteOk);
        }
        else if (key == 'l')
        {
            cout << "\n--- Sessions trong \"" << cfg.rootPath << "\" ---\n";
            if (!fs::exists(cfg.rootPath))
            {
                cout << "  (Not available sessions)\n";
                continue;
            }
 
            int count = 0;
            vector<fs::path> dirs;
            for (const auto& entry : fs::directory_iterator(cfg.rootPath))
                if (entry.is_directory())
                    dirs.push_back(entry.path());
            sort(dirs.begin(), dirs.end());
 
            for (const auto& obj : dirs)
            {
                cout << "\n"
                    << obj.filename().string()
                    << endl;

                int poseCount = 0;

                for (const auto& pose :
                    fs::directory_iterator(obj))
                {
                    if (!pose.is_directory())
                        continue;

                    cout << "   "
                        << pose.path().filename().string()
                        << endl;

                    poseCount++;
                }

                cout << "   Total poses: "
                    << poseCount
                    << endl;
                count++;
            }
 
            if (count == 0)
                cout << "  (Not available sessions)\n";
            else
                cout << "\nTotal: " << count << " sessions\n";
        }
        else if (key == 'q')
        {
            break;
        }
    }
 
    cap.release();
    destroyAllWindows();
}
 







