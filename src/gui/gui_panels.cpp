#include "gui_panels.h"
#include "path_utils.h"
#include "pipeline_io.h"

#include "camera_calib.h"
#include "projector_calib.h"
#include "object_capture.h"
#include "triangulator.h"
#include "multipose_triangulator.h"
#include "pose_estimator.h"
#include "app_config.h"

#include "imgui.h"
#include <cmath>
#include <sstream>
#include <cstring>
#include <filesystem>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

static AppConfig& cfg(GuiState& s) { return *reinterpret_cast<AppConfig*>(s.cfgPtr); }

// Khai bao truoc: dung trong DrawCapturePanel(), dinh nghia day du o duoi.
static void startCaptureJob(GuiState& state, const ObjectCaptureConfig& objCfg, const string& objectName);

// ================================================================
//  Sidebar
// ================================================================
static const char* kPanelNames[] = {
    "Camera Calib", "Projector Calib", "Capture", "Tri (1 pose)",
    "Tri (multi)", "Auto-calib turntable", "Build Model", "Locate", "Settings"
};

void DrawSidebar(GuiState& state)
{
    for (int i = 0; i < IM_ARRAYSIZE(kPanelNames); i++)
    {
        bool selected = (state.activePanel == i);
        if (ImGui::Selectable(kPanelNames[i], selected))
            state.activePanel = i;
    }
}

// ================================================================
//  Camera Calib
//  ----------------------------------------------------------------
//  Capture: mo 1 cua so OpenCV rieng (namedWindow "Phone Camera") va
//  block ben trong thread nen cho den khi nguoi dung bam ESC — dung
//  y nhu console cu, chi khac la KHONG chan luong render ImGui vi
//  chay tren BackgroundJob.
// ================================================================
void DrawCameraCalibPanel(GuiState& state)
{
    static char folder[256] = "Camera_calib";
    ImGui::Text("Thu muc anh chessboard:");
    ImGui::InputText("##camfolder", folder, sizeof(folder));

    bool busy = state.calibJob.isBusy();
    ImGui::BeginDisabled(busy);

    if (ImGui::Button("Chup anh (mo cua so camera, ESC de dong)"))
    {
        int url = cfg(state).app.camera_url;
        string f = folder;
        state.calibJob.start([url, f]() { captureImages(url, f); });
    }

    ImGui::SameLine();
    if (ImGui::Button("Tinh calib tu anh da chup"))
    {
        string f = folder;
        string out = cfg(state).app.cameraCalibFile;
        state.calibJob.start([f, out]() { calibrateAndSave(f, out); });
    }

    ImGui::EndDisabled();
    if (busy) { ImGui::SameLine(); ImGui::TextDisabled("Dang chay..."); }

    ImGui::Separator();
    ImGui::TextWrapped(
        "File dang dung: %s\n"
        "Quy trinh: chup >= 5 anh chessboard o nhieu goc/khoang cach khac nhau, "
        "roi bam 'Tinh calib'. Ket qua (RMS, cameraMatrix, distCoeffs) xem o "
        "panel Log ben duoi.",
        cfg(state).app.cameraCalibFile.c_str());
}

// ================================================================
//  Projector Calib
//  ----------------------------------------------------------------
//  GHI CHU: projector_calib.cpp KHONG nam trong bo file duoc xem xet
//  khi viet panel nay, nen chi wire duoc phan "recalibrateFromFolder()"
//  (doc lai anh da chup san trong 1 thu muc, decode + calib lai) — day
//  la ham CO SAN, an toan, khong doan mo. Phan "chup pose moi truc
//  tiep tu camera" (captureCalibPose trong 1 vong lap) theo DUNG mau
//  BackgroundJob nhu DrawCameraCalibPanel o tren — chi can dien vao
//  TODO ben duoi khi ban co noi dung projector_calib.cpp.
// ================================================================
void DrawProjectorCalibPanel(GuiState& state)
{
    static char folder[256] = "Projector_calib";
    ImGui::Text("Thu muc pose da chup (checkerboard qua Gray-code):");
    ImGui::InputText("##projfolder", folder, sizeof(folder));

    bool busy = state.calibJob.isBusy();
    ImGui::BeginDisabled(busy);

    // TODO: nut "Chup pose moi" — goi captureCalibPose(cap, patterns, projCfg,
    // root_path, pose_id, obj_valid, cam_valid, proj_valid) tren BackgroundJob,
    // cung mau nhu captureImages() o panel Camera Calib.
    ImGui::TextDisabled("[ Chup pose moi tu camera: can noi dung projector_calib.cpp ]");

    if (ImGui::Button("Doc lai anh da chup + tinh calib (recalibrateFromFolder)"))
    {
        Mat Kc, dist_c;
        if (!loadCameraCalib(cfg(state).app.cameraCalibFile, Kc, dist_c))
        {
            cout << "[ERROR] Can calib camera truoc (panel Camera Calib).\n";
        }
        else
        {
            ProjectorCalibConfig projCfg;
            projCfg.proj_w   = cfg(state).app.proj_w;
            projCfg.proj_h   = cfg(state).app.proj_h;
            projCfg.boardSize   = Size(cfg(state).projCalib.board_cols, cfg(state).projCalib.board_rows);
            projCfg.squareSize  = cfg(state).projCalib.squareSize_mm;
            projCfg.threshold   = cfg(state).projCalib.threshold;
            projCfg.patchRadius = cfg(state).projCalib.patchRadius;
            projCfg.minDecodePoints = cfg(state).projCalib.minDecodePoints;
            projCfg.minPoses    = cfg(state).projCalib.minPoses;
            projCfg.outputJson  = cfg(state).app.projectorCalibFile;

            string f = folder;
            state.calibJob.start([f, Kc, dist_c, projCfg]() mutable
            {
                recalibrateFromFolder(f, Kc, dist_c, projCfg);
            });
        }
    }

    ImGui::EndDisabled();
    if (busy) { ImGui::SameLine(); ImGui::TextDisabled("Dang chay..."); }
}

// ================================================================
//  Capture object
// ================================================================
void DrawCapturePanel(GuiState& state)
{
    ObjectCaptureConfig objCfg;
    objCfg.proj_w   = cfg(state).app.proj_w;
    objCfg.proj_h   = cfg(state).app.proj_h;
    objCfg.rootPath = cfg(state).app.objectDataRoot;

    vector<string> objects = listDirectories(objCfg.rootPath);

    ImGui::Text("Object hien co:");
    ImGui::BeginChild("objlist", ImVec2(0, 120), true);
    for (int i = 0; i < (int)objects.size(); i++)
    {
        bool sel = (state.selectedObject == i);
        if (ImGui::Selectable(objects[i].c_str(), sel))
            state.selectedObject = i;
    }
    ImGui::EndChild();

    ImGui::Separator();
    ImGui::InputText("Ten session moi", state.newSessionName, sizeof(state.newSessionName));

    bool busy = state.captureJob.isBusy();
    ImGui::BeginDisabled(busy);

    // Bam "Chup session MOI" dung state.newSessionName; "Chup them pose" dung
    // object dang chon trong danh sach — thay 2 phim 'n'/'c' cua console cu.
    if (ImGui::Button("Chup session MOI (mo cua so projector fullscreen)"))
    {
        string name = state.newSessionName;
        if (!name.empty())
            startCaptureJob(state, objCfg, name);
        else
            cout << "[WARN] Chua nhap ten session.\n";
    }

    ImGui::SameLine();
    bool hasSel = (state.selectedObject >= 0 && state.selectedObject < (int)objects.size());
    if (!hasSel) ImGui::BeginDisabled();
    if (ImGui::Button("Chup THEM pose cho object dang chon"))
        startCaptureJob(state, objCfg, objects[state.selectedObject]);
    if (!hasSel) ImGui::EndDisabled();

    ImGui::EndDisabled();
    if (busy) { ImGui::SameLine(); ImGui::TextDisabled("Dang chup..."); }

    // Popup xac nhan ghi de — thay cin >> confirm 'y'/'n' cua console cu.
    if (state.showOverwriteConfirm)
        ImGui::OpenPopup("Xac nhan ghi de");
    if (ImGui::BeginPopupModal("Xac nhan ghi de", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Session nay da ton tai. Ghi de anh cu?");
        if (ImGui::Button("Ghi de", ImVec2(120, 0)))
        {
            state.showOverwriteConfirm = false;
            if (state.onConfirmOverwrite) state.onConfirmOverwrite();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Huy", ImVec2(120, 0)))
        {
            state.showOverwriteConfirm = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// Helper dung chung cho 2 nut o tren — kiem tra pose_path co san chua, neu co
// thi mo popup xac nhan (thay cho confirmOverwriteConsole() ben console),
// neu chua thi chup luon.
static void startCaptureJob(GuiState& state, const ObjectCaptureConfig& objCfg, const string& objectName)
{
    string objectPath = objCfg.rootPath + "/" + objectName;
    int poseID = getNextPoseID(objectPath);
    string posePath = objectPath + "/pose_" + to_string(poseID);

    int cameraUrl = cfg(state).app.camera_url;   // doc TRUOC khi vao thread nen

    auto doCapture = [&state, objCfg, objectName, poseID, cameraUrl](bool overwriteOk)
    {
        // Gray-code patterns + VideoCapture duoc tao MOI lan chup (giong
        // runObjectCapture() cu) — don gian, it quan trong hieu nang vi
        // day la thao tac "bam 1 lan" chu khong phai vong lap real-time.
        state.captureJob.start([objCfg, objectName, poseID, overwriteOk, cameraUrl]()
        {
            auto graygen = structured_light::GrayCodePattern::create(objCfg.proj_w, objCfg.proj_h);
            vector<Mat> patterns;
            graygen->generate(patterns);

            VideoCapture cap(cameraUrl);
            if (!cap.isOpened()) { cout << "[ERROR] Khong mo duoc camera\n"; return; }

            namedWindow("projector", WINDOW_NORMAL);
            setWindowProperty("projector", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

            captureObjectSession(cap, patterns, objCfg, objectName, poseID, overwriteOk);

            destroyWindow("projector");
            cap.release();
        });
    };

    if (fs::exists(posePath))
    {
        state.onConfirmOverwrite = [doCapture]() mutable { doCapture(true); };
        state.showOverwriteConfirm = true;
    }
    else
    {
        doCapture(false);
    }
}

// ================================================================
//  Tri (1 pose) — mirror cmdTri() trong main.cpp cu, chi khac cach
//  chon object/pose (combo thay pickFromList).
// ================================================================
void DrawTriPanel(GuiState& state)
{
    ObjectCaptureConfig objCfg;
    objCfg.rootPath = cfg(state).app.objectDataRoot;

    vector<string> objects = listDirectories(objCfg.rootPath);
    ImGui::Text("Object:");
    for (int i = 0; i < (int)objects.size(); i++)
        if (ImGui::Selectable(objects[i].c_str(), state.selectedObject == i))
        { state.selectedObject = i; state.selectedPose = -1; }

    vector<string> poses;
    if (state.selectedObject >= 0 && state.selectedObject < (int)objects.size())
    {
        string objectFolder = objCfg.rootPath + "/" + objects[state.selectedObject];
        poses = listDirectories(objectFolder);
        ImGui::Separator();
        ImGui::Text("Pose:");
        for (int i = 0; i < (int)poses.size(); i++)
            if (ImGui::Selectable(poses[i].c_str(), state.selectedPose == i))
                state.selectedPose = i;
    }

    bool canRun = state.selectedObject >= 0 && state.selectedPose >= 0 && !state.triJob.isBusy();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Reconstruct (1 pose) -> xuat .ply"))
    {
        AppConfig cfgCopy = cfg(state);   // capture theo gia tri, thread rieng
        string objectFolder = objCfg.rootPath + "/" + objects[state.selectedObject];
        string folder = objectFolder + "/" + poses[state.selectedPose];
        string objName = objects[state.selectedObject];
        string poseName = poses[state.selectedPose];

        state.triJob.start([cfgCopy, folder, objName, poseName]()
        {
            Mat Kc, dist_c, Kp, dist_p, R, T;
            if (!loadAllCalib(cfgCopy.app, Kc, dist_c, Kp, dist_p, R, T)) return;

            const int BITS = (int)ceil(log2((double)max(cfgCopy.app.proj_w, cfgCopy.app.proj_h)));
            vector<Mat> patterns;
            if (!loadPatterns(folder, BITS * 4, patterns)) return;

            GraycodeTriangulator tri(Kc, dist_c, Kp, dist_p, R, T, cfgCopy.tri);
            vector<Point3D> cloud = tri.reconstruct(patterns, cfgCopy.app.proj_w, cfgCopy.app.proj_h,
                                                      cfgCopy.decode, cfgCopy.filter);
            if (cloud.empty()) { cout << "[ERROR] Point cloud empty!\n"; return; }

            string outFile = "pointcloud_" + objName + "_" + poseName + ".ply";
            tri.savePLY(cloud, outFile);
            cout << "[DONE] Saved " << outFile << "\n";
        });
    }
    ImGui::EndDisabled();
    if (state.triJob.isBusy()) { ImGui::SameLine(); ImGui::TextDisabled("Dang reconstruct..."); }
}

// ================================================================
//  Tri (multi-pose) — mirror cmdTriMulti()
// ================================================================
void DrawTriMultiPanel(GuiState& state)
{
    ObjectCaptureConfig objCfg;
    objCfg.rootPath = cfg(state).app.objectDataRoot;
    vector<string> objects = listDirectories(objCfg.rootPath);

    ImGui::Text("Object (dung TOAN BO pose cua object nay):");
    for (int i = 0; i < (int)objects.size(); i++)
        if (ImGui::Selectable(objects[i].c_str(), state.selectedObject == i))
            state.selectedObject = i;

    bool canRun = state.selectedObject >= 0 && !state.triJob.isBusy();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Reconstruct + ICP merge (multi-pose) -> xuat .ply"))
    {
        AppConfig cfgCopy = cfg(state);
        string objName = objects[state.selectedObject];
        string objectFolder = objCfg.rootPath + "/" + objName;
        vector<string> poses = listDirectories(objectFolder);

        state.triJob.start([cfgCopy, objectFolder, poses, objName]()
        {
            Mat Kc, dist_c, Kp, dist_p, R, T;
            if (!loadAllCalib(cfgCopy.app, Kc, dist_c, Kp, dist_p, R, T)) return;

            MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                                       cfgCopy.tri, cfgCopy.decode, cfgCopy.filter);
            vector<Point3D> cloud = tri.reconstructFolder(objectFolder, poses,
                                                            cfgCopy.app.proj_w, cfgCopy.app.proj_h);
            if (cloud.empty()) { cout << "[ERROR] Cloud empty!\n"; return; }

            string outFile = "pointcloud_multi_" + objName + ".ply";
            tri.savePLY(cloud, outFile);
            cout << "[DONE] Saved " << outFile << "\n";
        });
    }
    ImGui::EndDisabled();
    if (state.triJob.isBusy()) { ImGui::SameLine(); ImGui::TextDisabled("Dang xu ly..."); }
}

// ================================================================
//  Auto-calib turntable — mirror cmdAutoCalib()
// ================================================================
void DrawAutoCalibPanel(GuiState& state)
{
    ObjectCaptureConfig objCfg;
    objCfg.rootPath = cfg(state).app.objectDataRoot;
    vector<string> objects = listDirectories(objCfg.rootPath);

    ImGui::TextWrapped(
        "Tu do turntable_center (x,z) + step_deg + chieu quay tu du lieu da "
        "chup. KHONG tu ghi config.json — ket qua in ra panel Log, ban tu "
        "dan vao muc \"triangulator\" trong Settings/config.json.");

    ImGui::Text("Object:");
    for (int i = 0; i < (int)objects.size(); i++)
        if (ImGui::Selectable(objects[i].c_str(), state.selectedObject == i))
            state.selectedObject = i;

    bool canRun = state.selectedObject >= 0 && !state.triJob.isBusy();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Chay auto-calibrate"))
    {
        AppConfig cfgCopy = cfg(state);
        string objectFolder = objCfg.rootPath + "/" + objects[state.selectedObject];
        vector<string> poses = listDirectories(objectFolder);

        state.triJob.start([cfgCopy, objectFolder, poses]()
        {
            Mat Kc, dist_c, Kp, dist_p, R, T;
            if (!loadAllCalib(cfgCopy.app, Kc, dist_c, Kp, dist_p, R, T)) return;

            MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                                       cfgCopy.tri, cfgCopy.decode, cfgCopy.filter);
            TurntableCalibResult r = tri.autoCalibrateTurntable(objectFolder, poses,
                                                                  cfgCopy.app.proj_w, cfgCopy.app.proj_h);

            cout << "\n====== DAN VAO config.json, muc \"triangulator\" ======\n";
            cout << "    \"turntable_step_deg\": " << r.step_deg << ",\n";
            cout << "    \"turntable_sign\": " << r.sign << ",\n";
            cout << "    \"turntable_center_x\": " << r.cx << ",\n";
            cout << "    \"turntable_center_y\": " << r.cy << ",\n";
            cout << "    \"turntable_center_z\": " << r.cz << ",\n";
            cout << "Da luu debug_before_autocalib.ply va debug_after_autocalib.ply.\n";
        });
    }
    ImGui::EndDisabled();
    if (state.triJob.isBusy()) { ImGui::SameLine(); ImGui::TextDisabled("Dang tim..."); }
}

// ================================================================
//  Build Model (Huong B) — mirror cmdBuildModel()
// ================================================================
void DrawBuildModelPanel(GuiState& state)
{
    ObjectCaptureConfig objCfg;
    objCfg.rootPath = cfg(state).app.objectDataRoot;
    vector<string> objects = listDirectories(objCfg.rootPath);

    ImGui::Text("Object/session (1 MAT cua vat):");
    for (int i = 0; i < (int)objects.size(); i++)
        if (ImGui::Selectable(objects[i].c_str(), state.selectedObject == i))
            state.selectedObject = i;

    ImGui::Separator();
    ImGui::InputText("Ten nhom vat (de trong = dung ten object)", state.newGroupName, sizeof(state.newGroupName));

    bool canRun = state.selectedObject >= 0 && !state.modelJob.isBusy();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Build + luu reference model"))
    {
        AppConfig cfgCopy = cfg(state);
        string objName = objects[state.selectedObject];
        string objectFolder = objCfg.rootPath + "/" + objName;
        vector<string> poses = listDirectories(objectFolder);
        string groupName = string(state.newGroupName).empty() ? objName : string(state.newGroupName);

        state.modelJob.start([cfgCopy, objectFolder, poses, objName, groupName]()
        {
            Mat Kc, dist_c, Kp, dist_p, R, T;
            if (!loadAllCalib(cfgCopy.app, Kc, dist_c, Kp, dist_p, R, T)) return;

            MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                                       cfgCopy.tri, cfgCopy.decode, cfgCopy.filter);
            vector<Vec3f> normals;
            vector<Point3D> points = tri.reconstructFolder(objectFolder, poses,
                                                              cfgCopy.app.proj_w, cfgCopy.app.proj_h, &normals);
            if (points.empty()) { cout << "[ERROR] Cloud empty!\n"; return; }

            string groupFolder = cfgCopy.poseEst.modelRoot + "/" + groupName;
            fs::create_directories(groupFolder);

            ReferenceModel model; model.points = points; model.normals = normals;
            string modelFile = groupFolder + "/" + objName + ".yml";
            saveReferenceModel(modelFile, model);
        });
    }
    ImGui::EndDisabled();
    if (state.modelJob.isBusy()) { ImGui::SameLine(); ImGui::TextDisabled("Dang build..."); }
}

// ================================================================
//  Locate (Huong B) — mirror cmdLocate()
// ================================================================
void DrawLocatePanel(GuiState& state)
{
    string modelRoot = cfg(state).poseEst.modelRoot;
    vector<string> groups = listDirectories(modelRoot);

    ImGui::Text("Nhom vat (model) da build:");
    for (int i = 0; i < (int)groups.size(); i++)
        if (ImGui::Selectable(groups[i].c_str(), state.selectedGroup == i))
            state.selectedGroup = i;

    ObjectCaptureConfig objCfg;
    objCfg.rootPath = cfg(state).app.objectDataRoot;
    vector<string> objects = listDirectories(objCfg.rootPath);

    ImGui::Separator();
    ImGui::Text("Object/pose LIVE de dinh vi:");
    for (int i = 0; i < (int)objects.size(); i++)
        if (ImGui::Selectable(objects[i].c_str(), state.selectedObject == i))
        { state.selectedObject = i; state.selectedPose = -1; }

    vector<string> poses;
    if (state.selectedObject >= 0 && state.selectedObject < (int)objects.size())
    {
        poses = listDirectories(objCfg.rootPath + "/" + objects[state.selectedObject]);
        for (int i = 0; i < (int)poses.size(); i++)
            if (ImGui::Selectable(poses[i].c_str(), state.selectedPose == i))
                state.selectedPose = i;
    }

    bool canRun = state.selectedGroup >= 0 && state.selectedObject >= 0 &&
                  state.selectedPose >= 0 && !state.modelJob.isBusy();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Chay locate (thu qua het cac mat trong nhom)"))
    {
        AppConfig cfgCopy = cfg(state);
        string groupFolder = modelRoot + "/" + groups[state.selectedGroup];
        string liveFolder = objCfg.rootPath + "/" + objects[state.selectedObject] + "/" + poses[state.selectedPose];

        state.modelJob.start([cfgCopy, groupFolder, liveFolder]()
        {
            Mat Kc, dist_c, Kp, dist_p, R, T;
            if (!loadAllCalib(cfgCopy.app, Kc, dist_c, Kp, dist_p, R, T)) return;

            vector<string> modelFiles = listFiles(groupFolder, ".yml");
            if (modelFiles.empty()) { cout << "[ERROR] Nhom chua co model .yml nao.\n"; return; }

            const int BITS = (int)ceil(log2((double)max(cfgCopy.app.proj_w, cfgCopy.app.proj_h)));
            vector<Mat> patterns;
            if (!loadPatterns(liveFolder, BITS * 4, patterns)) return;

            GraycodeTriangulator liveTriObj(Kc, dist_c, Kp, dist_p, R, T, cfgCopy.tri);
            vector<Point3D> liveCloud = liveTriObj.reconstruct(patterns, cfgCopy.app.proj_w, cfgCopy.app.proj_h,
                                                                 cfgCopy.decode, cfgCopy.filter);
            if (liveCloud.empty()) { cout << "[ERROR] Live cloud empty!\n"; return; }

            MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                                       cfgCopy.tri, cfgCopy.decode, cfgCopy.filter);
            int camW = patterns[0].cols, camH = patterns[0].rows;
            vector<Vec3f> liveNormals = tri.computeNormals(liveCloud, camW, camH);

            ObjectPose best; string bestFace; bool foundValid = false;
            for (auto& mf : modelFiles)
            {
                ReferenceModel model;
                if (!loadReferenceModel(groupFolder + "/" + mf, model)) continue;

                ObjectPose pose = estimatePoseByRegistration(liveCloud, liveNormals, model,
                                                               tri, cfgCopy.tri, cfgCopy.poseEst);
                if (pose.valid && (!foundValid || pose.fitness < best.fitness))
                { best = pose; bestFace = mf; foundValid = true; }
            }

            if (!foundValid) { cout << "[LOCATE] Khong mat nao khop du tin cay.\n"; return; }

            Mat rvec; Rodrigues(best.R, rvec);
            double angleDeg = norm(rvec) * 180.0 / CV_PI;
            cout << "\n[LOCATE] Mat khop: " << bestFace << "  (fitness=" << best.fitness << " mm)\n";
            cout << "Vi tri (mm): (" << best.position.x << ", " << best.position.y << ", "
                 << best.position.z << ")  |  xoay ~" << angleDeg << " do\n";
        });
    }
    ImGui::EndDisabled();
    if (state.modelJob.isBusy()) { ImGui::SameLine(); ImGui::TextDisabled("Dang dinh vi..."); }
}

// ================================================================
//  Settings — xem/sua toan bo config.json qua ImGui, khong can Notepad
// ================================================================
void DrawSettingsPanel(GuiState& state)
{
    AppConfig& c = cfg(state);

    if (ImGui::CollapsingHeader("App / duong dan", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputInt("proj_w", &c.app.proj_w);
        ImGui::InputInt("proj_h", &c.app.proj_h);
        ImGui::InputInt("camera_url", &c.app.camera_url);
        char buf[256];
        strncpy(buf, c.app.objectDataRoot.c_str(), sizeof(buf)); buf[255] = 0;
        if (ImGui::InputText("objectDataRoot", buf, sizeof(buf))) c.app.objectDataRoot = buf;
    }

    if (ImGui::CollapsingHeader("Decode"))
    {
        ImGui::InputInt("bits", &c.decode.bits);
        ImGui::InputInt("threshold##decode", &c.decode.threshold);
        ImGui::Checkbox("useAdaptiveThreshold", &c.decode.useAdaptiveThreshold);
        ImGui::InputInt("maxLg", &c.decode.maxLg);
    }

    if (ImGui::CollapsingHeader("Filter"))
    {
        ImGui::InputFloat("reproj_max_err_px", &c.filter.reproj_max_err_px);
        ImGui::InputFloat("spatial_max_jump", &c.filter.spatial_max_jump);
        ImGui::InputFloat("iqr_scale", &c.filter.iqr_scale);
    }

    if (ImGui::CollapsingHeader("Triangulator / ICP / Turntable"))
    {
        ImGui::InputFloat("z_min", &c.tri.z_min);
        ImGui::InputFloat("z_max", &c.tri.z_max);
        ImGui::InputInt("icp_iter", &c.tri.icp_iter);
        ImGui::InputFloat("icp_dist_mm", &c.tri.icp_dist_mm);
        ImGui::InputFloat("icp_trim_ratio", &c.tri.icp_trim_ratio);
        ImGui::InputInt("turntable_poses", &c.tri.turntable_poses);
        ImGui::InputFloat("turntable_step_deg", &c.tri.turntable_step_deg);
        ImGui::InputInt("turntable_sign", &c.tri.turntable_sign);
        ImGui::InputFloat3("turntable_center (x,y,z)", c.tri.turntable_center);
    }

    if (ImGui::CollapsingHeader("Pose Estimation (Huong B)"))
    {
        ImGui::InputFloat("pose_coarse_angle_step_deg", &c.poseEst.pose_coarse_angle_step_deg);
        ImGui::InputInt("pose_min_live_points", &c.poseEst.pose_min_live_points);
        ImGui::InputDouble("pose_max_accept_fitness_mm", &c.poseEst.pose_max_accept_fitness_mm);
    }

    ImGui::Separator();
    if (ImGui::Button("Luu config.json")) AppConfig::save("config.json", c);
    ImGui::SameLine();
    if (ImGui::Button("Doc lai config.json")) { AppConfig::load("config.json", c); c.print(); }
}

// ================================================================
//  Status + Log panel (duoi cung man hinh)
// ================================================================
void DrawStatusAndLog(GuiState& state)
{
    bool anyBusy = state.calibJob.isBusy() || state.captureJob.isBusy() ||
                   state.triJob.isBusy() || state.modelJob.isBusy();
    if (anyBusy)
        ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "Dang chay 1 tac vu nen...");
    else
        ImGui::TextDisabled("San sang.");

    ImGui::SameLine(ImGui::GetWindowWidth() - 80);
    if (ImGui::Button("Clear log") && state.log) state.log->clear();

    ImGui::Separator();
    ImGui::BeginChild("logscroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    if (state.log)
    {
        std::string text = state.log->snapshot();
        ImGui::TextUnformatted(text.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
}
