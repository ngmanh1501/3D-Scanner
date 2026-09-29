#pragma once
#include <string>
#include <vector>

// ================================================================
//  path_utils.h
//  ----------------------------------------------------------------
//  3 ham nay TRUOC DAY la "static" ben trong main.cpp (chi main.cpp
//  goi duoc). GUI (gui_panels.cpp) can cung logic de fill Combo/ListBox
//  object/pose/model — chuyen ra day de dung chung 1 noi, KHONG copy
//  logic lan 2. main.cpp chi can #include "path_utils.h" va bo phan
//  dinh nghia "static" cu cua 3 ham nay di.
// ================================================================

// Lay so o cuoi ten (sau dau '_' cuoi cung), dung de sort "pose_0",
// "pose_1", ..., "pose_10" theo THU TU SO thay vi thu tu chuoi ky tu.
int extractNumber(const std::string& s);

// Liet ke cac thu muc con truc tiep trong "path", sort theo extractNumber()
// (dung cho danh sach object / pose, vd "Object_data/cup_01/pose_2").
std::vector<std::string> listDirectories(const std::string& path);

// Liet ke cac FILE (loc theo duoi "ext", vd ".yml") truc tiep trong
// "path", sort theo ten (alphabet) — dung cho danh sach reference model
// da build trong 1 thu muc nhom (xem PoseEstimationConfig::modelRoot).
std::vector<std::string> listFiles(const std::string& path, const std::string& ext);
