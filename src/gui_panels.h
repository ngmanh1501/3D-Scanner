#pragma once
#include "gui_common.h"

// ================================================================
//  Moi ham duoi day ve NOI DUNG cua 1 vung ImGui da duoc Begin() tu
//  gui_main.cpp (sidebar / panel phai / log) — khong tu goi
//  ImGui::Begin/End cho CUA SO CHINH, chi dung ImGui::BeginChild,
//  Text, Button, Combo... nhu binh thuong ben trong.
// ================================================================

// Sidebar trai: danh sach 9 buoc, cap nhat state.activePanel khi bam.
void DrawSidebar(GuiState& state);

// 1 trong 9 panel ben phai, tuy theo state.activePanel.
void DrawCameraCalibPanel(GuiState& state);
void DrawProjectorCalibPanel(GuiState& state);
void DrawCapturePanel(GuiState& state);
void DrawTriPanel(GuiState& state);
void DrawTriMultiPanel(GuiState& state);
void DrawAutoCalibPanel(GuiState& state);
void DrawBuildModelPanel(GuiState& state);
void DrawLocatePanel(GuiState& state);
void DrawSettingsPanel(GuiState& state);

// Thanh trang thai job nen + log console, ve o duoi cung man hinh.
void DrawStatusAndLog(GuiState& state);
