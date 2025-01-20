#pragma once

#include "pch.h"
#include "typedefs.h"
#include "vecm.h"
#include "cursor_manips.h"
#include "save_diveces_data_structure.h"

uint32_t abs_diff(uint32_t a, uint32_t b);

map<uint32_t, int> getTimeAlignedIndexes(const devices_data_t &data, const map<uint32_t, int> &start_indexes, int current_frame);

float computeConfidence(const AnnotationAtTimestamp &annotation, const CalibrationStatus &calibration);

fused_hand_data getFusedHand(const map<uint32_t, hands_annot_data> &frame_data, const map<uint32_t, CalibrationStatus> &data_status);

void calculateOptimalTranslationAndRotation(CalibrationStatus &data_status);
