//
// Created by Sasha on 07.11.2024.
// Edited by Nikita
//

#pragma once

#include "pch.h"

using namespace std;

// Enum class that indicates for every measurement how many hands were detected (or
// which hand were detected if sensor detected only 1 hand)
enum class gotHandsState
{
    bothHands,
    leftHand,
    rightHand,
    noHands
};

struct Point3D
{
    float x, y, z;

    // Constructor to initialize with zeros
    Point3D() : x(0.0), y(0.0), z(0.0) {}

    // Constructor to initialize with float[3] values
    Point3D(const float values[3]) : x(values[0]), y(values[1]), z(values[2]) {}
};

struct NormalVector
{
    float x, y, z;

    // Constructor to initialize with zeros
    NormalVector() : x(0.0), y(0.0), z(0.0) {}

    // Constructor to initialize with float[3] values
    NormalVector(const float values[3]) : x(values[0]), y(values[1]), z(values[2]) {}
};

struct AnnotationAtTimestamp
{
    uint32_t timestamp;
    gotHandsState state;
    Eigen::Vector3f center_left_hand;  // Point3D
    Eigen::Vector3f center_right_hand; // Point3D
    float confidence_left_hand;
    float confidence_right_hand;
    Eigen::Vector3f normal_left_hand;  // NormalVector
    Eigen::Vector3f normal_right_hand; // NormalVector
};

typedef vector<vector<vector<float>>> hand_data_t;
typedef map<uint32_t, pair<pair<hand_data_t, hand_data_t>, vector<AnnotationAtTimestamp>>> devices_data_t;

typedef vector<vector<float>> hand_data_sim;
typedef pair<pair<hand_data_sim, hand_data_sim>, AnnotationAtTimestamp> hands_annot_data;
typedef pair<uint32_t, hands_annot_data> devices_data_sim;

struct AnnotationFusedHand
{
    uint32_t sensor_id;
    uint32_t timestamp;
    float confidence_right_hand;
    float hand_deviation;
};

// first hand_data_sim is the highest confidence fusing mode
// second hand_data_sim is theaverage fusing mode
typedef pair<AnnotationFusedHand, pair<hand_data_sim, hand_data_sim>> fused_hand_data;

struct CalibrationStatus
{
    bool calibrated = false;
    uint32_t discarded_frames = 0;
    float minimal_distance = 50.0;
    vector<hands_annot_data> samples;
    vector<fused_hand_data> fused;
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation_vector = Eigen::Vector3f::Zero();
    Eigen::Vector3f sensor_position = Eigen::Vector3f::Zero();
    Eigen::Vector3f sensor_normal{0.0, 1.0, 0.0};
};