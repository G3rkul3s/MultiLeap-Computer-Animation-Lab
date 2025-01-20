//
// Created by Sasha on 08.11.2024.
//

#pragma once

#include "pch.h"
#include "typedefs.h"

// Function to write a vector of vectors of vectors of floats to a binary file
void write_hand_data(std::ofstream &out, const hand_data_t &hand_data);

// Function to read a vector of vectors of vectors of floats from a binary file
hand_data_t read_hand_data(std::ifstream &in);

// Function to serialize devices_data_t to a binary file
void save_devices_data(const devices_data_t &data,
                       const std::map<uint32_t, std::pair<uint32_t, std::string>> &registered_devices,
                       const std::string &filename);

// Function to deserialize devices_data_t from a binary file
void load_devices_data(
    map<uint32_t, pair<uint32_t, string>> &registered_devices,
    devices_data_t &data,
    const std::string &filename);

void write_hand_to_json_file(const vector<fused_hand_data> &data, const map<uint32_t, CalibrationStatus> &data_status, const string &filename);