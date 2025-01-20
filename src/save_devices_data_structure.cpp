//
// Created by Sasha on 08.11.2024.
// Edited by Nikita
//

#include "save_diveces_data_structure.h"

using json = nlohmann::json;

Eigen::Vector3f convertToEigenVector(const Point3D &point)
{
    return Eigen::Vector3f{point.x, point.y, point.z};
}

Eigen::Vector3f convertToEigenVector(const NormalVector &point)
{
    return Eigen::Vector3f{point.x, point.y, point.z};
}

// Function to write a vector of vectors of vectors of floats to a binary file
void write_hand_data(std::ofstream &out, const hand_data_t &hand_data)
{
    uint32_t outer_size = hand_data.size();
    out.write(reinterpret_cast<const char *>(&outer_size), sizeof(outer_size));
    for (const auto &mid_vec : hand_data)
    {
        uint32_t mid_size = mid_vec.size();
        out.write(reinterpret_cast<const char *>(&mid_size), sizeof(mid_size));
        for (const auto &inner_vec : mid_vec)
        {
            uint32_t inner_size = inner_vec.size();
            out.write(reinterpret_cast<const char *>(&inner_size), sizeof(inner_size));
            for (float value : inner_vec)
            {
                out.write(reinterpret_cast<const char *>(&value), sizeof(value));
            }
        }
    }
}

// Function to read a vector of vectors of vectors of floats from a binary file
hand_data_t read_hand_data(std::ifstream &in)
{
    uint32_t outer_size;
    in.read(reinterpret_cast<char *>(&outer_size), sizeof(outer_size));
    hand_data_t hand_data(outer_size);

    for (auto &mid_vec : hand_data)
    {
        uint32_t mid_size;
        in.read(reinterpret_cast<char *>(&mid_size), sizeof(mid_size));
        mid_vec.resize(mid_size);
        for (auto &inner_vec : mid_vec)
        {
            uint32_t inner_size;
            in.read(reinterpret_cast<char *>(&inner_size), sizeof(inner_size));
            inner_vec.resize(inner_size);
            for (float &value : inner_vec)
            {
                in.read(reinterpret_cast<char *>(&value), sizeof(value));
            }
        }
    }
    return hand_data;
}

// Function to serialize devices_data_t to a binary file
// NOTE: might not work correctly, was not tested with a new annotation data structure
void save_devices_data(const devices_data_t &data,
                       const std::map<uint32_t, std::pair<uint32_t, std::string>> &registered_devices,
                       const std::string &filename)
{
    std::ofstream out(filename, std::ios::binary);
    if (!out)
    {
        throw std::runtime_error("Cannot open file for writing");
    }

    // Save registered_devices
    uint32_t reg_size = registered_devices.size();
    out.write(reinterpret_cast<const char *>(&reg_size), sizeof(reg_size));

    for (const auto &[device_id, device_info] : registered_devices)
    {
        out.write(reinterpret_cast<const char *>(&device_id), sizeof(device_id));
        out.write(reinterpret_cast<const char *>(&device_info.first), sizeof(device_info.first));

        uint32_t serial_length = device_info.second.size();
        out.write(reinterpret_cast<const char *>(&serial_length), sizeof(serial_length));
        out.write(device_info.second.data(), serial_length);
    }

    // Save devices_data
    uint32_t map_size = data.size();
    out.write(reinterpret_cast<const char *>(&map_size), sizeof(map_size));

    for (const auto &kv : data)
    {
        // Device ID
        out.write(reinterpret_cast<const char *>(&kv.first), sizeof(kv.first));

        // Hand Data
        write_hand_data(out, kv.second.first.first);  // Left hand
        write_hand_data(out, kv.second.first.second); // Right hand

        // Annotations
        uint32_t vec_size = kv.second.second.size();
        out.write(reinterpret_cast<const char *>(&vec_size), sizeof(vec_size));

        for (const auto &annotation : kv.second.second)
        {
            out.write(reinterpret_cast<const char *>(&annotation.timestamp), sizeof(annotation.timestamp));
            int gotHandsStateValue = static_cast<int>(annotation.state);
            out.write(reinterpret_cast<const char *>(&gotHandsStateValue), sizeof(gotHandsStateValue));
            out.write(reinterpret_cast<const char *>(&annotation.center_left_hand), sizeof(annotation.center_left_hand));
            out.write(reinterpret_cast<const char *>(&annotation.center_right_hand), sizeof(annotation.center_right_hand));
            out.write(reinterpret_cast<const char *>(&annotation.confidence_left_hand), sizeof(annotation.confidence_left_hand));
            out.write(reinterpret_cast<const char *>(&annotation.confidence_right_hand), sizeof(annotation.confidence_right_hand));
            out.write(reinterpret_cast<const char *>(&annotation.normal_left_hand), sizeof(annotation.normal_left_hand));
            out.write(reinterpret_cast<const char *>(&annotation.normal_right_hand), sizeof(annotation.normal_right_hand));
        }
    }
    out.close();
}

// Function to convert AnnotationFusedHand to JSON
json annotation_to_json(const AnnotationFusedHand &annotation)
{
    return {
        {"sensor_id", annotation.sensor_id},
        {"timestamp", annotation.timestamp},
        {"confidence_right_hand", annotation.confidence_right_hand}};
}

// Function to convert hand_data_sim to JSON
json hand_data_sim_to_json(const hand_data_sim &data)
{
    json j_data = json::array();
    for (const auto &row : data)
    {
        j_data.push_back(row);
    }
    return j_data;
}

// Function to convert fused_hand_data to JSON
json fused_hand_data_to_json(const fused_hand_data &data)
{
    return {
        {"annotation", annotation_to_json(data.first)},
        {"right_hand", hand_data_sim_to_json(data.second)}};
}

// Function to convert sensor data to JSON
nlohmann::ordered_json sensor_data_to_json(const std::pair<const uint32_t, CalibrationStatus> &sensor)
{
    nlohmann::ordered_json jsonSensor;
    jsonSensor["sensor_id"] = sensor.first;
    jsonSensor["position"] = sensor.second.sensor_position;
    jsonSensor["normal"] = sensor.second.sensor_normal;
    return jsonSensor;
}

// Main function to write a vector<fused_hand_data> into a JSON file
void write_hand_to_json_file(const vector<fused_hand_data> &data, const map<uint32_t, CalibrationStatus> &data_status, const string &filename)
{
    nlohmann::ordered_json j_sensor = nlohmann::ordered_json::array();
    json j_frame = json::array();
    for (const auto &sensor : data_status)
    {
        j_sensor.push_back(sensor_data_to_json(sensor));
    }
    for (const auto &item : data)
    {
        j_frame.push_back(fused_hand_data_to_json(item));
    }
    nlohmann::ordered_json jsonData;
    jsonData["sensor_data"] = j_sensor;
    jsonData["frame_data"] = j_frame;
    // Write JSON to file
    ofstream file(filename);
    if (file.is_open())
    {
        file << jsonData.dump(4); // Pretty print with 4-space indentation
        file.close();
        cout << "Data successfully written to " << filename << endl;
    }
    else
    {
        cerr << "Error: Could not open file " << filename << " for writing." << endl;
    }
}

// Function to deserialize devices_data_t from a binary file
void load_devices_data(
    map<uint32_t, pair<uint32_t, string>> &registered_devices,
    devices_data_t &data,
    const std::string &filename)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in)
    {
        throw std::runtime_error("Cannot open file for reading");
    }

    // devices_data_t data;

    // Load registered_devices
    uint32_t reg_size;
    in.read(reinterpret_cast<char *>(&reg_size), sizeof(reg_size));

    for (uint32_t i = 0; i < reg_size; ++i)
    {
        uint32_t device_id, sample_count;
        in.read(reinterpret_cast<char *>(&device_id), sizeof(device_id));
        in.read(reinterpret_cast<char *>(&sample_count), sizeof(sample_count));

        uint32_t serial_length;
        in.read(reinterpret_cast<char *>(&serial_length), sizeof(serial_length));

        std::string serial_number(serial_length, '\0');
        in.read(&serial_number[0], serial_length);

        registered_devices[device_id] = std::make_pair(sample_count, serial_number);
    }

    // Load devices_data
    uint32_t map_size;
    in.read(reinterpret_cast<char *>(&map_size), sizeof(map_size));

    for (uint32_t i = 0; i < map_size; ++i)
    {
        uint32_t key;
        in.read(reinterpret_cast<char *>(&key), sizeof(key));

        hand_data_t hand_data1 = read_hand_data(in);
        hand_data_t hand_data2 = read_hand_data(in);

        uint32_t vec_size;
        in.read(reinterpret_cast<char *>(&vec_size), sizeof(vec_size));
        std::vector<AnnotationAtTimestamp> annotations(vec_size);

        for (uint32_t j = 0; j < vec_size; ++j)
        {
            AnnotationAtTimestamp annotation;
            Point3D pl;
            Point3D pr;
            NormalVector nl;
            NormalVector nr;
            in.read(reinterpret_cast<char *>(&annotation.timestamp), sizeof(annotation.timestamp));
            int gotHandsStateValue;
            in.read(reinterpret_cast<char *>(&gotHandsStateValue), sizeof(gotHandsStateValue));
            annotation.state = static_cast<gotHandsState>(gotHandsStateValue);
            in.read(reinterpret_cast<char *>(&pl), sizeof(Point3D));
            in.read(reinterpret_cast<char *>(&pr), sizeof(Point3D));
            in.read(reinterpret_cast<char *>(&annotation.confidence_left_hand), sizeof(annotation.confidence_left_hand));
            in.read(reinterpret_cast<char *>(&annotation.confidence_right_hand), sizeof(annotation.confidence_right_hand));
            in.read(reinterpret_cast<char *>(&nl), sizeof(NormalVector));
            in.read(reinterpret_cast<char *>(&nr), sizeof(NormalVector));
            annotation.center_left_hand = convertToEigenVector(pl);
            annotation.center_right_hand = convertToEigenVector(pr);
            annotation.normal_left_hand = convertToEigenVector(nl);
            annotation.normal_right_hand = convertToEigenVector(nr);
            annotations[j] = annotation;
        }
        data[key] = std::make_pair(std::make_pair(hand_data1, hand_data2), annotations);
    }

    in.close();
    // return data;
}
