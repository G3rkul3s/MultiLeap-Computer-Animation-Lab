#include "funcs.h"

// function to calculate absolute difference between uint32_t
uint32_t abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

// align sensors through time
map<uint32_t, int> getTimeAlignedIndexes(const devices_data_t &data, const map<uint32_t, int> &start_indexes, int current_frame)
{
    uint32_t max_timestamp = 0;
    map<uint32_t, bool> time_aligned;
    map<uint32_t, int> new_start_indexes;
    // initialize with old indexes
    for (const auto &sensor : data)
    {
        new_start_indexes[sensor.first] = start_indexes.at(sensor.first);
        time_aligned[sensor.first] = false;
        max_timestamp = max(max_timestamp, sensor.second.second.at(start_indexes.at(sensor.first) + current_frame).timestamp);
    }
    while (any_of(time_aligned.begin(), time_aligned.end(), [](const pair<uint32_t, bool> &entry)
                  { return !entry.second; }))
    {
        for (const auto &sensor : data)
        {
            if (!time_aligned[sensor.first])
            {
                int idx = new_start_indexes[sensor.first] + current_frame;
                if (sensor.second.second[idx].timestamp < max_timestamp)
                {
                    if (abs_diff(max_timestamp, sensor.second.second[idx].timestamp) <
                        abs_diff(max_timestamp, sensor.second.second[idx + 1].timestamp))
                    {
                        time_aligned[sensor.first] = true;
                    }
                    else
                    {
                        new_start_indexes[sensor.first]++;
                    }
                }
                else
                {
                    time_aligned[sensor.first] = true;
                }
            }
        }
    }
    return new_start_indexes;
}

// confidence computation
float computeConfidence(const AnnotationAtTimestamp &annotation, const CalibrationStatus &calibration, gotHandsState hand)
{
    float confidence;
    float distance_confidence;
    float angle_confidence;
    Eigen::Vector3f sensor_pos_new = calibration.sensor_position;
    Eigen::Vector3f palm_cemter = hand == gotHandsState::rightHand ? annotation.center_right_hand : annotation.center_left_hand;
    Eigen::Vector3f normal = hand == gotHandsState::rightHand ? annotation.normal_right_hand : annotation.normal_left_hand;
    // the median value of the vertical tracking range (10–80 centimetres above the sensor)
    float h = 400.0; // in mm
    // the distance of the palm centre and the centre of the sensor
    float distance;
    // angle between the normal of the palm and the normal of the sensor in radians
    float a;
    // the maximum tracking distance of the sensor (1 metre for the Leap Motion sensor)
    float m = 1.0; // in m

    // calculate distance confidence
    sensor_pos_new[1] += h;
    distance = vecm::distanceBetweenVectors(palm_cemter, sensor_pos_new);
    distance_confidence = m - (0.0005 * distance);

    // calculate angle confidence
    a = vecm::angleBetweenVectors(normal, calibration.sensor_normal);
    angle_confidence = (0.2837 * pow(a, 2)) - (0.89127 * a) + 1.0;

    confidence = distance_confidence * angle_confidence;
    return confidence;
}

// construct fused hand
// WARNING: frame_data is not const, so that left and right hand could be swapped TODO: remove swap
fused_hand_data getFusedHand(const map<uint32_t, hands_annot_data> &frame_data, const map<uint32_t, CalibrationStatus> &data_status)
{
    fused_hand_data fused_hand;
    map<uint32_t, float> confidence;
    float hand_deviation = 0;
    float chirality_tolerance = 30.0;
    int hand_in_view_count = 0; // how many calibrated sensors sees the hand
    uint32_t joint_num = frame_data.begin()->second.first.second.size();
    // compute tracking data confidence for each calibrated sensor
    // and confim hand chirality
    for (auto &current : frame_data)
    {
        if (!data_status.at(current.first).calibrated)
        {
            continue;
        }
        int left_cnt = current.second.second.state == gotHandsState::leftHand ? 1 : 0;
        int right_cnt = current.second.second.state == gotHandsState::rightHand ? 1 : 0;
        float max_conf_right = 0.0;
        float max_conf_left = 0.0;
        for (const auto &sensor : frame_data)
        {
            if (!data_status.at(sensor.first).calibrated || sensor.first == current.first)
            {
                continue;
            }
            if (sensor.second.second.state == gotHandsState::rightHand)
            {
                if (current.second.second.state == gotHandsState::rightHand &&
                    vecm::distanceBetweenVectors(data_status.at(sensor.first).translation_vector + data_status.at(sensor.first).rotation_matrix * sensor.second.second.center_right_hand,
                                                 data_status.at(current.first).translation_vector + data_status.at(current.first).rotation_matrix * current.second.second.center_right_hand) < chirality_tolerance)
                {
                    right_cnt++;
                    max_conf_right = std::max(max_conf_right, computeConfidence(sensor.second.second, data_status.at(sensor.first), sensor.second.second.state));
                }
                else if (current.second.second.state == gotHandsState::leftHand &&
                         vecm::distanceBetweenVectors(data_status.at(sensor.first).translation_vector + data_status.at(sensor.first).rotation_matrix * sensor.second.second.center_right_hand,
                                                      data_status.at(current.first).translation_vector + data_status.at(current.first).rotation_matrix * current.second.second.center_left_hand) < chirality_tolerance)
                {
                    right_cnt++;
                    max_conf_right = std::max(max_conf_right, computeConfidence(sensor.second.second, data_status.at(sensor.first), sensor.second.second.state));
                }
            }
            else if (sensor.second.second.state == gotHandsState::leftHand)
            {
                if (current.second.second.state == gotHandsState::rightHand &&
                    vecm::distanceBetweenVectors(data_status.at(sensor.first).translation_vector + data_status.at(sensor.first).rotation_matrix * sensor.second.second.center_left_hand,
                                                 data_status.at(current.first).translation_vector + data_status.at(current.first).rotation_matrix * current.second.second.center_right_hand) < chirality_tolerance)
                {
                    left_cnt++;
                    max_conf_left = std::max(max_conf_left, computeConfidence(sensor.second.second, data_status.at(sensor.first), sensor.second.second.state));
                }
                else if (current.second.second.state == gotHandsState::leftHand &&
                         vecm::distanceBetweenVectors(data_status.at(sensor.first).translation_vector + data_status.at(sensor.first).rotation_matrix * sensor.second.second.center_left_hand,
                                                      data_status.at(current.first).translation_vector + data_status.at(current.first).rotation_matrix * current.second.second.center_left_hand) < chirality_tolerance)
                {
                    left_cnt++;
                    max_conf_left = std::max(max_conf_left, computeConfidence(sensor.second.second, data_status.at(sensor.first), sensor.second.second.state));
                }
            }
        }
        // if the hand chirality agrees with the majority of the sensors
        if (right_cnt > left_cnt && current.second.second.state == gotHandsState::rightHand || left_cnt > right_cnt && current.second.second.state == gotHandsState::leftHand)
        {
            confidence[current.first] = computeConfidence(current.second.second, data_status.at(current.first), current.second.second.state);
            hand_in_view_count++;
        }
        // if the hand chirality disagrees with the majority of the sensors
        else if (right_cnt > left_cnt && current.second.second.state == gotHandsState::leftHand || left_cnt > right_cnt && current.second.second.state == gotHandsState::rightHand)
        {
            confidence[current.first] = 0.8 * computeConfidence(current.second.second, data_status.at(current.first), current.second.second.state);
            // current.second.second.state = current.second.second.state == gotHandsState::rightHand ? gotHandsState::leftHand : gotHandsState::rightHand;
            // std::swap(current.second.first.first, current.second.first.second);
            hand_in_view_count++;
        }
        // if there is no majority opinion on the chirality of the hand
        else if (right_cnt > 0 && right_cnt == left_cnt)
        {
            float conf = computeConfidence(current.second.second, data_status.at(current.first), current.second.second.state);
            if (current.second.second.state == gotHandsState::rightHand && conf >= max_conf_right || current.second.second.state == gotHandsState::leftHand && conf >= max_conf_left)
            {
                confidence[current.first] = conf;
                hand_in_view_count++;
            }
            else
            {
                confidence[current.first] = 0.8 * conf;
                // current.second.second.state = current.second.second.state == gotHandsState::rightHand ? gotHandsState::leftHand : gotHandsState::rightHand;
                // std::swap(current.second.first.first, current.second.first.second);
                hand_in_view_count++;
            }
        }
        // both or no hands detected case:
        else
        {
            confidence[current.first] = 0.0;
        }
    }
    // if no hands were detected (after calibration)
    if (hand_in_view_count == 0)
    {
        fused_hand.first.sensor_id.reset();
        fused_hand.first.confidence = 0.0;
        fused_hand.first.hand_deviation = 0.0;
        fused_hand.first.timestamp = frame_data.begin()->second.second.timestamp;
        vector<float> point = {0, 0, 0};
        for (int i = 0; i < joint_num; ++i)
        {
            fused_hand.second.first.push_back(point);
            fused_hand.second.second.push_back(point);
        }
        return fused_hand;
    }
    float confidence_sum = std::accumulate(confidence.begin(), confidence.end(), 0.0f,
                                           [](float acc, const std::pair<uint32_t, float> &p)
                                           {
                                               return acc + p.second;
                                           });
    // select the hand tracking data with the highest confidence
    uint32_t max_ind = std::max_element(confidence.begin(), confidence.end(),
                                        [](const auto &a, const auto &b)
                                        {
                                            return a.second < b.second;
                                        })
                           ->first;
    fused_hand.first.sensor_id = max_ind;
    fused_hand.first.confidence = confidence_sum / confidence.size();
    fused_hand.first.timestamp = frame_data.at(max_ind).second.timestamp;
    // apply translation and rotation to get hand postion from the perspective of the reference sensor
    for (int i = 0; i < joint_num; ++i)
    {
        vector<float> avrg_point = {0, 0, 0};
        Eigen::Map<Eigen::Vector3f> point_a(avrg_point.data());
        for (const auto &sensor : frame_data)
        {
            if (data_status.at(sensor.first).calibrated)
            {
                vector<float> point = {0, 0, 0};
                // manually take first 3 elements, because frame data has unnecessary quaternion information
                if (sensor.second.second.state == gotHandsState::rightHand)
                {
                    point.assign(sensor.second.first.second.at(i).begin(), sensor.second.first.second.at(i).begin() + 3);
                }
                else if (sensor.second.second.state == gotHandsState::leftHand)
                {
                    point.assign(sensor.second.first.first.at(i).begin(), sensor.second.first.first.at(i).begin() + 3);
                }
                // map to Eigen vector to perform vector operations
                Eigen::Map<Eigen::Vector3f> point_e(point.data());
                // perform translation and rotation
                point_e = data_status.at(sensor.first).translation_vector + data_status.at(sensor.first).rotation_matrix * point_e;
                // highest confidence mode
                if (sensor.first == max_ind)
                {
                    fused_hand.second.first.push_back(point);
                }
                // average mode
                point_a += point_e * confidence.at(sensor.first);
            }
        }
        if (confidence_sum != 0.0f)
        {
            point_a /= confidence_sum;
        }
        fused_hand.second.second.push_back(avrg_point);
        // calculate hand deviation
        for (const auto &sensor : frame_data)
        {
            if (data_status.at(sensor.first).calibrated)
            {
                vector<float> point = {0, 0, 0};
                if (sensor.second.second.state == gotHandsState::rightHand)
                {
                    point.assign(sensor.second.first.second.at(i).begin(), sensor.second.first.second.at(i).begin() + 3);
                }
                else if (sensor.second.second.state == gotHandsState::leftHand)
                {
                    point.assign(sensor.second.first.first.at(i).begin(), sensor.second.first.first.at(i).begin() + 3);
                }
                Eigen::Map<Eigen::Vector3f> point_e(point.data());
                hand_deviation += vecm::distanceBetweenVectors(point_e, point_a) * confidence.at(sensor.first);
            }
        }
    }
    fused_hand.first.hand_deviation = hand_deviation / joint_num;
    if (confidence_sum != 0.0f)
    {
        fused_hand.first.hand_deviation /= confidence_sum;
    }
    if (hand_in_view_count != 0)
    {
        fused_hand.first.hand_deviation /= hand_in_view_count;
    }
    return fused_hand;
}

// calculate optimal rotation and translation
// Kabsch algorithm
void calculateOptimalTranslationAndRotation(CalibrationStatus &data_status)
{
    int finger_points[20] = {2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25}; // all idexes of right hand fingers joints
    size_t rows = data_status.samples.size() * 20;
    // represent as P and Q
    Eigen::MatrixXf matP(rows, 3);
    Eigen::MatrixXf matQ(rows, 3);
    for (size_t k = 0; k < data_status.samples.size(); ++k)
    {
        for (size_t i = 0; i < 20; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                matP(i + k * 20, j) = data_status.fused.at(k).second.second.at(finger_points[i]).at(j);
                matQ(i + k * 20, j) = data_status.samples[k].first.second[finger_points[i]][j];
            }
        }
    }
    // compute translation
    //  find centroids
    Eigen::Vector3f cP = matP.colwise().sum() / rows;
    Eigen::Vector3f cQ = matQ.colwise().sum() / rows;
    //  center both sets
    matP = matP.rowwise() - cP.transpose();
    matQ = matQ.rowwise() - cQ.transpose();
    // compute the covariance matrix H
    Eigen::Matrix3f H = matP.transpose() * matQ;
    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    float d = svd.matrixU().determinant() * svd.matrixV().determinant();
    // compute rotation
    Eigen::Matrix3f matR = Eigen::Matrix3f::Identity();
    //  prevent reflection
    matR(2, 2) = d;

    data_status.rotation_matrix = svd.matrixU() * matR * svd.matrixV().transpose();
    data_status.translation_vector = cP - data_status.rotation_matrix * cQ;
}