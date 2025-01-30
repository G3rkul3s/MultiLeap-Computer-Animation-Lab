#include "funcs.h"

// Shared resources
queue<devices_data_sim> dataQueue; // Queue for storing data
mutex mtx;                         // Mutex for protecting shared resources
condition_variable cvEmitter;      // Used by the receiver to signal the emitter
condition_variable cvReceiver;     // Used by the emitter to signal the receiver
atomic<bool> done(false);          // Atomic flag to signal end of work
atomic<bool> stopEmitting(false);  // Flag to signal emitter to stop

// simulating the dataflow from sensors
void emitter(const map<uint32_t, pair<uint32_t, string>> &registered_devices, const devices_data_t &data)
{
    // align sensor data in time
    map<uint32_t, int> start_ind;
    for (const auto &sensor : data)
    {
        start_ind[sensor.first] = 0;
    }
    start_ind = getTimeAlignedIndexes(data, start_ind, 0);
    auto max_ind = max_element(start_ind.begin(), start_ind.end(),
                               [](const auto &a, const auto &b)
                               {
                                   return a.second < b.second;
                               });
    auto min_frames = min_element(registered_devices.begin(), registered_devices.end(),
                                  [](const auto &a, const auto &b)
                                  {
                                      return a.second.first < b.second.first;
                                  });
    printf("Data stream simulation START\n");
    for (int i = 0; max_ind->second + i < min_frames->second.first; i++)
    {
        if (stopEmitting)
        {
            // std::cout << "Stop signal received. Stopping transmitting data.\n";
            break;
        }
        for (const auto &sensor : data)
        {
            devices_data_sim data_sim;
            data_sim.first = sensor.first;
            data_sim.second = make_pair(make_pair(sensor.second.first.first[start_ind.at(sensor.first) + i],
                                                  sensor.second.first.second[start_ind.at(sensor.first) + i]),
                                        sensor.second.second[start_ind.at(sensor.first) + i]);
            lock_guard<mutex> lock(mtx);
            dataQueue.push(data_sim); // Emit data
        }
        cvReceiver.notify_one(); // Notify the receiver thread

        // align sensor data in time every 20 frames
        if (i != 0 && i % 20 == 0 && max_ind->second + i + 1 < min_frames->second.first)
        {
            start_ind = getTimeAlignedIndexes(data, start_ind, i);
            max_ind = max_element(start_ind.begin(), start_ind.end(),
                                  [](const auto &a, const auto &b)
                                  {
                                      return a.second < b.second;
                                  });
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    // Signal that the emitter is done
    lock_guard<mutex> lock(mtx);
    done = true;
    cvReceiver.notify_all(); // Wake up the receiver to allow it to exit
    printf("Data stream simulation END\n");
}

void receiver(const map<uint32_t, pair<uint32_t, string>> &registered_devices, uint32_t reference_id, int sample_count, string output_file, int recording_start_frame, int recording_finish_frame)
{
    hideCursor();
    printf("Calibration START\n");
    // initiallize all necessary data structures
    int f_count = 0; // frame counter
    bool calibration_end = false;
    vector<fused_hand_data> fused_hand_vec;
    int num_sensors = registered_devices.size();
    map<uint32_t, CalibrationStatus> data_status;
    int num_digits = to_string(sample_count).length();
    for (const auto &device : registered_devices)
    {
        data_status[device.first];
        if (device.first != reference_id)
        {
            printf("Added %*d / %d calibration sample(s) for sensor %d\n", num_digits, 0, sample_count, device.first);
        }
    }
    COORD currentPos = getCursorPosition();
    data_status[reference_id].calibrated = true;
    map<uint32_t, hands_annot_data> frame_data;
    while (true)
    {
        unique_lock<mutex> lock(mtx);
        cvReceiver.wait(lock, [&]
                        { return dataQueue.size() >= num_sensors || done; }); // Wait for data or completion signal

        // Receive all available data
        while (!dataQueue.empty())
        {
            devices_data_sim data = dataQueue.front();
            frame_data[data.first] = data.second;
            dataQueue.pop();
        }
        // Exit loop if emitter has finished and no more data is left
        if (done && dataQueue.empty())
        {
            break;
        }
        cvEmitter.notify_one(); // Notify the emitter to emit the next item

        // stop the calibation when all sensers are calibrated
        if (!calibration_end && all_of(data_status.begin(), data_status.end(), [](const auto &status)
                                       { return status.second.calibrated; }))
        {
            // stopEmitting = true;
            // break;
            calibration_end = true;
            moveCursor(0, currentPos.Y - 1);
            printf("\nCalibration END on frame %d\n", f_count);
            // for (const auto &device : data_status)
            // {
            //     std::cout << "Sensor " << device.first << std::endl
            //               << "Position: (" << device.second.sensor_position.transpose() << ")" << std::endl
            //               << "Normal: (" << device.second.sensor_normal.transpose() << ")" << std::endl
            //               << std::endl;
            // }
            printf("Waiting for data transmission to end...\n");
            showCursor();
        }
        // record fused hand after the calibration is done
        if (calibration_end)
        {
            if (f_count >= recording_start_frame)
            {
                if (f_count > recording_finish_frame)
                {
                    stopEmitting = true;
                    continue;
                }
                fused_hand_vec.emplace_back(getFusedHand(frame_data, data_status));
            }
        }
        else
        {
            // Sample the data
            for (const auto &sensor : frame_data)
            {
                // skip already calibrated sensors
                // or skip frames where right arm is not detected by any calibrated sensors
                // or if both/no hands are detected for the sensor being calibrated
                if (data_status[sensor.first].calibrated ||
                    all_of(frame_data.begin(), frame_data.end(), [&](const auto &sensor)
                           { return (!data_status[sensor.first].calibrated || sensor.second.second.state != gotHandsState::rightHand &&
                                                                                  sensor.second.second.state != gotHandsState::leftHand); }) ||
                    frame_data[sensor.first].second.state != gotHandsState::rightHand &&
                        frame_data[sensor.first].second.state != gotHandsState::leftHand)
                {
                    continue;
                }
                // ensure samples diversity
                if (any_of(data_status[sensor.first].samples.begin(), data_status[sensor.first].samples.end(), [&](const auto &sample)
                           { return vecm::distanceBetweenVectors(sensor.second.second.state == gotHandsState::rightHand ? sample.second.center_right_hand : sample.second.center_left_hand,
                                                                 frame_data[sensor.first].second.center_right_hand) < data_status[sensor.first].minimal_distance; }))
                {
                    data_status[sensor.first].discarded_frames++;
                    data_status[sensor.first].minimal_distance = data_status[sensor.first].minimal_distance * (1.0 - ((1.0 / sample_count) * data_status[sensor.first].discarded_frames / 1000.0));
                }
                else
                {
                    data_status[sensor.first].samples.emplace_back(frame_data[sensor.first]);
                    data_status[sensor.first].fused.emplace_back(getFusedHand(frame_data, data_status));
                    moveCursor(0, currentPos.Y - registered_devices.size() +
                                      ((std::distance(registered_devices.begin(), registered_devices.find(reference_id)) > std::distance(registered_devices.begin(), registered_devices.find(sensor.first)))
                                           ? std::distance(registered_devices.begin(), registered_devices.find(sensor.first)) + 1
                                           : std::distance(registered_devices.begin(), registered_devices.find(sensor.first))));
                    printf("Added %*d / %d calibration sample(s) for sensor %d",
                           num_digits, data_status[sensor.first].samples.size(), sample_count, sensor.first);
                    // this_thread::sleep_for(std::chrono::seconds(2));
                }
                // when enough samples are collected
                if (data_status[sensor.first].samples.size() >= sample_count)
                {
                    calculateOptimalTranslationAndRotation(data_status[sensor.first]);
                    // find new sensor position and normal
                    data_status[sensor.first].sensor_position = data_status[sensor.first].sensor_position + data_status[sensor.first].translation_vector;
                    data_status[sensor.first].sensor_normal = data_status[sensor.first].rotation_matrix * data_status[sensor.first].sensor_normal;
                    data_status[sensor.first].calibrated = true;
                }
            }
        }
        f_count++;
        // clear current frame
        frame_data.clear();
    }
    write_hand_to_json_file(fused_hand_vec, data_status, output_file);
}

int main(int argc, char *argv[])
{
    int number_of_calibration_samples = 20;
    std::optional<uint32_t> reference_sensor_id;
    // int record_frames = 500;
    int recording_start_frame = 0;
    int recording_finish_frame = 0;
    string input_file = "../data/12_12_2024_binData_fusion_largeMotion_rightHand_outside_3.bin";
    string output_file = "../results/fused_hand.json";
    if (argc > 1)
    {
        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if (arg == "-n")
            {
                if (i + 1 < argc)
                {
                    number_of_calibration_samples = std::stoi(argv[++i]);
                    if (number_of_calibration_samples < 1)
                    {
                        std::cerr << "Error: number of calibration samples must be greater than 1.\n";
                        return 1;
                    }
                }
                else
                {
                    std::cerr << "Error: -s requires a value.\n";
                    return 1;
                }
            }
            else if (arg == "-i")
            {
                if (i + 1 < argc)
                {
                    input_file = argv[++i];
                }
                else
                {
                    std::cerr << "Error: -i requires a value.\n";
                    return 1;
                }
            }
            else if (arg == "-o")
            {
                if (i + 1 < argc)
                {
                    output_file = argv[++i];
                }
                else
                {
                    std::cerr << "Error: -o requires a value.\n";
                    return 1;
                }
            }
            // else if (arg == "-t")
            // {
            //     if (i + 1 < argc)
            //     {
            //         record_frames = std::stoi(argv[++i]);
            //         if (record_frames < 0)
            //         {
            //             std::cerr << "Error: number of frames to record must be positive.\n";
            //             return 1;
            //         }
            //     }
            //     else
            //     {
            //         std::cerr << "Error: -t requires a value.\n";
            //         return 1;
            //     }
            // }
            else if (arg == "-r")
            {
                if (i + 1 < argc)
                {
                    reference_sensor_id = std::stoi(argv[++i]);
                }
                else
                {
                    std::cerr << "Error: -r requires a value.\n";
                    return 1;
                }
            }
            else if (arg == "-s")
            {
                if (i + 1 < argc)
                {
                    recording_start_frame = std::stoi(argv[++i]);
                    if (recording_start_frame < 0)
                    {
                        std::cerr << "Error: first frame of the recording must be positive.\n";
                        return 1;
                    }
                }
                else
                {
                    std::cerr << "Error: -f requires a value.\n";
                    return 1;
                }
            }
            else if (arg == "-f")
            {
                if (i + 1 < argc)
                {
                    recording_finish_frame = std::stoi(argv[++i]);
                    if (recording_finish_frame < 0)
                    {
                        std::cerr << "Error: first frame of the recording must be positive.\n";
                        return 1;
                    }
                    else if (recording_finish_frame < recording_start_frame)
                    {
                        std::cerr << "Error: recording finish frame must be after the starting frame.\n";
                        return 1;
                    }
                }
                else
                {
                    std::cerr << "Error: -f requires a value.\n";
                    return 1;
                }
            }
            else
            {
                std::cerr << "Unknown option: " << arg << std::endl;
                std::cerr << "Usage: .\\Lab [-n <number_of_calibration_samples>] [-i <input_file>] [-o <output_file>] [-r <reference_sensor>] [-s <recording_start_frame>] [-f <recording_finish_frame>]\n";
                return 1;
            }
        }
    }
    // read recorded data
    map<uint32_t, pair<uint32_t, string>> registered_devices;
    devices_data_t data;
    load_devices_data(registered_devices, data, input_file);
    if (!reference_sensor_id.has_value())
    {
        reference_sensor_id = registered_devices.begin()->first;
    }
    // check for the correct input
    if (registered_devices.find(reference_sensor_id.value()) == registered_devices.end())
    {
        std::cerr << "Error: sensor id does not exist.\n";
        std::cerr << "-r accepts id from 'device_id' field.\n";
        return 1;
    }
    // start data transmission simulation
    thread emitThread(emitter, registered_devices, cref(data));
    thread recvThread(receiver, registered_devices, reference_sensor_id.value(), number_of_calibration_samples, output_file, recording_start_frame, recording_finish_frame);
    // Wait for both threads to complete
    emitThread.join();
    recvThread.join();
}
