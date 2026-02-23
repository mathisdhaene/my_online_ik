#include <iostream>
#include <vector>
#include <thread>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <cmath>    // for std::isfinite
#include <unordered_map>
#include <functional>

#include "rtosim/queue/MarkerSetQueue.h"
#include "rtosim/queue/GeneralisedCoordinatesQueue.h"
#include "rtosim/IKSolverParallel.h"
#include "rtosim/MarkersReferenceFromQueue.h"
#include "rtosim/EndOfData.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Set.h>

#include <rtb/concurrency/Latch.h>

// ==========================================================
//              ONE EURO FILTER FOR JOINT ANGLES
// ==========================================================

class OneEuroFilter {
public:
    OneEuroFilter(double minCutoff = 1.5, double beta = 0.02, double dCutoff = 1.0)
        : minCutoff(minCutoff),
          beta(beta),
          dCutoff(dCutoff),
          initialized(false),
          prevValue(0.0),
          prevDeriv(0.0),
          prevTime(0.0) {}

    double filter(double x, double t) {
        if (!initialized) {
            initialized = true;
            prevValue = x;
            prevDeriv = 0.0;
            prevTime = t;
            return x;
        }

        double dt = t - prevTime;
        if (dt <= 0.0)
            dt = 1.0 / 30.0;

        // Derivative (velocity) estimation
        double dx = (x - prevValue) / dt;

        double alphaD = computeAlpha(dCutoff, dt);
        prevDeriv = alphaD * dx + (1.0 - alphaD) * prevDeriv;

        // Adaptive cutoff based on speed
        double cutoff = minCutoff + beta * std::fabs(prevDeriv);
        double alpha = computeAlpha(cutoff, dt);

        // EMA smoothing
        double filtered = alpha * x + (1.0 - alpha) * prevValue;

        prevValue = filtered;
        prevTime = t;

        return filtered;
    }

private:
    bool initialized;
    double prevValue;
    double prevDeriv;
    double prevTime;

    double minCutoff;
    double beta;
    double dCutoff;

    double computeAlpha(double cutoff, double dt) {
        const double PI = 3.14159265358979323846;
        double tau = 1.0 / (2.0 * PI * cutoff);
        return 1.0 / (1.0 + tau / dt);
    }
};

// ==========================================================
//          MARKER FILTER: OUTLIERS + LIGHT SMOOTHING
// ==========================================================

struct MarkerFilter {

    struct MarkerState {
        bool initialized = false;
        SimTK::Vec3 lastRaw;   // last raw sample
        SimTK::Vec3 xHat;      // filtered estimate
        double lastTime = 0.0;
    };

    std::vector<MarkerState> states;

    double maxJump;        // meters
    double maxVelocity;    // meters/second
    double alpha;          // smoothing factor (0.0–1.0)

    MarkerFilter(int numMarkers,
                 double maxJumpMeters = 1.5,
                 double maxVelocityMetersPerSec = 50.0,
                 double alpha_ = 0.1,
                 double /*minCutoffHz*/ = 0.0,
                 double /*beta_*/ = 0.0,
                 double /*dCutoffHz*/ = 0.0)
        : states(numMarkers),
          maxJump(maxJumpMeters),
          maxVelocity(maxVelocityMetersPerSec),
          alpha(alpha_) {}

    SimTK::Vec3 filterOne(int idx, const SimTK::Vec3& raw, double t) {

        auto& s = states[idx];

        // First-sample init
        if (!s.initialized) {
            s.initialized = true;
            s.lastRaw = raw;
            s.xHat = raw;
            s.lastTime = t;
            return raw;
        }

        double dt = t - s.lastTime;
        if (dt <= 0.0)
            dt = 1.0 / 30.0;

        // --- OUTLIER DETECTION ---
        double jump = (raw - s.lastRaw).norm();
        double velocity = jump / dt;
        bool outlier = (jump > maxJump) || (velocity > maxVelocity);

        if (outlier) {
            std::cout << "[OUTLIER] Marker " << idx
                      << " at time " << t
                      << " (jump=" << jump
                      << ", vel=" << velocity << ")" << std::endl;
        }

        // --- INPUT ---
        SimTK::Vec3 input = outlier ? s.xHat : raw;

        // --- SMOOTHING ---
        s.xHat = alpha * input + (1.0 - alpha) * s.xHat;

        // --- STATE UPDATE ---
        s.lastRaw  = raw;
        s.lastTime = t;

        return s.xHat;
    }
};


		

    


// ======================== SOCKET SETTINGS ========================
const int PORT = 5555;
const int STREAM_NUM_MARKERS = 20;
const int BYTES_PER_MARKER = 3 * sizeof(float);
const int FRAME_SIZE = STREAM_NUM_MARKERS * BYTES_PER_MARKER;

static const std::vector<std::string> kIncomingMarkerOrder = {
    "ACD", "CLAD", "CLAG", "MAN", "XYP", "T8", "MTACM", "MTACB", "MTACL", "MTHA",
    "MTHP", "MTBA", "MTBP", "EL", "EM", "PSU", "PSR", "MC5", "MC2", "C7"
};

// ======================== SOCKET RECEIVER ========================

void socketReceiver(
    rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame>& markerQueue,
    const std::vector<std::string>& modelMarkerNames) {

    if (kIncomingMarkerOrder.size() != static_cast<size_t>(STREAM_NUM_MARKERS)) {
        std::cerr << "[SocketReceiver] Internal error: stream marker order size mismatch." << std::endl;
        markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
        return;
    }

    std::unordered_map<std::string, int> incomingIdxByName;
    incomingIdxByName.reserve(kIncomingMarkerOrder.size());
    for (int i = 0; i < static_cast<int>(kIncomingMarkerOrder.size()); ++i) {
        incomingIdxByName[kIncomingMarkerOrder[i]] = i;
    }

    std::vector<int> modelToIncomingIdx(modelMarkerNames.size(), -1);
    int mappedCount = 0;
    for (int i = 0; i < static_cast<int>(modelMarkerNames.size()); ++i) {
        const auto it = incomingIdxByName.find(modelMarkerNames[i]);
        if (it != incomingIdxByName.end()) {
            modelToIncomingIdx[i] = it->second;
            ++mappedCount;
        } else {
            std::cerr << "[SocketReceiver] WARNING: model marker '" << modelMarkerNames[i]
                      << "' not found in incoming stream; filling with zeros." << std::endl;
        }
    }
    std::cout << "[SocketReceiver] Marker mapping: " << mappedCount << "/"
              << modelMarkerNames.size() << " model markers found in stream." << std::endl;

    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    float buffer[STREAM_NUM_MARKERS * 3];
    int frameCount = 0;

    // Marker filter: outliers + light smoothing
    MarkerFilter filter(static_cast<int>(modelMarkerNames.size()), 0.18, 6.0, 0.6);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    bind(server_fd, (struct sockaddr*)&address, sizeof(address));
    listen(server_fd, 3);
    new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);


    while (true) {
        int totalRead = 0;
        while (totalRead < FRAME_SIZE) {
            const int bytesRead = read(
                new_socket,
                reinterpret_cast<char*>(buffer) + totalRead,
                FRAME_SIZE - totalRead
            );
            if (bytesRead <= 0) {
                totalRead = bytesRead;
                break;
            }
            totalRead += bytesRead;
        }
        if (totalRead <= 0) {
            break;
        }
        if (totalRead != FRAME_SIZE) {
            std::cerr << "[SocketReceiver] Incomplete frame received (" << totalRead
                      << " bytes), stopping stream." << std::endl;
            break;
        }

        rtosim::MarkerSetFrame frame;

        // Use a simple synthetic timestamp at 30 Hz
        frame.time = frameCount * (1.0 / 30.0);
        double t = frame.time;
        frameCount++;

        std::vector<SimTK::Vec3> incomingMarkers(STREAM_NUM_MARKERS, SimTK::Vec3(0));
        for (int i = 0; i < STREAM_NUM_MARKERS; ++i) {
            SimTK::Vec3 raw(
                buffer[i * 3 + 0],
                buffer[i * 3 + 1],
                buffer[i * 3 + 2]
            );

            // Protect against NaNs / Infs
            if (!std::isfinite(raw[0]) ||
                !std::isfinite(raw[1]) ||
                !std::isfinite(raw[2])) {
                raw = SimTK::Vec3(0);
            }
            incomingMarkers[i] = raw;
        }

        frame.data.reserve(modelMarkerNames.size());
        for (int i = 0; i < static_cast<int>(modelMarkerNames.size()); ++i) {
            SimTK::Vec3 raw(0);
            const int incomingIdx = modelToIncomingIdx[i];
            if (incomingIdx >= 0 && incomingIdx < STREAM_NUM_MARKERS) {
                raw = incomingMarkers[incomingIdx];
            }
            SimTK::Vec3 clean = filter.filterOne(i, raw, t);
            rtosim::MarkerData marker = clean;
            frame.data.push_back(marker);
        }

        markerQueue.push(frame);
        std::cout << "[SocketReceiver] Frame #" << frameCount
                  << " pushed at time: " << frame.time << std::endl;
    }

    markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
    close(new_socket);
    close(server_fd);
}

// ======================== MAIN ========================

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cerr << "Usage: ./online_ik_test <model.osim> [output_directory]" << std::endl;
        return 1;
    }

    // Model path is required
    std::string modelPath = argv[1];

    // Optional output directory for the .mot file
    std::string outputDir = (argc >= 3) ? argv[2] : ".";

    // Ensure trailing slash is optional
    if (outputDir.back() == '/')
        outputDir.pop_back();

    // Build final .mot path
    std::string outputMotPath = outputDir + "/output_results.mot";

    rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame> markerQueue;
    rtosim::IKoutputs<rtosim::GeneralisedCoordinatesFrame> outputQueue;
    rtb::Concurrency::Latch doneWithSubscriptions(1);
    rtb::Concurrency::Latch doneWithExecution(1);

    OpenSim::Model model(modelPath);
    model.setUseVisualizer(true);  // Enable visualization

    auto& state = model.initSystem();  // Initialize system & visualizer
    auto& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(SimTK::Visualizer::GroundAndSky);
    model.realizePosition(state);
    viz.report(state); // Draw initial pose immediately.

    std::cout << "[Init] Model system and visualizer initialized." << std::endl;

    OpenSim::Array<std::string> modelMarkerNamesArray;
    const_cast<OpenSim::MarkerSet&>(model.getMarkerSet()).getMarkerNames(modelMarkerNamesArray);
    std::vector<std::string> modelMarkerNames;
    modelMarkerNames.reserve(modelMarkerNamesArray.getSize());
    for (int i = 0; i < modelMarkerNamesArray.getSize(); ++i) {
        modelMarkerNames.push_back(modelMarkerNamesArray[i]);
    }
    std::cout << "[Init] Model markers: " << modelMarkerNames.size() << std::endl;


    rtosim::IKSolverParallel ikSolver(
        markerQueue,
        outputQueue,
        doneWithSubscriptions,
        doneWithExecution,
        modelPath,
        1e-4,
        10.0
    );

    // ======================
    //  CUSTOM MARKER WEIGHTS
    // ======================
    OpenSim::IKTaskSet ikTasks;

    auto addTask = [&](const std::string& name, double w) {
        auto* t = new OpenSim::IKMarkerTask();
        t->setName(name);
        t->setApply(true);
        t->setWeight(w);
        ikTasks.adoptAndAppend(t);
    };

    // Torso
    addTask("ACD", 1.0);
    addTask("CLAD", 1.0);
    addTask("CLAG", 1.0);
    addTask("MAN", 0.8);
    addTask("XYP", 0.8);
    addTask("T8", 0.8);
    addTask("C7", 0.8);

    // Scapula
    addTask("MTACM", 0.2);
    addTask("MTACB", 0.2);
    addTask("MTACL", 0.2);

    // Humerus
    addTask("MTHA", 0.3);
    addTask("MTHP", 0.25);
    addTask("MTBA", 0.25);
    addTask("MTBP", 0.25);

    // Elbow
    addTask("EL", 0);
    addTask("EM", 0.5);

    // Forearm
    addTask("PSU", 0.8);
    addTask("PSR", 0.8);

    // Hand
    addTask("MC5", 0.9);
    addTask("MC2", 0.9);

    ikSolver.setInverseKinematicsTaskSet(ikTasks);

    // NOW start IK thread
    std::thread ikThread(std::ref(ikSolver));




    std::thread socketThread(socketReceiver, std::ref(markerQueue), std::cref(modelMarkerNames));

    int frameCount = 0;
    std::vector<OneEuroFilter> jointFilters;
    bool jointFiltersInitialized = false;
    std::vector<std::pair<double, std::vector<double>>> allResults;

    // ================= MAIN IK LOOP =================
    while (true) {
        auto result = outputQueue.pop();
        if (rtosim::EndOfData::isEod(result))
            break;

        auto start = std::chrono::high_resolution_clock::now();

        std::cout << "[Main] Processing frame at time: " << result.time << std::endl;
        auto qVals = result.data.getQ();

        // Initialize joint filters on first frame
        if (!jointFiltersInitialized) {
            jointFilters.resize(
                qVals.size(),
                OneEuroFilter(
                    /* minCutoff */ 4.0,   // more responsive
                    /* beta      */ 0.02,  // lighter smoothing
                    /* dCutoff   */ 1.0
                )
            );
            jointFiltersInitialized = true;
        }

        // One-Euro filtering of each joint angle
        for (int i = 0; i < (int)qVals.size(); ++i) {
            // qVals[i] = jointFilters[i].filter(qVals[i], result.time);
            qVals[i] = qVals[i];
        }

        auto& coordSet = model.updCoordinateSet();
        for (int i = 0; i < coordSet.getSize(); ++i) {
            coordSet[i].setValue(state, qVals[i]);
        }

        model.realizePosition(state);
        viz.report(state);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        double duration_ms = duration_us / 1000.0;

        std::cout << "[IK Timing] Frame " << frameCount << " took "
                  << duration_ms << " ms (" << duration_us << " μs)." << std::endl;

        allResults.emplace_back(result.time, qVals);
        frameCount++;
    }

    // Optional: drain any remaining frames if RTOSIM pushes after EoD
    while (true) {
        auto remainingResult = outputQueue.pop();
        if (rtosim::EndOfData::isEod(remainingResult))
            break;

        auto start = std::chrono::high_resolution_clock::now();

        std::cout << "[Main] Processing remaining frame at time: "
                  << remainingResult.time << std::endl;
        auto qVals = remainingResult.data.getQ();

        auto& coordSet = model.updCoordinateSet();
        for (int i = 0; i < coordSet.getSize(); ++i) {
            coordSet[i].setValue(state, qVals[i]);
        }

        model.realizePosition(state);
        viz.report(state);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        double duration_ms = duration_us / 1000.0;

        std::cout << "[IK Timing] Frame " << frameCount << " took "
                  << duration_ms << " ms (" << duration_us << " μs)." << std::endl;

        for (const auto& q : qVals)
            std::cout << q << " ";
        std::cout << std::endl;

        allResults.emplace_back(remainingResult.time, qVals);
        frameCount++;
    }

    socketThread.join();
    ikThread.join();

    // ================= WRITE .MOT =================
    if (!allResults.empty()) {
        OpenSim::TimeSeriesTable qTable;
        OpenSim::Array<std::string> coordNames;
        model.getCoordinateSet().getNames(coordNames);

        std::vector<std::string> stdCoordNames;
        for (int i = 0; i < coordNames.getSize(); ++i) {
            stdCoordNames.push_back(coordNames[i]);
        }
        qTable.setColumnLabels(stdCoordNames);

        double lastTime = -1.0;
        for (const auto& [time, qVals] : allResults) {
            if (time <= lastTime)
                continue;

            SimTK::RowVector row((int)qVals.size());
            for (int i = 0; i < (int)qVals.size(); ++i)
                row[i] = qVals[i];

            qTable.appendRow(time, row);
            lastTime = time;
        }

				// Write .mot to the chosen directory
				try {
						OpenSim::STOFileAdapter::write(qTable, outputMotPath);
						std::cout << "[Main] Saved " << outputMotPath
								      << " with " << allResults.size() << " frames." << std::endl;
				} catch (const std::exception& e) {
						std::cerr << "[ERROR] Failed to save " << outputMotPath
								      << " : " << e.what() << std::endl;
				}

        std::cout << "[Main] Saved output_results.mot with "
                  << allResults.size() << " frames." << std::endl;

    } else {
        std::cout << "[Main] No frames to save." << std::endl;
    }
		// ----- Close Simbody Visualizer (OpenSim 4.x workaround) -----
		system("pkill -f simbody-visualizer");

    return 0;
}
