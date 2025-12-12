#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>

// POSIX networking
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// RTOSIM
#include "rtosim/queue/MarkerSetQueue.h"
#include "rtosim/queue/GeneralisedCoordinatesQueue.h"
#include "rtosim/IKSolverParallel.h"
#include "rtosim/MarkersReferenceFromQueue.h"
#include "rtosim/EndOfData.h"

// OpenSim (v4.3)
#include <OpenSim/OpenSim.h>

// Concurrency
#include <rtb/concurrency/Latch.h>

// ----------------------
// Configuration
// ----------------------
struct Config {
    int port = 5555;
    int numMarkers = 20;
    double frameRate = 30.0; // Hz
    std::string modelPath;

    Config(const std::string& defaultModel)
        : modelPath(defaultModel) {}
};

// ----------------------
// Socket receiver
// ----------------------
void socketReceiver(
    rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame>& markerQueue,
    const Config& cfg)
{
    const int BYTES_PER_MARKER = 3 * sizeof(float);
    const int FRAME_SIZE = cfg.numMarkers * BYTES_PER_MARKER;

    int server_fd = -1;
    int client_fd = -1;
    sockaddr_in address{};
    socklen_t addrlen = sizeof(address);

    std::cout << "[SocketReceiver] Starting on port " << cfg.port << "…" << std::endl;

    server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::perror("[SocketReceiver] socket() failed");
        markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(cfg.port));

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        std::perror("[SocketReceiver] bind() failed");
        close(server_fd);
        markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
        return;
    }

    if (listen(server_fd, 1) < 0) {
        std::perror("[SocketReceiver] listen() failed");
        close(server_fd);
        markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
        return;
    }

    std::cout << "[SocketReceiver] Waiting for client…" << std::endl;
    client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&address), &addrlen);
    if (client_fd < 0) {
        std::perror("[SocketReceiver] accept() failed");
        close(server_fd);
        markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
        return;
    }
    std::cout << "[SocketReceiver] Client connected." << std::endl;

    std::vector<float> buffer(cfg.numMarkers * 3);
    int frameCount = 0;
    const double dt = 1.0 / cfg.frameRate;

    while (true) {
        int bytesRead = ::read(client_fd, buffer.data(), FRAME_SIZE);
        if (bytesRead == 0) {
            std::cout << "[SocketReceiver] Client closed connection." << std::endl;
            break;
        }
        if (bytesRead < 0) {
            std::perror("[SocketReceiver] read() failed");
            break;
        }
        if (bytesRead != FRAME_SIZE) {
            std::cerr << "[SocketReceiver] Incomplete frame: got "
                      << bytesRead << " bytes, expected "
                      << FRAME_SIZE << " — skipping.\n";
            continue;
        }

        rtosim::MarkerSetFrame frame;
        frame.time = frameCount * dt;
        frameCount++;

        frame.data.reserve(cfg.numMarkers);
        for (int i = 0; i < cfg.numMarkers; ++i) {
            const float x = buffer[3 * i + 0];
            const float y = buffer[3 * i + 1];
            const float z = buffer[3 * i + 2];
            frame.data.emplace_back(SimTK::Vec3(x, y, z));
        }

        markerQueue.push(frame);
        std::cout << "[SocketReceiver] Pushed frame #" << frameCount
                  << " at t = " << frame.time << " s" << std::endl;
    }

    markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());

    if (client_fd >= 0) close(client_fd);
    if (server_fd >= 0) close(server_fd);

    std::cout << "[SocketReceiver] Shutdown complete." << std::endl;
}

// ----------------------
// Main
// ----------------------
int main(int argc, char** argv) {
    // -------- Config from CLI --------
    std::string defaultModel = "upperlimb-biorob.osim"; // relative by default
    Config cfg(defaultModel);

    if (argc > 1) {
        cfg.modelPath = argv[1]; // first argument: model path
    }

    std::cout << "[Init] Using model: " << cfg.modelPath << std::endl;

    // -------- Queues & latches --------
    rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame> markerQueue;
    rtosim::IKoutputs<rtosim::GeneralisedCoordinatesFrame> outputQueue;

    rtb::Concurrency::Latch doneWithSubscriptions(1);
    rtb::Concurrency::Latch doneWithExecution(1);

    // -------- Load model & visualizer --------
    OpenSim::Model model;
    try {
        model = OpenSim::Model(cfg.modelPath);
    } catch (const std::exception& e) {
        std::cerr << "[Init] Failed to load model: " << e.what() << std::endl;
        return 1;
    }

    model.setUseVisualizer(true);
    SimTK::State state;
    try {
        state = model.initSystem();
    } catch (const std::exception& e) {
        std::cerr << "[Init] Failed to init system: " << e.what() << std::endl;
        return 1;
    }

    auto& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(SimTK::Visualizer::GroundAndSky);

    std::cout << "[Init] Model and visualizer initialized." << std::endl;

    // -------- Start IK solver + socket thread --------
    const double accuracy = 1e-4;
    const double constraintWeight = 10.0;

    rtosim::IKSolverParallel ikSolver(
        markerQueue,
        outputQueue,
        doneWithSubscriptions,
        doneWithExecution,
        cfg.modelPath,
        accuracy,
        constraintWeight);

    std::thread ikThread(std::ref(ikSolver));
    std::thread socketThread(socketReceiver, std::ref(markerQueue), std::cref(cfg));

    // -------- Consume IK results --------
    int frameCount = 0;
    std::vector<std::pair<double, std::vector<double>>> allResults;

    while (true) {
        auto result = outputQueue.pop();
        if (rtosim::EndOfData::isEod(result)) {
            std::cout << "[Main] Received EndOfData from IK solver." << std::endl;
            break;
        }

        auto start = std::chrono::high_resolution_clock::now();

        auto qVals = result.data.getQ();
        auto& coordSet = model.updCoordinateSet();

        const int nCoords = std::min<int>(coordSet.getSize(), qVals.size());
        for (int i = 0; i < nCoords; ++i) {
            coordSet[i].setValue(state, qVals[i]);
        }

        model.realizePosition(state);
        viz.report(state);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        double duration_ms = duration_us / 1000.0;

        std::cout << "[IK Timing] Frame " << frameCount
                  << " took " << duration_ms << " ms ("
                  << duration_us << " µs)." << std::endl;

        allResults.emplace_back(result.time, qVals);
        frameCount++;
    }

    // -------- Join threads --------
    if (socketThread.joinable()) socketThread.join();
    if (ikThread.joinable()) ikThread.join();

    // -------- Save results to .mot --------
    if (!allResults.empty()) {
        OpenSim::TimeSeriesTable qTable;
        OpenSim::Array<std::string> coordNames;
        model.getCoordinateSet().getNames(coordNames);

        std::vector<std::string> stdCoordNames;
        stdCoordNames.reserve(coordNames.getSize());
        for (int i = 0; i < coordNames.getSize(); ++i) {
            stdCoordNames.emplace_back(coordNames[i]);
        }
        qTable.setColumnLabels(stdCoordNames);

        double lastTime = -1.0;
        for (const auto& [time, qVals] : allResults) {
            if (time <= lastTime) continue;
            lastTime = time;

            SimTK::RowVector row(static_cast<int>(qVals.size()));
            for (int i = 0; i < static_cast<int>(qVals.size()); ++i) {
                row[i] = qVals[i];
            }
            qTable.appendRow(time, row);
        }

        const std::string outName = "output_results.mot";
        OpenSim::STOFileAdapter::write(qTable, outName);
        std::cout << "[Main] Saved " << outName
                  << " with " << allResults.size() << " raw frames." << std::endl;
    } else {
        std::cout << "[Main] No frames to save." << std::endl;
    }

    return 0;
}
