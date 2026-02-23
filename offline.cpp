// ==========================================================
//               OFFLINE IK WITHOUT VISUALIZER
//     Reads markers from a .trc, runs RTOSIM IKSolverParallel,
//     writes output .mot (TimeSeriesTable) like your online code.
// ==========================================================

#include <iostream>
#include <vector>
#include <thread>
#include <cstring>
#include <chrono>
#include <cmath>        // std::isfinite
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

#include "rtosim/queue/MarkerSetQueue.h"
#include "rtosim/queue/GeneralisedCoordinatesQueue.h"
#include "rtosim/IKSolverParallel.h"
#include "rtosim/EndOfData.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/DataAdapter.h>
#include <OpenSim/Tools/IKCoordinateTask.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>


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
        if (dt <= 0.0) dt = 1.0 / 30.0;

        double dx = (x - prevValue) / dt;

        double alphaD = computeAlpha(dCutoff, dt);
        prevDeriv = alphaD * dx + (1.0 - alphaD) * prevDeriv;

        double cutoff = minCutoff + beta * std::fabs(prevDeriv);
        double alpha = computeAlpha(cutoff, dt);

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
        SimTK::Vec3 lastRaw;
        SimTK::Vec3 xHat;
        double lastTime = 0.0;
    };

    std::vector<MarkerState> states;

    double maxJump;        // meters
    double maxVelocity;    // meters/second
    double alpha;          // smoothing factor (0.0–1.0)

    MarkerFilter(int numMarkers,
                 double maxJumpMeters = 1.5,
                 double maxVelocityMetersPerSec = 50.0,
                 double alpha_ = 0.1)
        : states(numMarkers),
          maxJump(maxJumpMeters),
          maxVelocity(maxVelocityMetersPerSec),
          alpha(alpha_) {}

    SimTK::Vec3 filterOne(int idx, const SimTK::Vec3& raw, double t) {

        auto& s = states[idx];

        if (!s.initialized) {
            s.initialized = true;
            s.lastRaw = raw;
            s.xHat = raw;
            s.lastTime = t;
            return raw;
        }

        double dt = t - s.lastTime;
        if (dt <= 0.0) dt = 1.0 / 30.0;

        double jump = (raw - s.lastRaw).norm();
        double velocity = jump / dt;
        bool outlier = (jump > maxJump) || (velocity > maxVelocity);

        if (outlier) {
            std::cout << "[OUTLIER] Marker " << idx
                      << " at time " << t
                      << " (jump=" << jump
                      << ", vel=" << velocity << ")\n";
        }

        SimTK::Vec3 input = outlier ? s.xHat : raw;

        // EMA smoothing
        s.xHat = alpha * input + (1.0 - alpha) * s.xHat;

        s.lastRaw  = raw;
        s.lastTime = t;

        return s.xHat;
    }
};

// ==========================================================
//                 OFFLINE PRODUCER (TRC)
// ==========================================================

static std::vector<std::string> kMarkerOrder = {
    // Torso
    "MAN","CLAG","C7","T8","XYP","ACD",
    // Scapula
    "MTACB","MTACM","MTACL",
    // Humerus
    // Elbow
    "EL","EM",
    // Forearm
    "PSR","PSU", "CLAD"
};

static void forceInDegreesNoInMot(const std::string& motPath) {
    std::ifstream in(motPath);
    if (!in.is_open()) return;

    std::vector<std::string> lines;
    lines.reserve(256);
    std::string line;
    bool foundInDegrees = false;
    bool insertedBeforeEndHeader = false;
    while (std::getline(in, line)) {
        if (line.rfind("inDegrees=", 0) == 0) {
            lines.emplace_back("inDegrees=no");
            foundInDegrees = true;
        } else if (!foundInDegrees && !insertedBeforeEndHeader && line == "endheader") {
            lines.emplace_back("inDegrees=no");
            lines.emplace_back(line);
            insertedBeforeEndHeader = true;
        } else {
            lines.emplace_back(line);
        }
    }
    in.close();

    if (!foundInDegrees && !insertedBeforeEndHeader) {
        lines.insert(lines.begin(), "inDegrees=no");
    }

    std::ofstream out(motPath, std::ios::trunc);
    if (!out.is_open()) return;
    for (const auto& l : lines) {
        out << l << "\n";
    }
}

static std::vector<std::string> splitWhitespaceTokens(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) tokens.push_back(token);
    return tokens;
}

static std::string toLowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

static double getTrcLengthScaleToMeters(const std::string& trcPath) {
    std::ifstream in(trcPath);
    if (!in.is_open()) {
        std::cout << "[TRC] WARNING: could not open file to read units. Assuming meters.\n";
        return 1.0;
    }

    // Typical TRC layout:
    // line 3: column names (contains "Units")
    // line 4: corresponding values (contains "mm" or "m")
    std::string line1, line2, line3, line4;
    if (!std::getline(in, line1) || !std::getline(in, line2) ||
        !std::getline(in, line3) || !std::getline(in, line4)) {
        std::cout << "[TRC] WARNING: header too short for units detection. Assuming meters.\n";
        return 1.0;
    }

    const auto keys = splitWhitespaceTokens(line3);
    const auto vals = splitWhitespaceTokens(line4);
    if (keys.empty() || vals.empty()) {
        std::cout << "[TRC] WARNING: unable to parse TRC header tokens for units. Assuming meters.\n";
        return 1.0;
    }

    int unitsIdx = -1;
    for (int i = 0; i < static_cast<int>(keys.size()); ++i) {
        if (toLowerCopy(keys[i]) == "units") {
            unitsIdx = i;
            break;
        }
    }
    if (unitsIdx < 0 || unitsIdx >= static_cast<int>(vals.size())) {
        std::cout << "[TRC] WARNING: TRC header has no usable 'Units' field. Assuming meters.\n";
        return 1.0;
    }

    const std::string units = toLowerCopy(vals[unitsIdx]);
    if (units == "mm" || units == "millimeter" || units == "millimeters") {
        std::cout << "[TRC] Units detected: " << vals[unitsIdx]
                  << " -> converting marker positions from mm to m.\n";
        return 0.001;
    }

    if (units == "m" || units == "meter" || units == "meters") {
        std::cout << "[TRC] Units detected: " << vals[unitsIdx]
                  << " -> marker positions already in meters.\n";
        return 1.0;
    }

    std::cout << "[TRC] WARNING: unknown units '" << vals[unitsIdx]
              << "'. Assuming meters.\n";
    return 1.0;
}

void trcProducer(const std::string& trcPath,
                 rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame>& markerQueue,
                 bool enableFilter)
{
    std::cout << "[TRC] Reading: " << trcPath << std::endl;
    const double lengthScaleToMeters = getTrcLengthScaleToMeters(trcPath);

    OpenSim::TimeSeriesTableVec3 table;

    try {
        // OpenSim 4.3: TRCFileAdapter::read() is a non-static member (DataAdapter API)
        OpenSim::TRCFileAdapter adapter;
        OpenSim::DataAdapter::OutputTables tables = adapter.read(trcPath);

        // Usually the TRC adapter stores the markers table under "markers"
        auto it = tables.find("markers");
        if (it == tables.end()) {
            std::cout << "[TRC] WARNING: key 'markers' not found. Available tables:\n";
            for (const auto& kv : tables) {
                std::cout << "  - " << kv.first << "\n";
            }
            throw std::runtime_error("[TRC] No 'markers' table found in TRC output.");
        }

        // Downcast from AbstractDataTable to TimeSeriesTableVec3
        const auto& tblPtr = it->second;  // shared_ptr<AbstractDataTable>
        if (!tblPtr) {
            throw std::runtime_error("[TRC] 'markers' table pointer is null.");
        }

        const auto* vec3Tbl = dynamic_cast<const OpenSim::TimeSeriesTableVec3*>(tblPtr.get());
        if (!vec3Tbl) {
            throw std::runtime_error("[TRC] 'markers' table is not a TimeSeriesTableVec3.");
        }

        table = *vec3Tbl; // copy


    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("[TRC] Failed to read TRC: ") + e.what());
    }

    const auto& labels = table.getColumnLabels();
    std::unordered_map<std::string, int> colIndex;
    colIndex.reserve(labels.size());
    for (int i = 0; i < (int)labels.size(); ++i) {
        colIndex[labels[i]] = i;
    }

    // Warn for missing markers (we’ll fill them with zeros)
    for (const auto& name : kMarkerOrder) {
        if (colIndex.find(name) == colIndex.end()) {
            std::cout << "[TRC] WARNING: marker column missing: '" << name
                      << "' (will fill with zeros)\n";
        }
    }

    const auto& times = table.getIndependentColumn();
    const int nRows = (int)table.getNumRows();
    const int nMarkers = (int)kMarkerOrder.size();

    std::cout << "[TRC] Rows: " << nRows << " | Markers expected: " << nMarkers << "\n";

    // Same filter parameters as your current online receiver
    MarkerFilter filter(nMarkers, /*maxJump*/0.18, /*maxVel*/6.0, /*alpha*/0.6);

    for (int r = 0; r < nRows; ++r) {
        const double t = times[r];

        rtosim::MarkerSetFrame frame;
        frame.time = t;
        frame.data.reserve(nMarkers);

        const auto row = table.getRowAtIndex(r); // RowVector_<SimTK::Vec3>

        for (int m = 0; m < nMarkers; ++m) {
            const auto& name = kMarkerOrder[m];

            SimTK::Vec3 raw(0.0);
            auto it = colIndex.find(name);
            if (it != colIndex.end()) {
                raw = row[it->second];
                raw *= lengthScaleToMeters;

            }

            // Protect against NaNs / Infs
            if (!std::isfinite(raw[0]) || !std::isfinite(raw[1]) || !std::isfinite(raw[2])) {
                raw = SimTK::Vec3(0);
            }

            SimTK::Vec3 clean = enableFilter ? filter.filterOne(m, raw, t) : raw;
            rtosim::MarkerData marker = clean;
            frame.data.push_back(marker);
        }

        markerQueue.push(frame);

        if ((r % 100) == 0) {
            std::cout << "[TRC] Pushed row " << r << "/" << nRows
                      << " at t=" << t << std::endl;
        }
    }

    markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
    std::cout << "[TRC] EndOfData pushed.\n";
}


// ==========================================================
//                         MAIN
// ==========================================================

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage:\n"
                  << "  ./offline_ik_noviz <model.osim> <markers.trc> [output_directory] [ik_tasks.xml] [--iktool] [--parity]\n"
                  << "  ./offline_ik_noviz <model.osim> <markers.trc> <ik_tasks.xml> [--iktool] [--parity]\n";
        return 1;
    }

    const std::string modelPath = argv[1];
    const std::string trcPath   = argv[2];
    std::string outputDir       = ".";
    std::string ikTasksPath;
    bool useIkTool = false;
    bool parityMode = false;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--iktool") {
            useIkTool = true;
        } else if (arg == "--parity") {
            parityMode = true;
        } else if (arg.size() >= 4 && arg.substr(arg.size() - 4) == ".xml") {
            ikTasksPath = arg;
        } else {
            outputDir = arg;
        }
    }

    if (!outputDir.empty() && outputDir.back() == '/')
        outputDir.pop_back();

    const std::string outputMotPath = outputDir + "/output_results.mot";

    // Queues & latches (same pattern as your online code)
    rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame> markerQueue;
    rtosim::IKoutputs<rtosim::GeneralisedCoordinatesFrame> outputQueue;
    rtb::Concurrency::Latch doneWithSubscriptions(1);
    rtb::Concurrency::Latch doneWithExecution(1);

    // Model (NO VISUALIZER)
    OpenSim::Model model(modelPath);
    model.setUseVisualizer(false);
    auto& state = model.initSystem();
    (void)state;

    std::cout << "[Init] Model system initialized (NO visualizer).\n";

    // Debug: verify producer marker list against model marker names/order.
    OpenSim::Array<std::string> modelMarkerNamesArray;
    const_cast<OpenSim::MarkerSet&>(model.getMarkerSet()).getMarkerNames(modelMarkerNamesArray);
    std::vector<std::string> modelMarkerNames;
    modelMarkerNames.reserve(modelMarkerNamesArray.getSize());
    for (int i = 0; i < modelMarkerNamesArray.getSize(); ++i) {
        modelMarkerNames.push_back(modelMarkerNamesArray[i]);
    }

    if ((int)modelMarkerNames.size() != (int)kMarkerOrder.size()) {
        std::cout << "[DEBUG] Marker count mismatch: model=" << modelMarkerNames.size()
                  << " producer(kMarkerOrder)=" << kMarkerOrder.size() << "\n";
    }

    const int minCount = std::min((int)modelMarkerNames.size(), (int)kMarkerOrder.size());
    int orderMismatches = 0;
    for (int i = 0; i < minCount; ++i) {
        if (modelMarkerNames[i] != kMarkerOrder[i]) {
            if (orderMismatches < 10) {
                std::cout << "[DEBUG] Marker order mismatch at index " << i
                          << ": model='" << modelMarkerNames[i]
                          << "' producer='" << kMarkerOrder[i] << "'\n";
            }
            ++orderMismatches;
        }
    }
    if (orderMismatches > 0) {
        std::cout << "[DEBUG] Total marker order mismatches: " << orderMismatches << "\n";
    } else {
        std::cout << "[DEBUG] Marker order check: producer order matches model order for "
                  << minCount << " markers.\n";
    }

    std::unordered_set<std::string> modelSet(modelMarkerNames.begin(), modelMarkerNames.end());
    std::unordered_set<std::string> producerSet(kMarkerOrder.begin(), kMarkerOrder.end());
    int printedMissingFromModel = 0;
    for (const auto& n : kMarkerOrder) {
        if (modelSet.find(n) == modelSet.end()) {
            if (printedMissingFromModel < 10) {
                std::cout << "[DEBUG] Producer marker not found in model: '" << n << "'\n";
            }
            ++printedMissingFromModel;
        }
    }
    int printedMissingFromProducer = 0;
    for (const auto& n : modelMarkerNames) {
        if (producerSet.find(n) == producerSet.end()) {
            if (printedMissingFromProducer < 10) {
                std::cout << "[DEBUG] Model marker not found in producer list: '" << n << "'\n";
            }
            ++printedMissingFromProducer;
        }
    }

    // Force producer marker order to model marker order.
    // This removes hardcoded-order/model-specific mismatches.
    kMarkerOrder = modelMarkerNames;
    std::cout << "[DEBUG] Producer marker order switched to model marker order ("
              << kMarkerOrder.size() << " markers).\n";

    if (useIkTool) {
        std::cout << "[DEBUG] Running OpenSim InverseKinematicsTool path.\n";
        OpenSim::InverseKinematicsTool ikTool;
        ikTool.setModel(model);
        ikTool.setMarkerDataFileName(trcPath);
        ikTool.setOutputMotionFileName(outputMotPath);
        ikTool.upd_accuracy() = 1e-4;
        ikTool.upd_constraint_weight() = 10.0;
        ikTool.upd_report_errors() = true;

        auto& tasks = ikTool.getIKTaskSet();
        for (const auto& markerName : modelMarkerNames) {
            auto* t = new OpenSim::IKMarkerTask();
            t->setName(markerName);
            t->setApply(true);
            t->setWeight(1.0);
            tasks.adoptAndAppend(t);
        }

        const bool ok = ikTool.run();
        if (!ok) {
            std::cerr << "[ERROR] InverseKinematicsTool::run() failed.\n";
            return 2;
        }
        std::cout << "[Main] Saved " << outputMotPath << " via IKTool.\n";
        std::cout << "[Main] Finished OFFLINE IK (IKTool mode).\n";
        return 0;
    }

    // IK solver
    rtosim::IKSolverParallel ikSolver(
        markerQueue,
        outputQueue,
        doneWithSubscriptions,
        doneWithExecution,
        modelPath,
        1e-4,
        10.0
    );
    ikSolver.setParityMode(parityMode);
    if (parityMode) {
        std::cout << "[DEBUG] Parity mode enabled: persistent IK tracking, no marker filter.\n";
    }

    // ======================
    //  IK TASKS
    // ======================
    if (!ikTasksPath.empty()) {
        ikSolver.setInverseKinematicsTaskSet(ikTasksPath);
        std::cout << "[DEBUG] Loaded IK tasks from: " << ikTasksPath << "\n";
    } else {
        std::cout << "[DEBUG] No IK task file: using solver defaults (all marker weights = 1).\n";
    }

    // Producer thread (TRC) + IK thread
    std::thread producerThread([&](){
        try {
            trcProducer(trcPath, markerQueue, !parityMode);
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
        }
    });

    std::thread ikThread(std::ref(ikSolver));

    // Results collection
    int frameCount = 0;
    std::vector<OneEuroFilter> jointFilters;
    bool jointFiltersInitialized = false;

    std::vector<std::pair<double, std::vector<double>>> allResults;
    allResults.reserve(10000);

    // ================= MAIN IK LOOP =================
    while (true) {
        auto result = outputQueue.pop();
        if (rtosim::EndOfData::isEod(result))
            break;

        auto start = std::chrono::high_resolution_clock::now();

        auto qVals = result.data.getQ();

        // Keep the same "present but disabled" OneEuro pattern as your online code
        if (!jointFiltersInitialized) {
            jointFilters.resize(
                qVals.size(),
                OneEuroFilter(/*minCutoff*/4.0, /*beta*/0.02, /*dCutoff*/1.0)
            );
            jointFiltersInitialized = true;
        }

        for (int i = 0; i < (int)qVals.size(); ++i) {
            // qVals[i] = jointFilters[i].filter(qVals[i], result.time);
            qVals[i] = qVals[i];
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        if ((frameCount % 100) == 0) {
            std::cout << "[IK] Frame " << frameCount
                      << " t=" << result.time
                      << " took " << (duration_us / 1000.0) << " ms\n";
        }

        allResults.emplace_back(result.time, qVals);
        frameCount++;
    }

    producerThread.join();
    ikThread.join();

    // ================= WRITE .MOT =================
    if (!allResults.empty()) {
        OpenSim::TimeSeriesTable qTable;
        OpenSim::Array<std::string> coordNames;
        model.getCoordinateSet().getNames(coordNames);

        std::vector<std::string> stdCoordNames;
        stdCoordNames.reserve(coordNames.getSize());
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

        try {
            OpenSim::STOFileAdapter::write(qTable, outputMotPath);
            forceInDegreesNoInMot(outputMotPath);
            std::cout << "[Main] Saved " << outputMotPath
                      << " with " << allResults.size() << " frames.\n";
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to save " << outputMotPath
                      << " : " << e.what() << std::endl;
            return 2;
        }
    } else {
        std::cout << "[Main] No frames to save.\n";
    }

    std::cout << "[Main] Finished OFFLINE IK (NO visualizer).\n";
    return 0;
}
