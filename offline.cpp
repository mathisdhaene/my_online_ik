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
#include <OpenSim/Tools/IKMarkerTask.h>
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

namespace {
constexpr double kJointFilterMinCutoff = 30.0;
constexpr double kJointFilterBeta = 0.1;
constexpr double kJointFilterDCutoff = 5.0;
}

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

    SimTK::Vec3 filterOne(int idx, const SimTK::Vec3& raw, double t, bool isValid) {
        auto& s = states[idx];

        if (!isValid) {
            if (s.initialized) {
                s.lastTime = t;
                return s.xHat;
            }
            return SimTK::Vec3(0.0);
        }

        if (!s.initialized) {
            s.initialized = true;
            s.lastRaw = raw;
            s.xHat = raw;
            s.lastTime = t;
            return raw;
        }

        double dt = t - s.lastTime;
        if (dt <= 0.0) dt = 1.0 / 30.0;

        const double jump = (raw - s.lastRaw).norm();
        const double velocity = jump / dt;
        const bool outlier = (jump > maxJump) || (velocity > maxVelocity);

        if (outlier) {
            std::cout << "[OUTLIER] Marker " << idx
                      << " at time " << t
                      << " (jump=" << jump
                      << ", vel=" << velocity << ")\n";
            // Hold the filtered estimate for this frame, but advance the raw
            // reference so the filter can reacquire after a fast real motion.
            s.lastRaw = raw;
            s.lastTime = t;
            return s.xHat;
        }

        s.xHat = alpha * raw + (1.0 - alpha) * s.xHat;
        s.lastRaw = raw;
        s.lastTime = t;
        return s.xHat;
    }
};

static bool isFiniteVec3(const SimTK::Vec3& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

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

    // TRC files are not always identical: detect the header row containing
    // "Units" and read the following row as values.
    std::vector<std::string> lines;
    lines.reserve(16);
    std::string line;
    for (int i = 0; i < 16 && std::getline(in, line); ++i) {
        lines.push_back(line);
    }
    if (lines.size() < 2) {
        std::cout << "[TRC] WARNING: header too short for units detection. Assuming meters.\n";
        return 1.0;
    }

    int headerRowIdx = -1;
    int unitsIdx = -1;
    for (int r = 0; r < static_cast<int>(lines.size()) - 1; ++r) {
        const auto keys = splitWhitespaceTokens(lines[r]);
        if (keys.empty()) continue;
        for (int c = 0; c < static_cast<int>(keys.size()); ++c) {
            if (toLowerCopy(keys[c]) == "units") {
                headerRowIdx = r;
                unitsIdx = c;
                break;
            }
        }
        if (headerRowIdx >= 0) break;
    }

    if (headerRowIdx < 0 || unitsIdx < 0 || headerRowIdx + 1 >= static_cast<int>(lines.size())) {
        std::cout << "[TRC] WARNING: TRC header has no usable 'Units' field. Assuming meters.\n";
        return 1.0;
    }

    const auto vals = splitWhitespaceTokens(lines[headerRowIdx + 1]);
    if (unitsIdx >= static_cast<int>(vals.size())) {
        std::cout << "[TRC] WARNING: 'Units' column found but value row is malformed. Assuming meters.\n";
        return 1.0;
    }

    const std::string unitsRaw = vals[unitsIdx];
    const std::string units = toLowerCopy(unitsRaw);
    if (units == "mm" || units == "millimeter" || units == "millimeters") {
        std::cout << "[TRC] Units detected: " << unitsRaw
                  << " -> converting marker positions from mm to m.\n";
        return 0.001;
    }

    if (units == "m" || units == "meter" || units == "meters") {
        std::cout << "[TRC] Units detected: " << unitsRaw
                  << " -> marker positions already in meters.\n";
        return 1.0;
    }

    std::cout << "[TRC] WARNING: unknown units '" << unitsRaw
              << "'. Assuming meters.\n";
    return 1.0;
}

static std::unordered_set<std::string> getTrcMarkerLabels(const std::string& trcPath) {
    OpenSim::TRCFileAdapter adapter;
    OpenSim::DataAdapter::OutputTables tables = adapter.read(trcPath);

    auto it = tables.find("markers");
    if (it == tables.end() || !it->second) {
        throw std::runtime_error("[TRC] No valid 'markers' table found while reading labels.");
    }

    const auto* vec3Tbl = dynamic_cast<const OpenSim::TimeSeriesTableVec3*>(it->second.get());
    if (!vec3Tbl) {
        throw std::runtime_error("[TRC] 'markers' table is not a TimeSeriesTableVec3.");
    }

    std::unordered_set<std::string> labels;
    const auto& colLabels = vec3Tbl->getColumnLabels();
    labels.reserve(colLabels.size());
    for (const auto& s : colLabels) {
        labels.insert(s);
    }
    return labels;
}

void trcProducer(const std::string& trcPath,
                 rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame>& markerQueue,
                 bool enableFilter,
                 double startTimeSeconds)
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

    // Warn for missing markers; the causal filter will hold the last estimate.
    for (const auto& name : kMarkerOrder) {
        if (colIndex.find(name) == colIndex.end()) {
            std::cout << "[TRC] WARNING: marker column missing: '" << name
                      << "' (will hold last estimate)\n";
        }
    }

    const auto& times = table.getIndependentColumn();
    const int nRows = (int)table.getNumRows();
    const int nMarkers = (int)kMarkerOrder.size();

    std::cout << "[TRC] Rows: " << nRows << " | Markers expected: " << nMarkers << "\n";
    if (startTimeSeconds > 0.0) {
        std::cout << "[TRC] Start-time filter enabled: keeping frames with t >= "
                  << startTimeSeconds << " s\n";
    }

    // Same filter parameters as your current online receiver
    MarkerFilter filter(nMarkers, /*maxJump*/0.18, /*maxVel*/6.0, /*alpha*/0.6);
    int keptRows = 0;
    bool warnedStartAfterEnd = false;

    for (int r = 0; r < nRows; ++r) {
        const double t = times[r];
        if (t < startTimeSeconds) {
            continue;
        }
        ++keptRows;

        rtosim::MarkerSetFrame frame;
        frame.time = t;
        frame.data.reserve(nMarkers);

        const auto row = table.getRowAtIndex(r); // RowVector_<SimTK::Vec3>

        for (int m = 0; m < nMarkers; ++m) {
            const auto& name = kMarkerOrder[m];

            SimTK::Vec3 raw(0.0);
            bool isValid = false;
            auto it = colIndex.find(name);
            if (it != colIndex.end()) {
                raw = row[it->second];
                raw *= lengthScaleToMeters;
                isValid = isFiniteVec3(raw);
            }

            SimTK::Vec3 clean;
            if (enableFilter) {
                clean = filter.filterOne(m, raw, t, isValid);
            } else {
                clean = isValid ? raw : SimTK::Vec3(0.0);
            }
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
    if (keptRows == 0) {
        warnedStartAfterEnd = true;
        std::cout << "[TRC] WARNING: no rows kept after start-time filter.\n";
    }
    if (!warnedStartAfterEnd) {
        std::cout << "[TRC] Kept rows after start-time filter: " << keptRows << "\n";
    }
    std::cout << "[TRC] EndOfData pushed.\n";
}


// ==========================================================
//                         MAIN
// ==========================================================

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage:\n"
                  << "  ./offline_ik_noviz <model.osim> <markers.trc> [output_directory] [ik_tasks.xml] [--iktool] [--parity] [--start-time <seconds>] [--output-name <file.mot>] [--output-file <path/file.mot>]\n"
                  << "  ./offline_ik_noviz <model.osim> <markers.trc> <ik_tasks.xml> [--iktool] [--parity] [--start-time <seconds>] [--output-name <file.mot>] [--output-file <path/file.mot>]\n";
        return 1;
    }

    const std::string modelPath = argv[1];
    const std::string trcPath   = argv[2];
    std::string outputDir       = ".";
    std::string ikTasksPath;
    std::string outputName      = "output_results.mot";
    std::string outputFilePath;
    bool useIkTool = false;
    bool parityMode = false;
    double startTimeSeconds = 0.0;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--iktool") {
            useIkTool = true;
        } else if (arg == "--parity") {
            parityMode = true;
        } else if (arg == "--start-time") {
            if (i + 1 >= argc) {
                std::cerr << "[ERROR] --start-time requires a value in seconds.\n";
                return 1;
            }
            try {
                startTimeSeconds = std::stod(argv[++i]);
            } catch (...) {
                std::cerr << "[ERROR] Invalid --start-time value: " << argv[i] << "\n";
                return 1;
            }
        } else if (arg == "--output-name") {
            if (i + 1 >= argc) {
                std::cerr << "[ERROR] --output-name requires a filename (e.g. my_trial.mot).\n";
                return 1;
            }
            outputName = argv[++i];
            if (outputName.empty()) {
                std::cerr << "[ERROR] --output-name cannot be empty.\n";
                return 1;
            }
            if (outputName.find('/') != std::string::npos) {
                std::cerr << "[ERROR] --output-name must be a filename only (no '/'). Use --output-file for full paths.\n";
                return 1;
            }
        } else if (arg.rfind("--output-name=", 0) == 0) {
            outputName = arg.substr(std::string("--output-name=").size());
            if (outputName.empty()) {
                std::cerr << "[ERROR] --output-name cannot be empty.\n";
                return 1;
            }
            if (outputName.find('/') != std::string::npos) {
                std::cerr << "[ERROR] --output-name must be a filename only (no '/'). Use --output-file for full paths.\n";
                return 1;
            }
        } else if (arg == "--output-file") {
            if (i + 1 >= argc) {
                std::cerr << "[ERROR] --output-file requires a full output path.\n";
                return 1;
            }
            outputFilePath = argv[++i];
            if (outputFilePath.empty()) {
                std::cerr << "[ERROR] --output-file cannot be empty.\n";
                return 1;
            }
        } else if (arg.rfind("--output-file=", 0) == 0) {
            outputFilePath = arg.substr(std::string("--output-file=").size());
            if (outputFilePath.empty()) {
                std::cerr << "[ERROR] --output-file cannot be empty.\n";
                return 1;
            }
        } else if (arg.rfind("--start-time=", 0) == 0) {
            const std::string val = arg.substr(std::string("--start-time=").size());
            try {
                startTimeSeconds = std::stod(val);
            } catch (...) {
                std::cerr << "[ERROR] Invalid --start-time value: " << val << "\n";
                return 1;
            }
        } else if (arg.size() >= 4 && arg.substr(arg.size() - 4) == ".xml") {
            ikTasksPath = arg;
        } else {
            outputDir = arg;
        }
    }
    if (startTimeSeconds < 0.0) {
        std::cerr << "[ERROR] --start-time must be >= 0.\n";
        return 1;
    }

    if (!outputDir.empty() && outputDir.back() == '/')
        outputDir.pop_back();

    std::string outputMotPath;
    if (!outputFilePath.empty()) {
        outputMotPath = outputFilePath;
        const auto slashPos = outputMotPath.find_last_of('/');
        if (slashPos != std::string::npos) {
            outputDir = outputMotPath.substr(0, slashPos);
            if (outputDir.empty()) outputDir = "/";
        }
    } else {
        outputMotPath = outputDir + "/" + outputName;
    }

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
        if (startTimeSeconds > 0.0) {
            std::cerr << "[ERROR] --start-time is currently supported only in solver mode (without --iktool).\n";
            return 1;
        }
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
        std::cout << "[DEBUG] Parity mode enabled: persistent IK tracking.\n";
    }

    // ======================
    //  IK TASKS
    // ======================
    if (!ikTasksPath.empty()) {
        ikSolver.setInverseKinematicsTaskSet(ikTasksPath);
        std::cout << "[DEBUG] Loaded IK tasks from: " << ikTasksPath << "\n";
    } else {
        // Build an automatic task set from TRC marker availability:
        // present markers -> weight 1; missing markers -> disabled.
        try {
            const auto trcLabels = getTrcMarkerLabels(trcPath);
            OpenSim::IKTaskSet autoTasks;
            int presentCount = 0;
            int missingCount = 0;
            for (const auto& markerName : modelMarkerNames) {
                auto* t = new OpenSim::IKMarkerTask();
                t->setName(markerName);
                const bool present = (trcLabels.find(markerName) != trcLabels.end());
                t->setApply(present);
                t->setWeight(present ? 1.0 : 0.0);
                autoTasks.adoptAndAppend(t);
                if (present) ++presentCount;
                else ++missingCount;
            }

            const std::string autoIkTasksPath = outputDir + "/auto_ik_tasks.xml";
            autoTasks.print(autoIkTasksPath);
            ikSolver.setInverseKinematicsTaskSet(autoIkTasksPath);
            std::cout << "[DEBUG] Auto IK tasks written to: " << autoIkTasksPath << "\n";
            std::cout << "[DEBUG] Marker tasks: present=" << presentCount
                      << ", missing/disabled=" << missingCount << "\n";
        } catch (const std::exception& e) {
            std::cout << "[WARN] Failed to auto-build IK tasks from TRC labels: "
                      << e.what() << "\n";
            std::cout << "[DEBUG] Falling back to solver defaults (all marker weights = 1).\n";
        }
    }

    // Producer thread (TRC) + IK thread
    std::thread producerThread([&](){
        try {
            trcProducer(trcPath, markerQueue, true, startTimeSeconds);
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
                OneEuroFilter(
                    /*minCutoff*/kJointFilterMinCutoff,
                    /*beta*/kJointFilterBeta,
                    /*dCutoff*/kJointFilterDCutoff
                )
            );
            jointFiltersInitialized = true;
        }

        for (int i = 0; i < (int)qVals.size(); ++i) {
            qVals[i] = jointFilters[i].filter(qVals[i], result.time);
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
