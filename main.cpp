#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <cmath>

#include <csignal>

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

// OpenSim
#include <OpenSim/OpenSim.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>

// Concurrency
#include <rtb/concurrency/Latch.h>

// ==========================================================
//                 ONE EURO FILTER (JOINTS)
// ==========================================================
class OneEuroFilter {
public:
    OneEuroFilter(double minCutoff = 4.0, double beta = 0.02, double dCutoff = 1.0)
        : minCutoff(minCutoff), beta(beta), dCutoff(dCutoff),
          initialized(false), prevValue(0.0), prevDeriv(0.0), prevTime(0.0) {}

    double filter(double x, double t) {
        if (!initialized) {
            initialized = true;
            prevValue = x;
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
    double prevValue, prevDeriv, prevTime;
    double minCutoff, beta, dCutoff;

    double computeAlpha(double cutoff, double dt) {
        const double PI = 3.14159265358979323846;
        double tau = 1.0 / (2.0 * PI * cutoff);
        return 1.0 / (1.0 + tau / dt);
    }
};

// ==========================================================
//              MARKER FILTER (OUTLIERS + SMOOTH)
// ==========================================================
struct MarkerFilter {
    struct MarkerState {
        bool initialized = false;
        SimTK::Vec3 lastRaw;
        SimTK::Vec3 xHat;
        double lastTime = 0.0;
    };

    std::vector<MarkerState> states;
    double maxJump, maxVelocity, alpha;

    MarkerFilter(int n, double mj = 0.18, double mv = 6.0, double a = 0.6)
        : states(n), maxJump(mj), maxVelocity(mv), alpha(a) {}

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
        double vel = jump / dt;
        bool outlier = (jump > maxJump) || (vel > maxVelocity);

        SimTK::Vec3 input = outlier ? s.xHat : raw;
        s.xHat = alpha * input + (1.0 - alpha) * s.xHat;

        s.lastRaw = raw;
        s.lastTime = t;
        return s.xHat;
    }
};

// ==========================================================
//                    SOCKET RECEIVER
// ==========================================================
static constexpr int PORT = 5555;
static constexpr int NUM_MARKERS = 20;
static constexpr int FRAME_SIZE = NUM_MARKERS * 3 * sizeof(float);

void socketReceiver(rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame>& markerQueue) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(server_fd, (sockaddr*)&address, sizeof(address));
    listen(server_fd, 1);

    int client_fd = accept(server_fd, nullptr, nullptr);

    MarkerFilter filter(NUM_MARKERS);
    std::vector<float> buffer(NUM_MARKERS * 3);
    int frameCount = 0;

    while (true) {
        int n = read(client_fd, buffer.data(), FRAME_SIZE);
        if (n <= 0) break;

        rtosim::MarkerSetFrame frame;
        frame.time = frameCount * (1.0 / 30.0);
        double t = frame.time;

        for (int i = 0; i < NUM_MARKERS; ++i) {
            SimTK::Vec3 raw(buffer[3*i], buffer[3*i+1], buffer[3*i+2]);
            frame.data.push_back(filter.filterOne(i, raw, t));
        }

        markerQueue.push(frame);
        frameCount++;
    }

    markerQueue.push(rtosim::EndOfData::get<rtosim::MarkerSetFrame>());
    close(client_fd);
    close(server_fd);
}

// ==========================================================
//                           MAIN
// ==========================================================
#include <csignal>   // +++
#include <string>    // +++ (au cas où)

// ...

int main(int argc, char** argv) {
    // Avoid SIGPIPE killing the process (Simbody still throws, but we handle it)
    std::signal(SIGPIPE, SIG_IGN);

    if (argc < 2) {
        std::cerr << "Usage: ./online_ik_test [--no-viz] <model.osim>\n";
        return 1;
    }

    // -------- Parse args --------
    bool useViz = true;
    std::string modelPath;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--no-viz") {
            useViz = false;
        } else if (!a.empty() && a[0] == '-') {
            std::cerr << "[warn] Unknown flag: " << a << "\n";
        } else {
            modelPath = a;
        }
    }

    if (modelPath.empty()) {
        std::cerr << "Usage: ./online_ik_test [--no-viz] <model.osim>\n";
        return 1;
    }

    std::cout << "[Init] Using model: " << modelPath << std::endl;

    rtosim::ThreadPoolJobs<rtosim::MarkerSetFrame> markerQueue;
    rtosim::IKoutputs<rtosim::GeneralisedCoordinatesFrame> outputQueue;
    rtb::Concurrency::Latch doneWithSubscriptions(1), doneWithExecution(1);

    // -------- Load model (single-thread) --------
    OpenSim::Model model(modelPath);
    model.setUseVisualizer(useViz);
    SimTK::State state = model.initSystem();

    // We keep a pointer so we can compile even when --no-viz is used
    SimTK::Visualizer* simbodyViz = nullptr;

    if (useViz) {
        auto& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(SimTK::Visualizer::GroundAndSky);
        simbodyViz = &viz;

        // ------------------------------------------------------------------
        // CRITICAL: warm-up report BEFORE creating any threads.
        // This forces the visualizer process to spawn while we're single-threaded.
        // ------------------------------------------------------------------
        try {
            model.realizePosition(state);
            simbodyViz->report(state);
            std::cout << "[viz] warm-up report ok\n";
        } catch (const SimTK::Exception::Base& e) {
            std::cerr << "[viz] warm-up failed, disabling viz: " << e.what() << "\n";
            useViz = false;
            simbodyViz = nullptr;
        } catch (const std::exception& e) {
            std::cerr << "[viz] warm-up failed, disabling viz: " << e.what() << "\n";
            useViz = false;
            simbodyViz = nullptr;
        } catch (...) {
            std::cerr << "[viz] warm-up failed (unknown), disabling viz\n";
            useViz = false;
            simbodyViz = nullptr;
        }
    }

    // -------- IK SOLVER (creates its own model internally from modelPath) --------
    rtosim::IKSolverParallel ikSolver(
        markerQueue, outputQueue,
        doneWithSubscriptions, doneWithExecution,
        modelPath, 1e-4, 10.0
    );

    // ---- Custom IK marker weights ----
    OpenSim::IKTaskSet ikTasks;
    auto addTask = [&](const std::string& n, double w) {
        auto* t = new OpenSim::IKMarkerTask();
        t->setName(n); t->setApply(true); t->setWeight(w);
        ikTasks.adoptAndAppend(t);
    };

    addTask("ACD",1.0); addTask("CLAD",1.0); addTask("CLAG",1.0);
    addTask("MAN",0.8); addTask("XYP",0.8); addTask("T8",0.8); addTask("C7",0.8);
    addTask("MTACM",0.2); addTask("MTACB",0.2); addTask("MTACL",0.2);
    addTask("MTHA",0.3); addTask("MTHP",0.25); addTask("MTBA",0.25); addTask("MTBP",0.25);
    addTask("EL",0.0); addTask("EM",0.5);
    addTask("PSU",0.8); addTask("PSR",0.8);
    addTask("MC5",0.9); addTask("MC2",0.9);

    ikSolver.setInverseKinematicsTaskSet(ikTasks);

    // -------- Start threads AFTER viz warm-up --------
    std::thread ikThread(std::ref(ikSolver));
    std::thread socketThread(socketReceiver, std::ref(markerQueue));

    // -------- MAIN LOOP --------
    std::vector<OneEuroFilter> jointFilters;
    bool filtersInit = false;
    bool viz_ok = (simbodyViz != nullptr);

    while (true) {
        auto res = outputQueue.pop();
        if (rtosim::EndOfData::isEod(res)) break;

        auto q = res.data.getQ();

        // Init filters once we know q size
        if (!filtersInit) {
            jointFilters.resize(q.size());
            filtersInit = true;
        }

        // Filter joints
        for (size_t i = 0; i < q.size(); ++i)
            q[i] = jointFilters[i].filter(q[i], res.time);

        // Safety: skip non-finite q (prevents visualizer/realize issues)
        bool bad = false;
        for (double v : q) {
            if (!std::isfinite(v)) { bad = true; break; }
        }
        if (bad) {
            std::cerr << "[warn] Non-finite q at t=" << res.time << " (skipping frame)\n";
            continue;
        }

        auto& coords = model.updCoordinateSet();
        if ((int)q.size() != coords.getSize()) {
            std::cerr << "[warn] q.size()=" << q.size()
                      << " != coords=" << coords.getSize() << " (skipping frame)\n";
            continue;
        }

        for (int i = 0; i < coords.getSize(); ++i)
            coords[i].setValue(state, q[i]);

        model.realizePosition(state);

        // Robust viz reporting
        if (viz_ok) {
            try {
                simbodyViz->report(state);
            } catch (const SimTK::Exception::Base& e) {
                std::cerr << "[viz] report failed, disabling viz: " << e.what() << "\n";
                viz_ok = false;
            } catch (const std::exception& e) {
                std::cerr << "[viz] report failed, disabling viz: " << e.what() << "\n";
                viz_ok = false;
            } catch (...) {
                std::cerr << "[viz] report failed (unknown), disabling viz\n";
                viz_ok = false;
            }
        }
    }

    socketThread.join();
    ikThread.join();

    if (useViz) {
        std::cout << "Press ENTER to close visualizer..." << std::endl;
        std::cin.get();
    }

    return 0;
}

