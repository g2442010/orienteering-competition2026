// Local benchmark for equations (9)-(12), called from main.cpp.
#include "rating.h"
#include "const.h"
#include "csv_loader.h"
#include "ga.h"
#include "graph.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <vector>

using namespace orienteering;

namespace {
constexpr int RUNS = 10;
struct Run { unsigned seed; double fitness; double seconds; };
struct Summary { std::string name; double ave; double sd; double time; };

// Suppress per-generation console output while timing; restore even on errors.
class QuietOutput : public std::streambuf {
    std::streambuf* previous;
    int_type overflow(int_type c) override { return traits_type::not_eof(c); }
public:
    QuietOutput() : previous(std::cout.rdbuf(this)) {}
    ~QuietOutput() override { std::cout.rdbuf(previous); }
};

void validate(const Run& r) {
    if (!std::isfinite(r.fitness) || r.fitness < 0 || r.fitness >= PENALTY ||
        !std::isfinite(r.seconds) || r.seconds < 0)
        throw std::runtime_error("Invalid fitness/time (including infeasible solution).");
}

Summary summarize(const std::string& name, const std::vector<Run>& runs) {
    if (runs.size() != RUNS) throw std::runtime_error("Exactly 10 runs are required: " + name);
    // Welford's method; population SD (divide by 10, not 9).
    double mean = 0, m2 = 0, time = 0;
    int n = 0;
    for (const auto& r : runs) {
        validate(r);
        ++n;
        const double delta = r.fitness - mean;
        mean += delta / n;
        m2 += delta * (r.fitness - mean);
        time += (r.seconds - time) / n;
    }
    const double sd = std::sqrt(std::max(0.0, m2 / RUNS));
    if (!std::isfinite(mean) || !std::isfinite(m2) || !std::isfinite(sd))
        throw std::runtime_error("Statistics overflow: " + name);
    return {name, mean, sd, time};
}

std::vector<Run> measure(bool fixed_seed) {
    std::vector<Run> runs;
    for (int i = 0; i < RUNS; ++i) {
        const unsigned seed = RANDOM_SEED + (fixed_seed ? 0u : static_cast<unsigned>(i));
        Run measurement{seed, 0, 0};
        {
            QuietOutput quiet;
            const auto start = std::chrono::steady_clock::now();
            // Rebuild inputs and shortest-path cache for every independent run.
            auto landmarks = load_landmarks(DATA_DIR + "/landmarks.csv");
            auto nodes = load_nodes(DATA_DIR + "/nodes.csv");
            auto edges = load_edges(DATA_DIR + "/edges.csv");
            auto gate = load_gate(DATA_DIR + "/seimon.csv");
            if (landmarks.size() < MAX_CONTROLS || nodes.empty())
                throw std::runtime_error("Insufficient landmarks/nodes.");
            Graph graph(nodes, edges);
            const auto gate_node = graph.find_nearest_node(gate.lat, gate.lon);
            std::set<long long> source_set{gate_node};
            for (const auto& lm : landmarks) source_set.insert(lm.nearest_node);
            std::vector<long long> sources(source_set.begin(), source_set.end());
            PathCache cache(graph, sources);
            RNG rng(seed);
            auto result = run_ga(landmarks, cache, gate_node, rng);
            measurement.seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (!result.best_eval.is_valid) throw std::runtime_error("No feasible solution.");
            measurement.fitness = result.best_eval.fitness;
        }
        validate(measurement);
        runs.push_back(measurement);
        std::cout << "Run " << i + 1 << "/10: seed=" << seed
                  << " fitness=" << measurement.fitness
                  << " seconds=" << measurement.seconds << '\n';
    }
    return runs;
}

void save(const std::string& path, const std::vector<Run>& runs) {
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream out(path);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << "seed,fitness,seconds\n" << std::setprecision(17);
    for (const auto& r : runs) out << r.seed << ',' << r.fitness << ',' << r.seconds << '\n';
    out.close();
}

std::vector<Run> read(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);
    std::string line;
    std::getline(in, line);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "seed,fitness,seconds") throw std::runtime_error("Invalid CSV header: " + path);
    std::vector<Run> runs;
    while (std::getline(in, line)) {
        std::istringstream row(line);
        Run r{};
        char a = 0, b = 0;
        if (!(row >> r.seed >> a >> r.fitness >> b >> r.seconds) || a != ',' || b != ',')
            throw std::runtime_error("Invalid CSV row: " + path);
        row >> std::ws;
        if (!row.eof()) throw std::runtime_error("Extra CSV data: " + path);
        validate(r);
        runs.push_back(r);
    }
    return runs;
}

void rate(const std::vector<Summary>& summaries) {
    double max_ave = 0, max_sd = 0, max_time = 0;
    for (const auto& s : summaries) {
        max_ave = std::max(max_ave, s.ave);
        max_sd = std::max(max_sd, s.sd);
        max_time = std::max(max_time, s.time);
    }
    if (summaries.size() == 1)
        std::cout << "NOTE: One entry only; self-normalized E is not a comparative rating.\n";
    if (max_ave == 0 || max_sd == 0 || max_time == 0)
        std::cout << "NOTE: All-zero metrics use normalized value 0 (local convention for 0/0).\n";
    auto normalize = [](double value, double maximum) { return maximum == 0 ? 0 : value / maximum; };
    for (const auto& s : summaries) {
        const double a = normalize(s.ave, max_ave);
        const double d = normalize(s.sd, max_sd);
        const double t = normalize(s.time, max_time);
        std::cout << '\n' << s.name << "\nAve=" << s.ave << " SD=" << s.sd << " T=" << s.time
                  << "\nAve'=" << a << " SD'=" << d << " T'=" << t
                  << "\nE (12)=" << std::sqrt(a*a + d*d + t*t) << '\n';
    }
}
} // namespace

int orienteering::run_rating(int argc, char* argv[]) {
    try {
        std::cout << std::setprecision(10);
        if (argc > 1 && std::string(argv[1]) == "--compare") {
            if (argc < 4) throw std::runtime_error("Usage: orienteering --rating --compare before.csv after.csv [...]");
            std::vector<Summary> summaries;
            std::vector<unsigned> expected_seeds;
            for (int i = 2; i < argc; ++i) {
                const auto runs = read(argv[i]);
                std::vector<unsigned> seeds;
                for (const auto& r : runs) seeds.push_back(r.seed);
                if (i == 2) expected_seeds = seeds;
                else if (seeds != expected_seeds) throw std::runtime_error("Seed schedules differ between entries.");
                summaries.push_back(summarize(argv[i], runs));
            }
            rate(summaries);
        } else {
            std::string output = OUTPUT_DIR + "/rating_runs.csv";
            bool fixed = false;
            for (int i = 1; i < argc; ++i) {
                const std::string arg = argv[i];
                if (arg == "--fixed-seed") fixed = true;
                else if (arg == "--output" && i + 1 < argc) output = argv[++i];
                else if (arg == "--help") {
                    std::cout << "orienteering --rating [--output FILE] [--fixed-seed]\n"
                              << "orienteering --rating --compare FILE1 FILE2 [...]\n";
                    return 0;
                } else throw std::runtime_error("Unknown/incomplete argument: " + arg);
            }
            std::cout << "10 runs; SD divisor=10; seeds=" << (fixed ? "fixed" : "base+i")
                      << "\nTiming: CSV load + graph/cache construction + GA; excludes output and process startup.\n";
            const auto runs = measure(fixed);
            const auto summary = summarize(output, runs);
            save(output, runs);
            std::cout << "Saved: " << output << '\n';
            rate({summary});
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "rating error: " << e.what() << '\n';
        return 1;
    }
}
