#pragma once

// Profiling instrumentation for the TEB control loop (performance work, target: 100 Hz).
//
// Activation follows the nav2_mppi_controller convention (compile-time macro):
// uncomment the define below, or build with -DBENCHMARK_TESTING.
// When disabled, all PROFILE_* macros expand to nothing -> zero overhead.

// #define BENCHMARK_TESTING

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nav2_teb_controller {

#ifdef BENCHMARK_TESTING

class TebProfiler {
public:
  // Counts control-loop ticks; report() fires once the window is full.
  void tick() { ++tick_; }

  void recordBlock(const std::string &name, int64_t elapsed_ns) {
    auto &acc = blocks_[name];
    ++acc.count;
    acc.total_ns += elapsed_ns;
    acc.max_ns = std::max(acc.max_ns, elapsed_ns);
  }

  // Returns a formatted summary table once the window is full (and resets the window),
  // otherwise an empty string.
  std::string report() {
    if (tick_ < kReportEveryTicks)
      return {};
    tick_ = 0;

    std::vector<std::pair<std::string, Accum>> items(blocks_.begin(), blocks_.end());
    std::sort(items.begin(), items.end(),
              [](const auto &a, const auto &b) { return a.second.total_ns > b.second.total_ns; });

    std::string out;
    char line[256];
    const auto it_total = blocks_.find("total");
    if (it_total != blocks_.end() && it_total->second.count > 0) {
      const double avg_ms = it_total->second.total_ns / 1e6 / it_total->second.count;
      std::snprintf(line, sizeof(line), "ticks=%u  avg_total=%.2fms (%.1fHz)  max_total=%.2fms\n",
                    kReportEveryTicks, avg_ms, avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0,
                    it_total->second.max_ns / 1e6);
      out += line;
    }
    for (const auto &[name, acc] : items) {
      const double total_ms = acc.total_ns / 1e6;
      const double avg_ms = total_ms / static_cast<double>(acc.count);
      std::snprintf(line, sizeof(line), "  %-24s %6llu %10.3f %9.3f %9.3f\n", name.c_str(),
                    static_cast<unsigned long long>(acc.count), total_ms, avg_ms,
                    acc.max_ns / 1e6);
      out += line;
    }
    blocks_.clear();
    return out;
  }

private:
  struct Accum {
    uint64_t count = 0;
    int64_t total_ns = 0;
    int64_t max_ns = 0;
  };
  std::unordered_map<std::string, Accum> blocks_;
  uint32_t tick_ = 0;
  static constexpr uint32_t kReportEveryTicks = 100;
};

inline TebProfiler &tebProfiler() {
  static TebProfiler p;
  return p;
}

// RAII scoped timer; records elapsed time into the profiler on destruction.
class ProfileBlock {
public:
  explicit ProfileBlock(std::string name)
      : name_(std::move(name)), start_(std::chrono::steady_clock::now()) {}

  ~ProfileBlock() {
    const int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now() - start_)
                                   .count();
    tebProfiler().recordBlock(name_, elapsed_ns);
  }

  ProfileBlock(const ProfileBlock &) = delete;
  ProfileBlock &operator=(const ProfileBlock &) = delete;

private:
  std::string name_;
  std::chrono::steady_clock::time_point start_;
};

#define PROFILE_BLOCK(name) ::nav2_teb_controller::ProfileBlock profile_block_##__LINE__(name)
#define PROFILE_TICK() ::nav2_teb_controller::tebProfiler().tick()
#define PROFILE_REPORT() ::nav2_teb_controller::tebProfiler().report()

#else  // !BENCHMARK_TESTING

#define PROFILE_BLOCK(name) ((void)0)
#define PROFILE_TICK() ((void)0)
#define PROFILE_REPORT() (std::string())

#endif  // BENCHMARK_TESTING

}  // namespace nav2_teb_controller
