// Isolated manager setup cost experiment: NOT message RTT or an end-to-end
// robotics benchmark. Cold = create mapping + first epoch; warm = same region,
// new epoch, no allocation. Does not compare independent implementation paths.
#include "robotcore/persistent_shm.hpp"
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
struct Options {
  std::string csv;
  std::uint64_t samples{1000};
  std::uint64_t warmup{50};
  std::size_t capacity{1024};
};
std::uint64_t positive_number(const std::string& value, std::uint64_t max) {
  if (value.empty() || value[0] == '-') throw std::invalid_argument("positive integer required");
  std::size_t used = 0;
  const auto result = std::stoull(value, &used, 10);
  if (used != value.size() || result == 0 || result > max)
    throw std::invalid_argument("out-of-range integer: " + value);
  return result;
}
Options options(int argc, char** argv) {
  Options result;
  for (int i=1; i<argc; i+=2) {
    if (i+1>=argc) throw std::invalid_argument("missing argument value");
    const std::string key=argv[i], val=argv[i+1];
    if (key=="--csv") result.csv=val;
    else if (key=="--samples") result.samples=positive_number(val, 100000);
    else if (key=="--warmup") result.warmup=positive_number(val, 10000);
    else if (key=="--capacity") result.capacity=positive_number(val, robotcore::kMaxPayloadBytes);
    else throw std::invalid_argument("unknown argument: "+key);
  }
  if (result.csv.empty()) throw std::invalid_argument("--csv path is required");
  if (result.warmup > 100000 - result.samples)
    throw std::invalid_argument("too many total iterations");
  return result;
}
double elapsed_us(Clock::time_point start, Clock::time_point stop) {
  return std::chrono::duration<double,std::micro>(stop-start).count();
}
int run(const Options& opts) {
  std::ofstream out(opts.csv, std::ios::trunc);
  if (!out) throw std::runtime_error("cannot create CSV");
  out << "iteration,phase,capacity_bytes,duration_us,epoch\n";
  const std::uint64_t total=opts.samples+opts.warmup;
  for (std::uint64_t i=0; i<total; ++i) {
    const std::string name="/robotcore-setup-"+std::to_string(::getpid())+
                           "-"+std::to_string(i);
    robotcore::PersistentRegionManager manager(name, opts.capacity);
    const auto started=Clock::now();
    auto status=manager.start();
    if (!status) throw std::runtime_error("create mapping: "+status.detail);
    status=manager.begin_session();
    if (!status) throw std::runtime_error("first epoch: "+status.detail);
    const auto cold_done=Clock::now();
    if (manager.epoch()!=1) throw std::runtime_error("wrong cold epoch");
    status=manager.begin_session();
    if (!status) throw std::runtime_error("reuse mapping: "+status.detail);
    const auto warm_done=Clock::now();
    if (manager.epoch()!=2) throw std::runtime_error("warm epoch did not advance");
    status=manager.stop();
    if (!status) throw std::runtime_error("manager cleanup: "+status.detail);
    if (i < opts.warmup) continue;
    const auto iteration=i-opts.warmup+1;
    out << iteration << ",cold_create_and_first_epoch," << opts.capacity << ','
        << std::fixed << std::setprecision(3) << elapsed_us(started,cold_done) << ",1\n";
    out << iteration << ",warm_reset_existing_region," << opts.capacity << ','
        << std::fixed << std::setprecision(3) << elapsed_us(cold_done,warm_done) << ",2\n";
  }
  out.flush();
  if (!out) throw std::runtime_error("CSV write failed");
  std::cout << "Recorded " << opts.samples << " setup pairs (not message RTT)\n";
  return 0;
}
}
int main(int argc, char** argv) {
  try { return run(options(argc, argv)); }
  catch (const std::exception& e) {
    std::cerr << "persistent setup experiment failed: " << e.what() << '\n';
    return 1;
  }
}
