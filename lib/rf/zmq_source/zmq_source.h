#ifndef FREE5GRAN_ZMQ_SOURCE_H
#define FREE5GRAN_ZMQ_SOURCE_H

#include <mutex>
#include <string>
#include <vector>

#include <zmq.hpp>

#include "../rf.h"

namespace free5GRAN {
class zmq_source : public rf {
 public:
  zmq_source(const std::string& address,
             double sample_rate,
             double center_frequency,
             double gain,
             free5GRAN::rf_buffer* rf_buff);

  void get_samples(std::vector<std::complex<float>>& buff,
                   double& time_first_sample) override;

  auto getSampleRate() -> double override;

  void setSampleRate(double rate) override;

  auto getCenterFrequency() -> double override;

  void setCenterFrequency(double freq) override;

  void setGain(double gain) override;

  auto getGain() -> double override;

  void start_loopback_recv(bool& stop_signal, size_t buff_size) override;

 private:
  zmq::context_t context;
  zmq::socket_t socket;
  std::string address;
  std::mutex socket_mutex;
  uint64_t sample_offset = 0;
  float gain_linear = 1.0f;

  size_t receive_samples(std::vector<std::complex<float>>& buff,
                         size_t offset,
                         size_t count);
};
}  // namespace free5GRAN

#endif  // FREE5GRAN_ZMQ_SOURCE_H
