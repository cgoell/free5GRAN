#ifndef FREE5GRAN_FILE_SOURCE_H
#define FREE5GRAN_FILE_SOURCE_H

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "../rf.h"

namespace free5GRAN {
class file_source : public rf {
 public:
  file_source(const std::string& path,
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
  std::ifstream sample_file;
  std::string file_path;
  std::mutex file_mutex;
  uint64_t sample_offset = 0;

  bool reopen_file();
  size_t read_samples(std::vector<std::complex<float>>& buff,
                      size_t offset,
                      size_t count);
};
}  // namespace free5GRAN

#endif  // FREE5GRAN_FILE_SOURCE_H
