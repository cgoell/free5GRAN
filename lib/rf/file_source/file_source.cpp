#include "file_source.h"

#include <iostream>
#include <stdexcept>

#include <uhd/utils/thread.hpp>

using namespace std;

free5GRAN::file_source::file_source(const std::string& path,
                                    double sample_rate,
                                    double center_frequency,
                                    double gain,
                                    free5GRAN::rf_buffer* rf_buff) {
  this->file_path = path;
  this->sample_rate = sample_rate;
  this->center_frequency = center_frequency;
  this->gain = gain;
  this->bandwidth = sample_rate;
  this->rf_buff = rf_buff;

  this->sample_file.open(this->file_path, ios::binary);
  if (!this->sample_file.is_open()) {
    throw runtime_error("Could not open IQ sample file: " + this->file_path);
  }
}

auto free5GRAN::file_source::getSampleRate() -> double { return sample_rate; }

void free5GRAN::file_source::get_samples(vector<complex<float>>& buff,
                                         double& time_first_sample) {
  uhd::set_thread_priority_safe(1.0, true);
  lock_guard<mutex> lock(file_mutex);

  size_t total_read = read_samples(buff, 0, buff.size());
  if (total_read < buff.size()) {
    throw runtime_error("Unable to read enough samples from file source");
  }

  time_first_sample = static_cast<double>(sample_offset) / sample_rate;
  sample_offset += total_read;
}

void free5GRAN::file_source::setSampleRate(double rate) {
  this->sample_rate = rate;
  this->bandwidth = rate;
}

auto free5GRAN::file_source::getCenterFrequency() -> double {
  return this->center_frequency;
}

void free5GRAN::file_source::setCenterFrequency(double freq) {
  this->center_frequency = freq;
}

void free5GRAN::file_source::setGain(double gain) { this->gain = gain; }

auto free5GRAN::file_source::getGain() -> double { return this->gain; }

void free5GRAN::file_source::start_loopback_recv(bool& stop_signal,
                                                 size_t buff_size) {
  uhd::set_thread_priority_safe(1.0, true);

  free5GRAN::buffer_element new_elem = {
      .frame_id = primary_frame_id,
      .buffer = vector<complex<float>>(buff_size),
      .overflow = false};

  cout << "Starting file source recv thread" << endl;

  bool notified_all = false;
  bool last_notify = false;

  while (!stop_signal) {
    {
      lock_guard<mutex> lock(file_mutex);
      size_t total_read = read_samples(new_elem.buffer, 0, buff_size);
      if (total_read < buff_size) {
        new_elem.overflow = true;
      } else {
        new_elem.overflow = false;
      }
      sample_offset += total_read;
    }

    new_elem.frame_id = primary_frame_id;
    rf_buff->primary_buffer->push_back(new_elem);

    if (last_notify) {
      (*rf_buff->cond_var_vec_prim_buffer)[rf_buff->primary_buffer->size() - 1]
          .notify_all();
      last_notify = false;
    }
    if (!notified_all) {
      (*rf_buff->cond_var_vec_prim_buffer)[rf_buff->primary_buffer->size() - 1]
          .notify_all();
      if (rf_buff->primary_buffer->size() ==
          rf_buff->primary_buffer->capacity() - 1) {
        last_notify = true;
        notified_all = true;
      }
    }

    if (rf_buff->frame_thread_started) {
      sem_post(rf_buff->semaphore);
    }

    primary_frame_id = (primary_frame_id + 1) % 6000;
  }
  primary_frame_id = 0;
  cout << "Finishing file source recv thread" << endl;
}

bool free5GRAN::file_source::reopen_file() {
  sample_file.clear();
  sample_file.seekg(0, ios::beg);
  return sample_file.good();
}

size_t free5GRAN::file_source::read_samples(vector<complex<float>>& buff,
                                            size_t offset,
                                            size_t count) {
  size_t samples_read = 0;
  while (samples_read < count) {
    sample_file.read(reinterpret_cast<char*>(&buff[offset + samples_read]),
                     (count - samples_read) * sizeof(complex<float>));
    size_t received = sample_file.gcount() / sizeof(complex<float>);
    samples_read += received;

    if (samples_read == count) {
      break;
    }

    if (!reopen_file()) {
      break;
    }
  }

  return samples_read;
}
