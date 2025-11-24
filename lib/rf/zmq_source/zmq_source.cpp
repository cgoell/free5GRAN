#include "zmq_source.h"

#include <cstring>
#include <iostream>
#include <stdexcept>

#include <uhd/utils/thread.hpp>

using namespace std;

free5GRAN::zmq_source::zmq_source(const std::string& address,
                                  double sample_rate,
                                  double center_frequency,
                                  double gain,
                                  free5GRAN::rf_buffer* rf_buff)
    : context(1), socket(context, zmq::socket_type::sub) {
  this->address = address;
  this->sample_rate = sample_rate;
  this->center_frequency = center_frequency;
  this->gain = gain;
  this->bandwidth = sample_rate;
  this->rf_buff = rf_buff;

  try {
    socket.set(zmq::sockopt::subscribe, "");
    socket.connect(address);
  } catch (const zmq::error_t& err) {
    throw runtime_error("Could not connect to ZMQ publisher at " + address +
                        ": " + string(err.what()));
  }
}

auto free5GRAN::zmq_source::getSampleRate() -> double { return sample_rate; }

void free5GRAN::zmq_source::get_samples(vector<complex<float>>& buff,
                                        double& time_first_sample) {
  uhd::set_thread_priority_safe(1.0, true);
  lock_guard<mutex> lock(socket_mutex);

  size_t total_received = 0;
  while (total_received < buff.size()) {
    size_t just_received =
        receive_samples(buff, total_received, buff.size() - total_received);
    if (just_received == 0) {
      throw runtime_error("Unable to receive enough samples from ZMQ source");
    }
    total_received += just_received;
  }

  time_first_sample = static_cast<double>(sample_offset) / sample_rate;
  sample_offset += total_received;
}

void free5GRAN::zmq_source::setSampleRate(double rate) {
  this->sample_rate = rate;
  this->bandwidth = rate;
}

auto free5GRAN::zmq_source::getCenterFrequency() -> double {
  return this->center_frequency;
}

void free5GRAN::zmq_source::setCenterFrequency(double freq) {
  this->center_frequency = freq;
}

void free5GRAN::zmq_source::setGain(double gain) { this->gain = gain; }

auto free5GRAN::zmq_source::getGain() -> double { return this->gain; }

void free5GRAN::zmq_source::start_loopback_recv(bool& stop_signal,
                                               size_t buff_size) {
  uhd::set_thread_priority_safe(1.0, true);

  free5GRAN::buffer_element new_elem = {
      .frame_id = primary_frame_id,
      .buffer = vector<complex<float>>(buff_size),
      .overflow = false};

  cout << "Starting ZMQ source recv thread" << endl;

  bool notified_all = false;
  bool last_notify = false;

  while (!stop_signal) {
    if (primary_frame_id == 6000) {
      primary_frame_id = 0;
    }

    new_elem.overflow = false;
    size_t received = receive_samples(new_elem.buffer, 0, buff_size);
    if (received < buff_size) {
      new_elem.overflow = true;
    }
    sample_offset += received;

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
  cout << "Finishing ZMQ source recv thread" << endl;
}

size_t free5GRAN::zmq_source::receive_samples(
    vector<complex<float>>& buff, size_t offset, size_t count) {
  zmq::message_t message;
  size_t copied_samples = 0;

  try {
    if (!socket.recv(message, zmq::recv_flags::none)) {
      return 0;
    }
  } catch (const zmq::error_t& err) {
    cerr << "ZMQ receive error: " << err.what() << endl;
    return 0;
  }

  size_t available_samples = message.size() / sizeof(complex<float>);
  copied_samples = min(count, available_samples);

  if (copied_samples > 0) {
    memcpy(reinterpret_cast<char*>(&buff[offset]), message.data(),
           copied_samples * sizeof(complex<float>));
  }

  return copied_samples;
}
