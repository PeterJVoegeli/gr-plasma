/* -*- c++ -*- */
/*
 * Copyright 2022 gr-plasma author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PLASMA_USRP_RADAR_IMPL_H
#define INCLUDED_PLASMA_USRP_RADAR_IMPL_H

#include <gnuradio/plasma/pmt_dict_keys.h>
#include <gnuradio/plasma/usrp_radar.h>
#include <nlohmann/json.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <fstream>
#include <queue>

namespace gr {
namespace plasma {
static const double BURST_MODE_DELAY = 2e-6;
class usrp_radar_impl : public usrp_radar
{
private:
    uhd::usrp::multi_usrp::sptr d_usrp;
    double d_samp_rate;
    double d_tx_gain;
    double d_rx_gain;
    double d_tx_freq;
    double d_rx_freq;
    double d_prf;
    size_t d_delay_samps;
    size_t d_pulse_count;
    size_t d_sample_count;
    uhd::time_spec_t d_start_time;
    std::string d_tx_args, d_rx_args;
    std::vector<gr_complex> d_tx_buff;
    std::vector<gr_complex> d_rx_buff;
    pmt::pmt_t d_meta;
    pmt::pmt_t d_rx_data;
    gr::thread::thread d_main_thread;
    gr::thread::thread d_tx_thread;
    gr::thread::mutex d_tx_buff_mutex;
    std::atomic<bool> d_finished;
    std::atomic<bool> d_armed;
    double d_tx_thread_priority;
    double d_rx_thread_priority;


    /**
     * @brief Transmit the data in the tx buffer until
     *
     * @param usrp
     * @param buff_ptrs
     * @param num_samps_pulse
     * @param start_time
     */
    void transmit(uhd::usrp::multi_usrp::sptr usrp,
                  std::vector<std::complex<float>*> buff_ptrs,
                  size_t num_samps_pulse,
                  uhd::time_spec_t start_time);
    /**
     * @brief Receive a CPI of samples from the USRP, then package them into a PDU
     *
     * @param usrp
     * @param buff_ptrs
     * @param num_samp_cpi
     * @param start_time
     */
    void receive(uhd::usrp::multi_usrp::sptr usrp,
                 std::vector<std::complex<float>*> buff_ptrs,
                 size_t num_samp_cpi,
                 uhd::time_spec_t start_time);

    /**
     * @brief Receive a pulse of samples from the USRP, then package them into a PDU
     *
     * @param usrp
     * @param buff_ptrs
     * @param start_time
     */
    void receive(uhd::usrp::multi_usrp::sptr usrp,
                 std::vector<std::vector<gr_complex>> buff_ptrs,
                 uhd::time_spec_t start_time);

public:
    usrp_radar_impl(double samp_rate,
                    double tx_gain,
                    double rx_gain,
                    double tx_freq,
                    double rx_freq,
                    double start_time,
                    const std::string& tx_args,
                    const std::string& rx_args);
    ~usrp_radar_impl();

    /**
     * @brief
     *
     * @param msg
     */
    void handle_message(const pmt::pmt_t& msg);

    /**
     * @brief Initialize all buffers and set up the transmit and recieve threads
     *
     */
    void run();
    /**
     * @brief Start the main worker thread
     */
    bool start() override;
    /**
     * @brief Stop the main worker thread
     */
    bool stop() override;

    /**
     * @brief Use ~/.uhd/delay_calibration.json to determine the number of
     * samples to remove from the beginning of transmission
     *
     */
    void read_calibration_json();

    void set_tx_thread_priority(const double) override;

    void set_rx_thread_priority(const double) override;
};

} // namespace plasma
} // namespace gr

#endif /* INCLUDED_PLASMA_USRP_RADAR_IMPL_H */
