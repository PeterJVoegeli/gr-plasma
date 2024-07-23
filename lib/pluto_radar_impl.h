/* -*- c++ -*- */
/*
 * Copyright 2024 gr-plasma author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PLASMA_PLUTO_RADAR_IMPL_H
#define INCLUDED_PLASMA_PLUTO_RADAR_IMPL_H

#include <gnuradio/plasma/pmt_constants.h>
#include <gnuradio/plasma/pluto_radar.h>
#include <nlohmann/json.hpp>
#include <gnuradio/iio/api.h>
#include <uhd/convert.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <queue>

#include <string>
#include <thread>
#include <vector>

#include <volk/volk_alloc.hh>

#define MIN_RATE 520333
#define DECINT_RATIO 8
#define OVERFLOW_CHECK_PERIOD_MS 1000


namespace gr {
namespace plasma {
class pluto_radar_impl :  public pluto_radar
    

{
// usrp radar private
private:
    // Block params
    uhd::usrp::multi_usrp::sptr usrp;
    std::string usrp_args;
    double tx_rate, rx_rate;
    double tx_freq, rx_freq;
    double tx_gain, rx_gain;
    double start_delay;
    bool elevate_priority;
    std::string cal_file;
    std::vector<size_t> tx_channel_nums, rx_channel_nums;
    std::string tx_subdev, rx_subdev;
    std::string tx_device_addr, rx_device_addr;
    std::string tx_cpu_format, rx_cpu_format;
    std::string tx_otw_format, rx_otw_format;
    bool verbose;
    //std::string& uri; i do not 
    const std::vector<bool>& ch_en;
    unsigned long buffer_size;
    bool cyclic;
    size_t n_delay;
    
    // Implementation params
    gr::thread::thread d_main_thread;
    boost::thread_group d_tx_rx_thread_group;
    std::vector<const void*> tx_buffs;
    size_t tx_buff_size;
    std::atomic<bool> finished;
    std::atomic<bool> msg_received;
    size_t n_tx_total;
    
    pmt::pmt_t tx_data;
    pmt::pmt_t next_meta; // Metadata for the next Rx pdu
    std::atomic<bool> new_msg_received;


    // Metadata keys
    std::string tx_freq_key;
    std::string rx_freq_key;
    std::string sample_start_key;
    
    void handle_msg(pmt::pmt_t msg);
    void config_pluto(  //uhd::usrp::multi_usrp::sptr& usrp,    //
                        //const std::string& args,
                     const double tx_rate,
                     const double rx_rate,
                     const double tx_freq,
                     const double rx_freq,
                     const double tx_gain,
                     const double rx_gain,
                     const std::string& tx_subdev,
                     const std::string& rx_subdev,
                     bool verbose,
                     iio_context* ctx,
                     const std::vector<bool>& ch_en,
                     unsigned long buffer_size,
                     bool cyclic);
/*    void receive(uhd::usrp::multi_usrp::sptr usrp,
                 uhd::rx_streamer::sptr rx_stream,
                 std::atomic<bool>& finished,
                 bool elevate_priority,
                 double adjusted_rx_delay,
                 bool rx_stream_now);
    void transmit(uhd::usrp::multi_usrp::sptr usrp,
                  uhd::tx_streamer::sptr tx_stream,
                  std::atomic<bool>& finished,
                  bool elevate_priority,
                  double tx_delay,
                  double has_time_spec);    */
    void read_calibration_file(const std::string& filename);
    void set_metadata_keys(const std::string& tx_freq_key,
                                   const std::string& rx_freq_key,
                                   const std::string& sample_start_key);
    
//fmcomms2 source private
    std::vector<std::string>
    source_get_channels_vector(bool ch1_en, bool ch2_en, bool ch3_en, bool ch4_en);
    std::vector<std::string> source_get_channels_vector(const std::vector<bool>& ch_en);
    std::thread overflow_thd;
    void check_overflow(void);

    const static int s_initial_device_buf_size = 8192;
    std::vector<volk::vector<short>> d_device_bufs;
    gr_vector_void_star d_device_item_ptrs;
    volk::vector<float> d_float_rvec;
    volk::vector<float> d_float_ivec;    
    
// fmcomms2 sink private
    bool stop_thread;

    std::mutex uf_mutex;
    std::thread underflow_thd;
    std::vector<std::string>

    sink_get_channels_vector(bool ch1_en, bool ch2_en, bool ch3_en, bool ch4_en);
    std::vector<std::string> sink_get_channels_vector(const std::vector<bool>& ch_en);
    void check_underflow(void);

//    const static int s_initial_device_buf_size = 8192;

//    std::vector<volk::vector<short>> d_device_bufs;
//    gr_vector_const_void_star d_device_item_ptrs;
    volk::vector<float> d_float_r;
    volk::vector<float> d_float_i;




protected:
//  device sink and source
//    gr::iio::iio_context* ctx;
  //  gr::iio::iio_device *dev, *phy;
    //gr::iio::iio_buffer* buf;
    
    std::vector<iio_channel*> channel_list;
    unsigned int interpolation;
    bool destroy_ctx;
    pmt::pmt_t d_len_tag_key;
    unsigned int decimation;
    volatile bool thread_stopped;
    
//fmcomms2 sink protected
    std::vector<double> d_attenuation = { 50.0, 50.0, 50.0, 50.0 };
    std::string d_rf_port_select = "A";
    std::string d_filter_source = "Auto";
    std::string d_filter_filename = "";
    float d_fpass = (float)d_samplerate / 4.0;
    float d_fstop = (float)d_samplerate / 3.0;
//fmcomms2 sink protected
//fmcomms2 source protected
    void source_update_dependent_params();

    unsigned long long d_frequency = 2400000000;
    unsigned long d_samplerate = 1000000;
    unsigned long d_bandwidth = 20000000;
    bool d_quadrature = true;
    bool d_rfdc = true;
    bool d_bbdc = true;
    std::vector<std::string> d_gain_mode = {
        "manual", "manual", "manual", "manual"
    }; // TODO - make these enums
    std::vector<double> d_gain_value = { 0, 0, 0, 0 };
  //      std::string d_rf_port_select = "A_BALANCED";

// fmcomms2 source protected


public:
    pluto_radar_impl(   //const std::string& args,
                    const double tx_rate,
                    const double rx_rate,
                    const double tx_freq,
                    const double rx_freq,
                    const double tx_gain,
                    const double rx_gain,
                    const double start_delay,
                    const bool elevate_priority,
                    const std::string& cal_file,
                    const bool verbose,
                    //const std::string& uri,
                    const std::vector<bool>& ch_en,
                    unsigned long buffer_size,
                    bool cyclic,
                    iio_context* ctx);
    //~pluto_radar_impl();

// Overloaded Constructors
    //Sink
  /*  pluto_radar_impl(iio_context* ctx,
                     bool destroy_ctx,
                     const std::string& device,
                     const std::vector<std::string>& channels,
                     const std::string& device_phy,
                     const gr::iio::iio_param_vec_t& params,
                     unsigned int buffer_size = DEFAULT_BUFFER_SIZE,
                     unsigned int interpolation = 0,
                     bool cyclic = false);

    //~pluto_radar_impl()
    
    //Source
    pluto_radar_impl(iio_context* ctx,
                       bool destroy_ctx,
                       const std::string& device,
                       const std::vector<std::string>& channels,
                       const std::string& device_phy,
                       const gr::iio::iio_param_vec_t& params,
                       unsigned int buffer_size = DEFAULT_BUFFER_SIZE,
                       unsigned int decimation = 0);    */

    ~pluto_radar_impl();
    
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
 //   void run();
    /**
     * @brief Start the main worker thread
     */
 //   bool start() override;
    /**
     * @brief Stop the main worker thread
     */
//    bool stop() override;

    /**
     * @brief Use the calibration file to determine the number of
     * samples to remove from the beginning of transmission
     *
     */
    /** void read_calibration_file(const std::string&) override;    */


    


    void sink_update_dependent_params();
    virtual void sink_set_len_tag_key(const std::string& len_tag_key);
    virtual void sink_set_bandwidth(unsigned long bandwidth);
    virtual void sink_set_rf_port_select(const std::string& rf_port_select);
    virtual void sink_set_frequency(double frequency);
    virtual void sink_set_samplerate(unsigned long samplerate);
    virtual void sink_set_attenuation(size_t chan, double gain);
    virtual void sink_set_filter_params(const std::string& filter_source,
                                   const std::string& filter_filename = "",
                                   float fpass = 0.0,
                                   float fstop = 0.0);


    void source(iio_context* ctx,
                         const std::vector<bool>& ch_en,
                         unsigned long buffer_size);


//    virtual void source_set_len_tag_key(const std::string& len_tag_key);
    virtual void source_set_frequency(double frequency);
    virtual void source_set_samplerate(unsigned long samplerate);
    virtual void source_set_gain_mode(size_t chan, const std::string& mode);
    virtual void source_set_gain(size_t chan, double gain_value);
    virtual void source_set_quadrature(bool quadrature);
    virtual void source_set_rfdc(bool rfdc);
    virtual void source_set_bbdc(bool bbdc);
    virtual void source_set_filter_params(const std::string& filter_source,
                                   const std::string& filter_filename,
                                   float fpass,
                                   float fstop);



    static void set_params(iio_device* phy, const gr::iio::iio_param_vec_t& params);

    void set_len_tag_key(const std::string& len_tag_key); //override;

    void set_params(const gr::iio::iio_param_vec_t& params);
    static bool load_fir_filter(std::string& filter, iio_device* phy);
    static int handle_decimation_interpolation(unsigned long samplerate,
                                               const char* channel_name,
                                               const char* attr_name,
                                               iio_device* dev,
                                               bool disable_dec,
                                               bool output_chan);
    void set_buffer_size(unsigned int buffer_size);
    iio_context* ctx;
    iio_device *dev, *phy;
    iio_buffer* buf;
//    static iio_context* get_context(const std::string& uri);
};  // plasma class


} // namespace plasma
} // namespace gr

#endif /* INCLUDED_PLASMA_USRP_RADAR_IMPL_H */
