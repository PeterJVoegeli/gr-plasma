/* -*- c++ -*- */
/*
 * Copyright 2024 gr-plasma author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
 
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "pluto_radar_impl.h"
#include <gnuradio/io_signature.h>
#include <ad9361.h>

#include <string>
#include <thread>
#include <vector>

#include <volk/volk.h>


#define MIN_RATE 520333
#define DECINT_RATIO 8
#define OVERFLOW_CHECK_PERIOD_MS 1000

namespace gr {
namespace plasma {

/*      Notes
 *  Will need to check how the LO works for the rx and tx respectively

 * TO DO
 * How can the rf port select be found by the sink impl cc if there is no passing from the yml
 * 
 * Get message passing working  -Verify the port in actually receives msg data
 * Make rx into msg             Is thread necessary idk
 * Check over and underflow criteria + other error detections
 * Remember to tell the GRC that the block is a hardware block
 * 
 * + Current errors:
 *      undefined symbol from impl.h function

 */

pluto_radar::sptr pluto_radar::make(    //const std::string& args,
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
                                  const std::string& uri,
                                  const std::vector<bool>& ch_en,
                                  unsigned long buffer_size,
                                  bool cyclic)
{
    return gnuradio::make_block_sptr<pluto_radar_impl>( //args,
                                                      tx_rate,
                                                      rx_rate,
                                                      tx_freq,
                                                      rx_freq,
                                                      tx_gain,
                                                      rx_gain,
                                                      start_delay,
                                                      elevate_priority,
                                                      cal_file,
                                                      verbose,
                                                      ch_en,
                                                      buffer_size,
                                                      cyclic,
                                                      get_context(uri));
}
 /**   pluto_radar_impl(ctx, //device,
                       true,
                       "cf-ad9361-dds-core-lpc",
                       sink_get_channels_vector(ch_en),
                       "ad9361-phy",
                       gr::iio::iio_param_vec_t(),
                       buffer_size,
                       0,
                       cyclic)
    { return gnuradio::make_block_sptr<pluto_radar_impl>(
        pluto_radar_impl::get_context(uri), ch_en, buffer_size, cyclic);
    }
      pluto_radar_impl(ctx,
                         true,
                         "cf-ad9361-lpc",
                         source_get_channels_vector(ch_en),
                         "ad9361-phy",
                         gr::iio::iio_param_vec_t(),
                         buffer_size,
                         0)  
    { return gnuradio::make_block_sptr<pluto_radar_impl>(
        pluto_radar_impl::get_context(uri), ch_en, buffer_size);
    }   */

std::vector<std::string> 
pluto_radar_impl::sink_get_channels_vector(bool ch1_en,
                                        bool ch2_en,
                                        bool ch3_en,
                                        bool ch4_en)
{
    std::vector<std::string> channels;
    if (ch1_en)
        channels.push_back("voltage0");
    if (ch2_en)
        channels.push_back("voltage1");
    if (ch3_en)
        channels.push_back("voltage2");
    if (ch4_en)
        channels.push_back("voltage3");
    return channels;
}

//template <typename T>
std::vector<std::string>
pluto_radar_impl::sink_get_channels_vector(const std::vector<bool>& ch_en)
{
    std::vector<std::string> channels;
    int idx = 0;
    for (auto en : ch_en) {
        if (en) {
            channels.push_back("voltage" + std::to_string(idx));
        }
        idx++;
    }

    return channels;
}

pluto_radar_impl::pluto_radar_impl( //const std::string& args,
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
                                 const std::vector<bool>& ch_en,
                                 unsigned long buffer_size,
                                 bool cyclic,
                                 iio_context* ctx)
    : gr::block(
          "pluto_radar", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
        //usrp_args(args),
      tx_rate(tx_rate),
      rx_rate(rx_rate),
      tx_freq(tx_freq),
      rx_freq(rx_freq),
      tx_gain(tx_gain),
      rx_gain(rx_gain),
      start_delay(start_delay),
      elevate_priority(elevate_priority),
      cal_file(cal_file),
      verbose(verbose),
      // uri(uri),/* uri("ip:192.168.2.1"),   */
      //ctx(ctx),
      ch_en(ch_en),
      buffer_size(buffer_size),
      cyclic(cyclic), 
      ctx(ctx)   
{
    // Additional parameters. I have the hooks in to make them configurable, but we don't
    // need them right now.
    this->tx_subdev = this->rx_subdev = "";
    this->tx_cpu_format = this->rx_cpu_format = "fc32";
    this->tx_otw_format = this->rx_otw_format = "sc16";
    this->tx_device_addr = this->rx_device_addr = "";
    this->tx_channel_nums = std::vector<size_t>(1, 0);
    this->rx_channel_nums = std::vector<size_t>(1, 0);
    this->tx_buffs = std::vector<const void*>(tx_channel_nums.size(), nullptr);

    this->n_tx_total = 0;
    this->new_msg_received = false;
    this->next_meta = pmt::make_dict();

 /*   config_pluto(//this->usrp,
                this->uri,
                this->tx_rate,
                this->rx_rate,
                this->tx_freq,
                this->rx_freq,
                this->tx_gain,
                this->rx_gain,
                this->tx_subdev,
                this->rx_subdev,
                this->verbose);

    n_delay = 0;
    if (not cal_file.empty()) {
        read_calibration_file(cal_file);
    }   */
 /**
    stop_thread = false;
    underflow_thd = std::thread(&pluto_radar_impl::check_underflow, this);
    overflow_thd = std::thread(&pluto_radar_impl::check_overflow, this);
*/
    pluto_radar::message_port_register_in(PMT_IN);
    pluto_radar::message_port_register_out(PMT_OUT);
    pluto_radar::set_msg_handler(PMT_IN, [this](pmt::pmt_t msg) { this->handle_message(msg); });
}



pluto_radar_impl::~pluto_radar_impl() 
{
    // remember to check the sink and source destructors before finalizing 
}

// Commented out usrp radar functions until ~ln 450 

/**
bool pluto_radar_impl::start()
{
    finished = false;
    d_main_thread = gr::thread::thread([this] { run(); });
    return block::start();
}

bool pluto_radar_impl::stop()
{
    finished = true;
    return block::stop();
}

void pluto_radar_impl::handle_message(const pmt::pmt_t& msg)
{
    if (pmt::is_pdu(msg)) {
        next_meta = pmt::dict_update(next_meta, pmt::car(msg));
        tx_data = pmt::cdr(msg);
        tx_buff_size = pmt::length(tx_data);

        new_msg_received = true;
    }
}   */
/**
void pluto_radar_impl::run()
{
    while (not new_msg_received) {
        if (finished) {
            return;
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    std::atomic<bool>& finished = this->finished;
    if (this->elevate_priority) {
        uhd::set_thread_priority_safe();
    }

    ***********************************************************************
     * Receive thread
     **********************************************************************
    double start_time = usrp->get_time_now().get_real_secs() + start_delay;
    bool rx_stream_now = (start_delay == 0.0 && (rx_channel_nums.size() == 1));
    // create a receive streamer
    uhd::stream_args_t rx_stream_args(rx_cpu_format, rx_otw_format);
    rx_stream_args.channels = rx_channel_nums;
    rx_stream_args.args = uhd::device_addr_t(rx_device_addr);
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);
    auto rx_thread = d_tx_rx_thread_group.create_thread([=, &finished]() {
        receive(usrp, rx_stream, finished, elevate_priority, start_time, rx_stream_now);
    });
    uhd::set_thread_name(rx_thread, "rx_stream");
    ***********************************************************************
     * Transmit thread
     **********************************************************************
    bool tx_has_time_spec = (start_delay != 0.0);
    uhd::stream_args_t tx_stream_args(tx_cpu_format, tx_otw_format);
    tx_stream_args.channels = tx_channel_nums;
    tx_stream_args.args = uhd::device_addr_t(tx_device_addr);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);
    auto tx_thread = d_tx_rx_thread_group.create_thread([=, &finished]() {
        transmit(
            usrp, tx_stream, finished, elevate_priority, start_time, tx_has_time_spec);
    });
    uhd::set_thread_name(tx_thread, "tx_stream");

    d_tx_rx_thread_group.join_all();
} */

/*
void pluto_radar_impl::config_pluto(iio::usrp::multi_usrp::sptr& usrp,
                                  const std::string& args,
                                  const double tx_rate,
                                  const double rx_rate,
                                  const double tx_freq,
                                  const double rx_freq,
                                  const double tx_gain,
                                  const double rx_gain,
                                  const std::string& tx_subdev,
                                  const std::string& rx_subdev,
                                  bool verbose)
{
    pluto = uhd::usrp::multi_usrp::make(args);
    if (not tx_subdev.empty()) {
        usrp->set_tx_subdev_spec(tx_subdev);
    }
    if (not rx_subdev.empty()) {
        usrp->set_rx_subdev_spec(rx_subdev);
    }
    usrp->set_tx_rate(tx_rate);
    usrp->set_rx_rate(rx_rate);
    usrp->set_tx_freq(tx_freq);
    usrp->set_rx_freq(rx_freq);
    usrp->set_tx_gain(tx_gain);
    usrp->set_rx_gain(rx_gain);

    if (verbose) {
        std::cout << boost::format("Using Device: %s") % usrp->get_pp_string()
                  << std::endl;
        std::cout << boost::format("Actual TX Rate: %f Msps") %
                         (usrp->get_tx_rate() / 1e6)
                  << std::endl;
        std::cout << boost::format("Actual RX Rate: %f Msps") %
                         (usrp->get_rx_rate() / 1e6)
                  << std::endl;
        std::cout << boost::format("Actual TX Freq: %f MHz") % (usrp->get_tx_freq() / 1e6)
                  << std::endl;
        std::cout << boost::format("Actual RX Freq: %f MHz") % (usrp->get_rx_freq() / 1e6)
                  << std::endl;
        std::cout << boost::format("Actual TX Gain: %f dB") % usrp->get_tx_gain()
                  << std::endl;
        std::cout << boost::format("Actual RX Gain: %f dB") % usrp->get_rx_gain()
                  << std::endl;
    }
}   */
/*
void pluto_radar_impl::receive(uhd::usrp::multi_usrp::sptr usrp,
                              uhd::rx_streamer::sptr rx_stream,
                              std::atomic<bool>& finished,
                              bool elevate_priority,
                              double start_time,
                              bool rx_stream_now)
{
    if (elevate_priority) {
        uhd::set_thread_priority_safe(1, true);
    }

    // setup variables
    uhd::rx_metadata_t md;
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    cmd.time_spec = uhd::time_spec_t(start_time);
    cmd.stream_now = rx_stream_now;
    rx_stream->issue_stream_cmd(cmd);

    // Set up and allocate buffers
    pmt::pmt_t rx_data_pmt = pmt::make_c32vector(tx_buff_size, 0);
    gr_complex* rx_data_ptr = pmt::c32vector_writable_elements(rx_data_pmt, tx_buff_size);


    double time_until_start = start_time - usrp->get_time_now().get_real_secs();
    double recv_timeout = 0.1 + time_until_start;
    bool stop_called = false;

    // TODO: Handle multiple channels (e.g., one out port per channel)
    while (true) {
        if (finished and not stop_called) {
            rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
            stop_called = true;
        }
        try {
            if (n_delay > 0) {
                // Throw away n_delay samples at the beginning
                std::vector<gr_complex> dummy_vec(n_delay);
                size_t n_rx =
                    rx_stream->recv(dummy_vec.data(), n_delay, md, recv_timeout);
                n_delay -= n_rx;
            }
            rx_stream->recv(rx_data_ptr, tx_buff_size, md, recv_timeout);
            recv_timeout = 0.1;
            // Copy any new metadata to the output and reset the metadata
            pmt::pmt_t meta = this->next_meta;
            if (pmt::length(meta) > 0) {
                meta = pmt::dict_add(
                    meta, pmt::intern(rx_freq_key), pmt::from_double(rx_freq));
                this->next_meta = pmt::make_dict();
            }
            message_port_pub(PMT_OUT, pmt::cons(meta, rx_data_pmt));
            // TODO: May need to resize the buffer if the transmit waveform changes
        } catch (uhd::io_error& e) {
            std::cerr << "Caught an IO exception. " << std::endl;
            std::cerr << e.what() << std::endl;
            return;
        }

        // Handle errors
        switch (md.error_code) {
        case uhd::rx_metadata_t::ERROR_CODE_NONE:
            if ((finished or stop_called) and md.end_of_burst) {
                return;
            }
            break;
        default:
            break;
        }
    }
}   */
/*
void pluto_radar_impl::transmit(uhd::usrp::multi_usrp::sptr usrp,
                               uhd::tx_streamer::sptr tx_stream,
                               std::atomic<bool>& finished,
                               bool elevate_priority,
                               double start_time,
                               double has_time_spec)
{
    if (elevate_priority) {
        uhd::set_thread_priority_safe(1, true);
    }

    // Create the metadata, and populate the time spec at the latest possible moment
    uhd::tx_metadata_t md;
    md.has_time_spec = has_time_spec;
    md.time_spec = uhd::time_spec_t(start_time);

    double timeout = 0.1 + start_time;
    while (not finished) {
        if (new_msg_received) {
            tx_buffs[0] = pmt::c32vector_writable_elements(tx_data, tx_buff_size);
            next_meta = pmt::dict_add(
                next_meta, pmt::intern(tx_freq_key), pmt::from_double(tx_freq));
            next_meta = pmt::dict_add(
                next_meta, pmt::intern(sample_start_key), pmt::from_long(n_tx_total));
            new_msg_received = false;
        }
        n_tx_total += tx_stream->send(tx_buffs, tx_buff_size, md, timeout) *
                      tx_stream->get_num_channels();
        md.has_time_spec = false;
        timeout = 0.1;
    }

    // send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send("", 0, md);
}   */

void pluto_radar_impl::read_calibration_file(const std::string& filename)
{
    std::ifstream file(filename);
    nlohmann::json json;
    if (file) {
        file >> json;
        std::string radio_type = usrp->get_mboard_name();
        for (auto& config : json[radio_type]) {
            if (config["samp_rate"] == usrp->get_tx_rate() and
                config["master_clock_rate"] == usrp->get_master_clock_rate()) {
                n_delay = config["delay"];
                break;
            }
        }
        if (n_delay == 0)
            UHD_LOG_INFO("pluto Radar",
                         "Calibration file found, but no data exists for this "
                         "combination of radio, master clock rate, and sample rate");
    } else {
        UHD_LOG_INFO("pluto Radar", "No calibration file found");
    }

    file.close();
}

void pluto_radar_impl::set_metadata_keys(const std::string& tx_freq_key,
                                        const std::string& rx_freq_key,
                                        const std::string& sample_start_key)
{
    this->tx_freq_key = tx_freq_key;
    this->rx_freq_key = rx_freq_key;
    this->sample_start_key = sample_start_key;
}



// fmcomms2 sink functions


void pluto_radar_impl::sink_set_len_tag_key(const std::string& str)
{
    pluto_radar_impl::set_len_tag_key(str);
}   

 
 void pluto_radar_impl::check_underflow(void)
{
    uint32_t status;
    int ret; 
    std::unique_lock<std::mutex> lock(uf_mutex, std::defer_lock);

    // Clear status registers  
    iio_device_reg_write(dev, 0x80000088, 0x6);   

    for (;;) {
        ret = iio_device_reg_read(dev, 0x80000088, &status);
        if (ret) {
            throw std::runtime_error("Failed to read underflow status register");
        }
        if (status & 1) {
            printf("U");
            // Clear status registers
            iio_device_reg_write(dev, 0x80000088, 1);
        }   
#ifdef _WIN32
        Sleep(OVERFLOW_CHECK_PERIOD_MS);
#else
        usleep(OVERFLOW_CHECK_PERIOD_MS * 1000);
#endif
        lock.lock();
        if (stop_thread)
            break;
        lock.unlock();
    } 
}  

void pluto_radar_impl::sink_set_bandwidth(unsigned long bandwidth)
{                                                                       
    gr::iio::iio_param_vec_t params;
    params.emplace_back("out_voltage_rf_bandwidth", bandwidth);
    pluto_radar_impl::set_params(this->phy, params);
    d_bandwidth = bandwidth;
}

void pluto_radar_impl::sink_set_rf_port_select(const std::string& rf_port_select)
{
    gr::iio::iio_param_vec_t params;
    params.emplace_back("out_voltage0_rf_port_select", rf_port_select);
    pluto_radar_impl::set_params(this->phy, params);
    d_rf_port_select = rf_port_select;
}

void pluto_radar_impl::sink_set_frequency(double frequency)
{
    gr::iio::iio_param_vec_t params;
    params.emplace_back("out_altvoltage1_TX_LO_frequency",
                        static_cast<unsigned long long>(frequency));
    pluto_radar_impl::set_params(this->phy, params);
    d_frequency = frequency;
}

void pluto_radar_impl::sink_set_samplerate(unsigned long samplerate)
{
    gr::iio::iio_param_vec_t params;
    if (samplerate < MIN_RATE) {
        int ret;
        samplerate = samplerate * DECINT_RATIO;
        ret = pluto_radar_impl::handle_decimation_interpolation(
            samplerate, "voltage0", "sampling_frequency", dev, false, true);
        if (ret < 0)
            samplerate = samplerate / 8;
    } else // Disable decimation filter if on
    {
        pluto_radar_impl::handle_decimation_interpolation(
            samplerate, "voltage0", "sampling_frequency", dev, true, true);
    }

    pluto_radar_impl::set_params(this->phy, params);
    d_samplerate = samplerate;
    sink_update_dependent_params();                 
}

void pluto_radar_impl::sink_set_attenuation(size_t chan, double attenuation)
{   /** Temp fix bc find_channel() is an undefined symbol ********************************** undefined symbol error
    bool is_fmcomms4 = !iio_device_find_channel(phy, "voltage1", false);
    if ((!is_fmcomms4 && chan > 0) || chan > 1) {
        throw std::runtime_error("Channel out of range for this device");
    }   */
    gr::iio::iio_param_vec_t params;
    params.emplace_back("out_voltage" + std::to_string(chan) + "_hardwaregain",
                        -attenuation);
    pluto_radar_impl::set_params(this->phy, params);

    d_attenuation[chan] = attenuation;
}

void pluto_radar_impl::sink_update_dependent_params()
{
    gr::iio::iio_param_vec_t params;
    // Set rate configuration
    if (d_filter_source.compare("Off") == 0) {
        params.emplace_back("out_voltage_sampling_frequency", d_samplerate);
        params.emplace_back("out_voltage_rf_bandwidth", d_bandwidth);
    } else if (d_filter_source.compare("Auto") == 0) {
        int ret = ad9361_set_bb_rate(phy, d_samplerate);
        if (ret) {
            throw std::runtime_error("Unable to set BB rate");
            params.emplace_back("out_voltage_rf_bandwidth", d_bandwidth);
        }
    } else if (d_filter_source.compare("File") == 0) {
        std::string filt(d_filter_filename);
        if (!pluto_radar_impl::load_fir_filter(filt, phy))
            throw std::runtime_error("Unable to load filter file");
    } else if (d_filter_source.compare("Design") == 0) {
        int ret = ad9361_set_bb_rate_custom_filter_manual(
            phy, d_samplerate, d_fpass, d_fstop, d_bandwidth, d_bandwidth);
        if (ret) {
            throw std::runtime_error("Unable to set BB rate");
        }
    } else
        throw std::runtime_error("Unknown filter configuration");

    pluto_radar_impl::set_params(this->phy, params);
    // Filters can only be disabled after the sample rate has been set
    if (d_filter_source.compare("Off") == 0) {
        int ret = ad9361_set_trx_fir_enable(phy, false);
        if (ret) {
            throw std::runtime_error("Unable to disable filters");
        }
    }
}

void pluto_radar_impl::sink_set_filter_params(const std::string& filter_source,
                                              const std::string& filter_filename,
                                              float fpass,
                                              float fstop)
{
    d_filter_source = filter_source;
    d_filter_filename = filter_filename;
    d_fpass = fpass;
    d_fstop = fstop;
    sink_update_dependent_params();     
}

// fmcomms2 source functions

 
void pluto_radar_impl::check_overflow(void)
{
    uint32_t status;
    int ret;

    // Wait for stream startup
#ifdef _WIN32
    while (thread_stopped) {
        Sleep(OVERFLOW_CHECK_PERIOD_MS);
    }
    Sleep(OVERFLOW_CHECK_PERIOD_MS);
#else
    while (thread_stopped) {
        usleep(OVERFLOW_CHECK_PERIOD_MS * 1000);
    }
    usleep(OVERFLOW_CHECK_PERIOD_MS * 1000);
#endif

    // Clear status registers
    iio_device_reg_write(dev, 0x80000088, 0x6);

    while (!thread_stopped) {
        ret = iio_device_reg_read(dev, 0x80000088, &status);
        if (ret) {
            throw std::runtime_error("Failed to read overflow status register");
        }
        if (status & 4) {
            printf("O");
            // Clear status registers
            iio_device_reg_write(dev, 0x80000088, 4);
        }
#ifdef _WIN32
        Sleep(OVERFLOW_CHECK_PERIOD_MS);
#else
        usleep(OVERFLOW_CHECK_PERIOD_MS * 1000);
#endif
    }
}     

void pluto_radar_impl::source_update_dependent_params()
{
    gr::iio::iio_param_vec_t params;
    // Set rate configuration
    if (d_filter_source.compare("Off") == 0) {
        params.emplace_back("in_voltage_sampling_frequency", d_samplerate);
        params.emplace_back("in_voltage_rf_bandwidth", d_bandwidth);
    } else if (d_filter_source.compare("Auto") == 0) {
        int ret = ad9361_set_bb_rate(phy, d_samplerate);
        if (ret) {
            throw std::runtime_error("Unable to set BB rate");
            params.emplace_back("in_voltage_rf_bandwidth", d_bandwidth);
        }
    } else if (d_filter_source.compare("File") == 0) {
        std::string filt(d_filter_filename);
        if (!load_fir_filter(filt, phy))
            throw std::runtime_error("Unable to load filter file");
    } else if (d_filter_source.compare("Design") == 0) {
        int ret = ad9361_set_bb_rate_custom_filter_manual(
            phy, d_samplerate, d_fpass, d_fstop, d_bandwidth, d_bandwidth);
        if (ret) {
            throw std::runtime_error("Unable to set BB rate");
        }
    } else
        throw std::runtime_error("Unknown filter configuration");

    pluto_radar_impl::set_params(params);
    // Filters can only be disabled after the sample rate has been set
    if (d_filter_source.compare("Off") == 0) {
        int ret = ad9361_set_trx_fir_enable(phy, false);
        if (ret) {
            throw std::runtime_error("Unable to disable filters");
        }
    }
}   

void pluto_radar_impl::source_set_frequency(double frequency)
{
    gr::iio::iio_param_vec_t params;
    params.emplace_back("out_altvoltage0_RX_LO_frequency",
                        static_cast<unsigned long long>(frequency));
    pluto_radar_impl::set_params(params);
}

void pluto_radar_impl::source_set_samplerate(unsigned long samplerate)
{
    if (samplerate < MIN_RATE) {
        int ret;
        samplerate = samplerate * DECINT_RATIO;
        ret = pluto_radar_impl::handle_decimation_interpolation(
            samplerate, "voltage0", "sampling_frequency", dev, false, false);
        if (ret < 0)
            samplerate = samplerate / 8;
    } else // Disable decimation filter if on
    {
        pluto_radar_impl::handle_decimation_interpolation(
            samplerate, "voltage0", "sampling_frequency", dev, true, false);
    }

    d_samplerate = samplerate;
    source_update_dependent_params();       
}

//template <typename T>
void pluto_radar_impl::source_set_gain_mode(size_t chan, const std::string& mode)
{   /** Temp fix bc find_channel is an undefined symbol ********************************** undefined symbol error
    bool is_fmcomms4 = !iio_device_find_channel(phy, "voltage1", false);
    if ((!is_fmcomms4 && chan > 0) || chan > 1) {
        throw std::runtime_error("Channel out of range for this device");
    }   */
    gr::iio::iio_param_vec_t params;

    params.emplace_back("in_voltage" + std::to_string(chan) +
                        "_gain_control_mode=" + d_gain_mode[chan]);

    pluto_radar_impl::set_params(params);
    d_gain_mode[chan] = mode;
}

//template <typename T>
void pluto_radar_impl::source_set_gain(size_t chan, double gain_value)
{   /** Temp fix bc find_channel is an undefined symbol ********************************** undefined symbol error
    bool is_fmcomms4 = !iio_device_find_channel(phy, "voltage1", false);
    if ((!is_fmcomms4 && chan > 0) || chan > 1) {
        throw std::runtime_error("Channel out of range for this device");
    }   */
    gr::iio::iio_param_vec_t params;

    if (d_gain_mode[chan].compare("manual") == 0) {
        params.emplace_back("in_voltage" + std::to_string(chan) + "_hardwaregain",
                            gain_value);
    }
    pluto_radar_impl::set_params(params);
    d_gain_value[chan] = gain_value;
}

//template <typename T>
void pluto_radar_impl::source_set_quadrature(bool quadrature)
{
    gr::iio::iio_param_vec_t params;
    params.emplace_back("in_voltage_quadrature_tracking_en", quadrature);
    pluto_radar_impl::set_params(params);
    d_quadrature = quadrature;
}

//template <typename T>
void pluto_radar_impl::source_set_rfdc(bool rfdc)
{
    gr::iio::iio_param_vec_t params;
    params.emplace_back("in_voltage_rf_dc_offset_tracking_en", rfdc);
    pluto_radar_impl::set_params(params);
    d_rfdc = rfdc;
}

//template <typename T>
void pluto_radar_impl::source_set_bbdc(bool bbdc)
{
    gr::iio::iio_param_vec_t params;
    params.emplace_back("in_voltage_bb_dc_offset_tracking_en", bbdc);
    pluto_radar_impl::set_params(params);
    d_bbdc = bbdc;
}

//template <typename T>
void pluto_radar_impl::source_set_filter_params(const std::string& filter_source,
                                                const std::string& filter_filename,
                                                float fpass,
                                                float fstop)
{
    d_filter_source = filter_source;
    d_filter_filename = filter_filename;
    d_fpass = fpass;
    d_fstop = fstop;

    source_update_dependent_params();     
}
} /* namespace plasma */
} // namespace gr 
