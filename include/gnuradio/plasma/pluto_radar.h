/* -*- c++ -*- */
/*
 * Copyright 2024 gr-plasma author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PLASMA_PLUTO_RADAR_H
#define INCLUDED_PLASMA_PLUTO_RADAR_H

#include <gnuradio/block.h>
#include <iio.h>
#include <gnuradio/iio/iio_types.h>
#include <gnuradio/plasma/api.h>
#include <gnuradio/iio/api.h>
#include <gnuradio/sync_block.h>
#include <string>
#include <thread>
#include <vector>

#include <volk/volk_alloc.hh>

#define DEFAULT_BUFFER_SIZE 0x8000
extern "C" {
struct iio_context;
};

namespace gr {
namespace plasma {

/*!
 * \brief A block that simultaneously transmits and receives data from a USRP device
 * \ingroup plasma
 *
 */
class PLASMA_API pluto_radar : virtual public gr::block
{
public:
    static iio_context* get_context(const std::string& uri);
    typedef std::shared_ptr<pluto_radar> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of plasma::usrp_radar.
     *
     * To avoid accidental use of raw pointers, plasma::usrp_radar's
     * constructor is in a private implementation
     * class. plasma::usrp_radar::make is the public interface for
     * creating new instances.
     */              //const std::string& args,
    static sptr make(const double tx_rate,
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
                     bool cyclic);
                     
    virtual void set_metadata_keys(const std::string& tx_freq_key,
                                   const std::string& rx_freq_key,
                                   const std::string& sample_start_key) = 0;
    // Overloaded 
       // Sink.h
/*    static sptr make(const std::string& uri,
                     const std::string& device,
                     const std::vector<std::string>& channels,
                     const std::string& device_phy,
                     const gr::iio::iio_param_vec_t& params,
                     unsigned int buffer_size = DEFAULT_BUFFER_SIZE,
                     unsigned int interpolation = 0,
                     bool cyclic = false);

    static sptr make_from(iio_context* ctx,
                          const std::string& device,
                          const std::vector<std::string>& channels,
                          const std::string& device_phy,
                          const gr::iio::iio_param_vec_t& params,
                          unsigned int buffer_size = DEFAULT_BUFFER_SIZE,
                          unsigned int interpolation = 0,
                          bool cyclic = false);
       // Source.h
    static sptr make(const std::string& uri,
                     const std::string& device,
                     const std::vector<std::string>& channels,
                     const std::string& device_phy,
                     const gr::iio::iio_param_vec_t& params,
                     unsigned int buffer_size = DEFAULT_BUFFER_SIZE,
                     unsigned int decimation = 0);

    static sptr make_from(iio_context* ctx,
                          const std::string& device,
                          const std::vector<std::string>& channels,
                          const std::string& device_phy,
                          const gr::iio::iio_param_vec_t& params,
                          unsigned int buffer_size = DEFAULT_BUFFER_SIZE,
                          unsigned int decimation = 0);*/


    virtual void sink_set_len_tag_key(const std::string& val = "") = 0;  //use the source version? idk

    virtual void sink_set_bandwidth(unsigned long bandwidth) = 0;
    virtual void sink_set_rf_port_select(const std::string& rf_port_select) = 0;
    virtual void sink_set_frequency(double frequency) = 0;
    virtual void sink_set_samplerate(unsigned long samplerate) = 0;
    virtual void sink_set_attenuation(size_t chan, double gain) = 0;
    virtual void sink_set_filter_params(const std::string& filter_source,
                                   const std::string& filter_filename = "",
                                   float fpass = 0.0,
                                   float fstop = 0.0) = 0;

    
//    virtual void source_set_len_tag_key(const std::string& len_tag_key = "packet_len") = 0;

    virtual void source_set_frequency(double frequency) = 0;
    virtual void source_set_samplerate(unsigned long samplerate) = 0;
    virtual void source_set_gain_mode(size_t chan, const std::string& mode) = 0;
    virtual void source_set_gain(size_t chan, double gain) = 0;
    virtual void source_set_quadrature(bool quadrature) = 0;
    virtual void source_set_rfdc(bool rfdc) = 0;
    virtual void source_set_bbdc(bool bbdc) = 0;
    virtual void source_set_filter_params(const std::string& filter_source,
                                   const std::string& filter_filename = "",
                                   float fpass = 0.0,
                                   float fstop = 0.0) = 0;

};

} // plasma namespace 
   

} // namespace gr

#endif /* INCLUDED_PLASMA_USRP_RADAR_H */
