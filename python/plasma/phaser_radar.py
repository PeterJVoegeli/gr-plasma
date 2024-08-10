#!/usr/bin/env python3
#  Must use Python 3
# Copyright (C) 2022 Analog Devices, Inc. 
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     - Neither the name of Analog Devices, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#     - The use of this software may or may not infringe the patent rights
#       of one or more patent holders.  This license does not release you
#       from the requirement that you obtain separate licenses from these
#       patent holders to use this software.
#     - Use of the software either in source or binary form, must be run
#       on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''CFAR Radar Demo with Phaser (CN0566)
   Jon Kraft, Jan 20 2024'''

# Imports
import adi
#from target_detection_dbfs import cfar

import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from pyqtgraph.Qt import QtCore, QtGui

from gnuradio import gr, blocks
import pmt
#import chrono = time\
import threading
from threading import Thread


print("before")
print(threading.current_thread())
#t = Thread(target=print, args=[1])
#t.run()
#def cool(self):
#    Thread.start()
#    Thread.run()
#print(threading.active_count())
#print(std.chrono.microseconds(10))

# while loop?
class phaser_radar(gr.sync_block):
    def __init__(self, incomingKey, outgoingKey):
        gr.sync_block.__init__(self,
            name="phaser_radar",
            in_sig=None, out_sig=None)
        self.message_port_register_in(pmt.intern("in"))
        self.message_port_register_in(pmt.intern("control"))
        self.message_port_register_out(pmt.intern("out"))
        self.set_msg_handler(pmt.intern("in"), self.msg_handler)

    def start(self):
        print("in start")
#        Thread.run(self)
#        t = Thread(target=print, args=[1])
#        main_thread = t.run()
#        Thread.start(self)  #Passes '_initialized'? which conflicts with block_gateway?

            # idk if this will end up doing anything
        pulse_width = 1e-5
        threading.Timer(pulse_width, phaser_radar.msg_handler) # Only called after each pulse
            
#        t.start()
#        return block.start()

    #def stop(self):
    #    main_thread.interrupt()
    #    main_thread.join()
    #    return block.stop()

    def msg_handler(self, msg):
        print("in msg_handler")
        #Thread.run(self)
        #Thread.start()
#        Thread.run()
#        t = Thread(target=print, args=[1])
#        t.run()

        print(threading.current_thread())

        tx_data = pmt.cdr(msg) # Pass the signal to the tx
        # Instantiate all the Devices
        rpi_ip = "ip:phaser.local"  # IP address of the Raspberry Pi
        sdr_ip = "ip:192.168.2.1"  # "192.168.2.1, or pluto.local"  # IP address of the Transceiver Block
        my_sdr = adi.ad9361(uri=sdr_ip)
        my_phaser = adi.CN0566(uri=rpi_ip, sdr=my_sdr)

        # Initialize both ADAR1000s, set gains to max, and all phases to 0
        my_phaser.configure(device_mode="rx")
        my_phaser.load_gain_cal()
        my_phaser.load_phase_cal()
        for i in range(0, 8):
            my_phaser.set_chan_phase(i, 0)

        gain_list = [8, 34, 84, 127, 127, 84, 34, 8]  # Blackman taper
        for i in range(0, len(gain_list)):
            my_phaser.set_chan_gain(i, gain_list[i], apply_cal=True)

        # Setup Raspberry Pi GPIO states
        try:
            my_phaser._gpios.gpio_tx_sw = 0  # 0 = TX_OUT_2, 1 = TX_OUT_1
            my_phaser._gpios.gpio_vctrl_1 = 1 # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
            my_phaser._gpios.gpio_vctrl_2 = 1 # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)
        except:
            my_phaser.gpios.gpio_tx_sw = 0  # 0 = TX_OUT_2, 1 = TX_OUT_1
            my_phaser.gpios.gpio_vctrl_1 = 1 # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
            my_phaser.gpios.gpio_vctrl_2 = 1 # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)
# Values to pass live via message:
# sample_rate  Prob just leave as a variable in the block
# signal_freq
# tx & rx enabled channels
# tx & rx gain
# manual or slow attack? Can prob leave this as a variable in the block

        sample_rate = 0.6e6
        center_freq = 2.1e9
        signal_freq = 100e3
#        num_slices = 50     # this sets how much time will be displayed on the waterfall plot
        fft_size = 1024 * 8
#        plot_freq = 100e3    # x-axis freq range to plot
#        img_array = np.ones((num_slices, fft_size))*(-100)

        # Configure SDR Rx
        my_sdr.sample_rate = int(sample_rate)
        my_sdr.rx_lo = int(center_freq)  # set this to output_freq - (the freq of the HB100)
        my_sdr.rx_enabled_channels = [0, 1]  # enable Rx1 (voltage0) and Rx2 (voltage1)
        my_sdr.rx_buffer_size = int(fft_size)
        my_sdr.gain_control_mode_chan0 = "manual"  # manual or slow_attack
        my_sdr.gain_control_mode_chan1 = "manual"  # manual or slow_attack
        my_sdr.rx_hardwaregain_chan0 = int(30)  # must be between -3 and 70
        my_sdr.rx_hardwaregain_chan1 = int(30)  # must be between -3 and 70
        # Configure SDR Tx
        my_sdr.tx_lo = int(center_freq)
        my_sdr.tx_enabled_channels = [0, 1]
        my_sdr.tx_cyclic_buffer = True  # must set cyclic buffer to true for the tdd burst mode.  Otherwise Tx will turn on and off randomly
        my_sdr.tx_hardwaregain_chan0 = -88  # must be between 0 and -88
        my_sdr.tx_hardwaregain_chan1 = -0  # must be between 0 and -88

        # Configure the ADF4159 Rampling PLL
        output_freq = 12.145e9
        BW = 500e6
        num_steps = 500
        ramp_time = 0.5e3  # us
        my_phaser.frequency = int(output_freq / 4)  # Output frequency divided by 4
        my_phaser.freq_dev_range = int(
            BW / 4
        )  # frequency deviation range in Hz.  This is the total freq deviation of the complete freq ramp
        my_phaser.freq_dev_step = int(
            (BW/4) / num_steps
        )  # frequency deviation step in Hz.  This is fDEV, in Hz.  Can be positive or negative
        my_phaser.freq_dev_time = int(
            ramp_time
        )  # total time (in us) of the complete frequency ramp
#        print("requested freq dev time = ", ramp_time)
        ramp_time = my_phaser.freq_dev_time
        ramp_time_s = ramp_time / 1e6
#        print("actual freq dev time = ", ramp_time)
        my_phaser.delay_word = 4095  # 12 bit delay word.  4095*PFD = 40.95 us.  For sawtooth ramps, this is also the length of the Ramp_complete signal
        my_phaser.delay_clk = "PFD"  # can be 'PFD' or 'PFD*CLK1'
        my_phaser.delay_start_en = 0  # delay start
        my_phaser.ramp_delay_en = 0  # delay between ramps.
        my_phaser.trig_delay_en = 0  # triangle delay
        my_phaser.ramp_mode = "continuous_triangular"  # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
        my_phaser.sing_ful_tri = (
            0  # full triangle enable/disable -- this is used with the single_ramp_burst mode
        )
        my_phaser.tx_trig_en = 0  # start a ramp with TXdata
        my_phaser.enable = 0  # 0 = PLL enable.  Write this last to update all the registers
    # Create a sinewave waveform
        fs = int(my_sdr.sample_rate)
        N = int(my_sdr.rx_buffer_size)
        fc = int(signal_freq / (fs / N)) * (fs / N)
        ts = 1 / float(fs)
        t = np.arange(0, N * ts, ts)
        i = np.cos(2 * np.pi * t * fc) * 2 ** 14
        q = np.sin(2 * np.pi * t * fc) * 2 ** 14
        iq = 1 * (i + 1j * q)
# send and receive data
        my_sdr._ctx.set_timeout(0)
        my_sdr.tx([iq * 0.5, iq])
        data = my_sdr.rx()
        data = data[0] + data[1]
#        print(data[0]) 
#        pmt_data = pmt.make_c32vector(1, data[0])
#        data_list= [data[0], data[1]] 
        pmt_data = pmt.init_c32vector(2, [data[0], data[1]])
#        print(pmt_data)

#        next_meta = pmt.make_dict()
#        next_meta = pmt.dict_update(next_meta, msg)
#        print(next_meta)
#        dict_data = pmt.dict_update(pmt_data)
#        meta = pmt.dict_update(next_meta, pmt_data)
#        print(msg)

        msg_car = pmt.car(msg)
        pdu = pmt.cons(msg_car, pmt_data)
        print("Is PDU?", pmt.is_pdu(pdu))
        print(pdu)

        self.message_port_pub(pmt.intern("out"), pdu)
        
#    def start(self):
#        main_thread = gr.thread(run())
#        return block.start()
#    def stop(self):
#        return block.stop()
   