#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: FSK TRX
# Author: C3
# Copyright: Cubesat Team PoliTO
# Description: FSK packet mod and demod flowgraph with HDLC framing and CRC32 (+CRC16). Blocks disabled are for debugging.
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import analog
import math
from gnuradio import blocks
import pmt
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import gr, pdu
import satellites
import sip
import threading



class FSK_HDLC_TRX(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "FSK TRX", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("FSK TRX")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "FSK_HDLC_TRX")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.fsk_deviation_hz = fsk_deviation_hz = 5000
        self.bitrate = bitrate = 9600
        self.samp_rate = samp_rate = 192000
        self.decim_rx = decim_rx = 5
        self.bandwidth = bandwidth = 2*(bitrate+fsk_deviation_hz)
        self.space = space = -fsk_deviation_hz
        self.samp_rate_rx = samp_rate_rx = samp_rate/decim_rx
        self.maxVCO = maxVCO = fsk_deviation_hz
        self.mark = mark = fsk_deviation_hz
        self.low_pass_filter_taps = low_pass_filter_taps = firdes.low_pass(1.0, samp_rate, bandwidth/1.4, 500, window.WIN_HAMMING, 6.76)
        self.loop = loop = 0.7
        self.filesize = filesize = 256
        self.TXRX = TXRX = 0
        self.TED = TED = 0.35
        self.FREQ = FREQ = 0

        ##################################################
        # Blocks
        ##################################################

        self._TXRX_choices = {'Pressed': 1, 'Released': 0}

        _TXRX_toggle_button = qtgui.ToggleButton(self.set_TXRX, 'Press red for TX', self._TXRX_choices, False, 'value')
        _TXRX_toggle_button.setColors("default", "default", "red", "default")
        self.TXRX = _TXRX_toggle_button

        self.top_grid_layout.addWidget(_TXRX_toggle_button, 0, 0, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.satellites_hdlc_framer_0 = satellites.hdlc_framer(preamble_bytes=32, postamble_bytes=32)
        self.satellites_hdlc_deframer_0 = satellites.hdlc_deframer(check_fcs=False, max_length=500)
        self.qtgui_waterfall_sink_x_0 = qtgui.waterfall_sink_c(
            16384, #size
            window.WIN_HAMMING, #wintype
            0, #fc
            samp_rate_rx, #bw
            "RX", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_waterfall_sink_x_0.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0.enable_grid(False)
        self.qtgui_waterfall_sink_x_0.enable_axis_labels(True)



        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0.qwidget(), Qt.QWidget)

        self.top_grid_layout.addWidget(self._qtgui_waterfall_sink_x_0_win, 0, 1, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
            1024, #size
            samp_rate_rx, #samp_rate
            "Symbol Syncronizator Error", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(False)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(True)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_time_sink_x_0_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            16384, #size
            window.WIN_HAMMING, #wintype
            0, #fc
            samp_rate_rx, #bw
            "", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_0.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_x_0_win, 1, 1, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_eye_sink_x_0 = qtgui.eye_sink_f(
            (1024*10), #size
            samp_rate_rx, #samp_rate
            1, #number of inputs
            None
        )
        self.qtgui_eye_sink_x_0.set_update_time(0.10)
        self.qtgui_eye_sink_x_0.set_samp_per_symbol((int((samp_rate_rx)/bitrate)))
        self.qtgui_eye_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_eye_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_eye_sink_x_0.enable_tags(False)
        self.qtgui_eye_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_eye_sink_x_0.enable_autoscale(True)
        self.qtgui_eye_sink_x_0.enable_grid(False)
        self.qtgui_eye_sink_x_0.enable_axis_labels(True)
        self.qtgui_eye_sink_x_0.enable_control_panel(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'blue', 'blue', 'blue', 'blue',
            'blue', 'blue', 'blue', 'blue', 'blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_eye_sink_x_0.set_line_label(i, "Eye[Data {0}]".format(i))
            else:
                self.qtgui_eye_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_eye_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_eye_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_eye_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_eye_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_eye_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_eye_sink_x_0_win = sip.wrapinstance(self.qtgui_eye_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_eye_sink_x_0_win)
        self.pdu_tagged_stream_to_pdu_0 = pdu.tagged_stream_to_pdu(gr.types.byte_t, 'packet_len')
        self.pdu_pdu_to_tagged_stream_1 = pdu.pdu_to_tagged_stream(gr.types.byte_t, 'packet_len')
        self.pdu_pdu_to_tagged_stream_0 = pdu.pdu_to_tagged_stream(gr.types.byte_t, 'packet_len')
        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(decim_rx, low_pass_filter_taps, FREQ, samp_rate)
        self.digital_symbol_sync_xx_0 = digital.symbol_sync_ff(
            digital.TED_EARLY_LATE,
            ((samp_rate_rx)/bitrate),
            loop,
            1.0,
            TED,
            1.5,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_MMSE_8TAP,
            128,
            [])
        self.digital_crc32_bb_0_0 = digital.crc32_bb(True, "packet_len", True)
        self.digital_crc32_bb_0 = digital.crc32_bb(False, "packet_len", True)
        self.digital_binary_slicer_fb_0 = digital.binary_slicer_fb()
        self.blocks_vector_to_stream_0 = blocks.vector_to_stream(gr.sizeof_char*1, 256)
        self.blocks_vco_c_0 = blocks.vco_c(samp_rate, maxVCO, 1)
        self.blocks_uchar_to_float_1 = blocks.uchar_to_float()
        self.blocks_throttle2_0 = blocks.throttle( gr.sizeof_char*1, samp_rate, True, 0 if "auto" == "auto" else max( int(float(0.1) * samp_rate) if "auto" == "time" else int(0.1), 1) )
        self.blocks_tag_debug_0 = blocks.tag_debug(gr.sizeof_char*1, 'received_pkts', "")
        self.blocks_tag_debug_0.set_display(True)
        self.blocks_stream_to_tagged_stream_0 = blocks.stream_to_tagged_stream(gr.sizeof_char, 1, filesize, "packet_len")
        self.blocks_selector_0 = blocks.selector(gr.sizeof_gr_complex*1,0,TXRX)
        self.blocks_selector_0.set_enabled(True)
        self.blocks_repeat_0 = blocks.repeat(gr.sizeof_char*1, (int(samp_rate*(1/9600))))
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_ff(((mark - space) / (maxVCO)))
        self.blocks_message_strobe_0 = blocks.message_strobe(pmt.make_dict(), 2000)
        self.blocks_file_source_0 = blocks.file_source(gr.sizeof_char*filesize, '/home/giorgio/input.txt', True, 0, 0)
        self.blocks_file_source_0.set_begin_tag(pmt.PMT_NIL)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, '/home/giorgio/output.txt', False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_add_const_vxx_1 = blocks.add_const_ff((-0.075))
        self.blocks_add_const_vxx_0 = blocks.add_const_ff(((space) / (maxVCO)))
        self.analog_quadrature_demod_cf_0 = analog.quadrature_demod_cf(((samp_rate_rx)/(2*math.pi*fsk_deviation_hz)))


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_message_strobe_0, 'strobe'), (self.pdu_pdu_to_tagged_stream_0, 'pdus'))
        self.msg_connect((self.pdu_tagged_stream_to_pdu_0, 'pdus'), (self.satellites_hdlc_framer_0, 'in'))
        self.msg_connect((self.satellites_hdlc_deframer_0, 'out'), (self.pdu_pdu_to_tagged_stream_1, 'pdus'))
        self.msg_connect((self.satellites_hdlc_framer_0, 'out'), (self.blocks_message_strobe_0, 'set_msg'))
        self.connect((self.analog_quadrature_demod_cf_0, 0), (self.digital_symbol_sync_xx_0, 0))
        self.connect((self.blocks_add_const_vxx_0, 0), (self.blocks_vco_c_0, 0))
        self.connect((self.blocks_add_const_vxx_1, 0), (self.digital_binary_slicer_fb_0, 0))
        self.connect((self.blocks_file_source_0, 0), (self.blocks_vector_to_stream_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_add_const_vxx_0, 0))
        self.connect((self.blocks_repeat_0, 0), (self.blocks_throttle2_0, 0))
        self.connect((self.blocks_selector_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.blocks_selector_0, 1), (self.freq_xlating_fir_filter_xxx_0, 0))
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.digital_crc32_bb_0, 0))
        self.connect((self.blocks_throttle2_0, 0), (self.blocks_uchar_to_float_1, 0))
        self.connect((self.blocks_uchar_to_float_1, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_vco_c_0, 0), (self.blocks_selector_0, 0))
        self.connect((self.blocks_vector_to_stream_0, 0), (self.blocks_stream_to_tagged_stream_0, 0))
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.satellites_hdlc_deframer_0, 0))
        self.connect((self.digital_crc32_bb_0, 0), (self.pdu_tagged_stream_to_pdu_0, 0))
        self.connect((self.digital_crc32_bb_0_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.digital_crc32_bb_0_0, 0), (self.blocks_tag_debug_0, 0))
        self.connect((self.digital_symbol_sync_xx_0, 0), (self.blocks_add_const_vxx_1, 0))
        self.connect((self.digital_symbol_sync_xx_0, 0), (self.qtgui_eye_sink_x_0, 0))
        self.connect((self.digital_symbol_sync_xx_0, 1), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.analog_quadrature_demod_cf_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.qtgui_waterfall_sink_x_0, 0))
        self.connect((self.pdu_pdu_to_tagged_stream_0, 0), (self.blocks_repeat_0, 0))
        self.connect((self.pdu_pdu_to_tagged_stream_1, 0), (self.digital_crc32_bb_0_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "FSK_HDLC_TRX")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_fsk_deviation_hz(self):
        return self.fsk_deviation_hz

    def set_fsk_deviation_hz(self, fsk_deviation_hz):
        self.fsk_deviation_hz = fsk_deviation_hz
        self.set_bandwidth(2*(self.bitrate+self.fsk_deviation_hz))
        self.set_mark(self.fsk_deviation_hz)
        self.set_maxVCO(self.fsk_deviation_hz)
        self.set_space(-self.fsk_deviation_hz)
        self.analog_quadrature_demod_cf_0.set_gain(((self.samp_rate_rx)/(2*math.pi*self.fsk_deviation_hz)))

    def get_bitrate(self):
        return self.bitrate

    def set_bitrate(self, bitrate):
        self.bitrate = bitrate
        self.set_bandwidth(2*(self.bitrate+self.fsk_deviation_hz))
        self.digital_symbol_sync_xx_0.set_sps(((self.samp_rate_rx)/self.bitrate))
        self.qtgui_eye_sink_x_0.set_samp_per_symbol((int((self.samp_rate_rx)/self.bitrate)))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_low_pass_filter_taps(firdes.low_pass(1.0, self.samp_rate, self.bandwidth/1.4, 500, window.WIN_HAMMING, 6.76))
        self.set_samp_rate_rx(self.samp_rate/self.decim_rx)
        self.blocks_repeat_0.set_interpolation((int(self.samp_rate*(1/9600))))
        self.blocks_throttle2_0.set_sample_rate(self.samp_rate)

    def get_decim_rx(self):
        return self.decim_rx

    def set_decim_rx(self, decim_rx):
        self.decim_rx = decim_rx
        self.set_samp_rate_rx(self.samp_rate/self.decim_rx)

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth
        self.set_low_pass_filter_taps(firdes.low_pass(1.0, self.samp_rate, self.bandwidth/1.4, 500, window.WIN_HAMMING, 6.76))

    def get_space(self):
        return self.space

    def set_space(self, space):
        self.space = space
        self.blocks_add_const_vxx_0.set_k(((self.space) / (self.maxVCO)))
        self.blocks_multiply_const_vxx_0.set_k(((self.mark - self.space) / (self.maxVCO)))

    def get_samp_rate_rx(self):
        return self.samp_rate_rx

    def set_samp_rate_rx(self, samp_rate_rx):
        self.samp_rate_rx = samp_rate_rx
        self.analog_quadrature_demod_cf_0.set_gain(((self.samp_rate_rx)/(2*math.pi*self.fsk_deviation_hz)))
        self.digital_symbol_sync_xx_0.set_sps(((self.samp_rate_rx)/self.bitrate))
        self.qtgui_eye_sink_x_0.set_samp_rate(self.samp_rate_rx)
        self.qtgui_eye_sink_x_0.set_samp_per_symbol((int((self.samp_rate_rx)/self.bitrate)))
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate_rx)
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate_rx)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(0, self.samp_rate_rx)

    def get_maxVCO(self):
        return self.maxVCO

    def set_maxVCO(self, maxVCO):
        self.maxVCO = maxVCO
        self.blocks_add_const_vxx_0.set_k(((self.space) / (self.maxVCO)))
        self.blocks_multiply_const_vxx_0.set_k(((self.mark - self.space) / (self.maxVCO)))

    def get_mark(self):
        return self.mark

    def set_mark(self, mark):
        self.mark = mark
        self.blocks_multiply_const_vxx_0.set_k(((self.mark - self.space) / (self.maxVCO)))

    def get_low_pass_filter_taps(self):
        return self.low_pass_filter_taps

    def set_low_pass_filter_taps(self, low_pass_filter_taps):
        self.low_pass_filter_taps = low_pass_filter_taps
        self.freq_xlating_fir_filter_xxx_0.set_taps(self.low_pass_filter_taps)

    def get_loop(self):
        return self.loop

    def set_loop(self, loop):
        self.loop = loop
        self.digital_symbol_sync_xx_0.set_loop_bandwidth(self.loop)

    def get_filesize(self):
        return self.filesize

    def set_filesize(self, filesize):
        self.filesize = filesize
        self.blocks_stream_to_tagged_stream_0.set_packet_len(self.filesize)
        self.blocks_stream_to_tagged_stream_0.set_packet_len_pmt(self.filesize)

    def get_TXRX(self):
        return self.TXRX

    def set_TXRX(self, TXRX):
        self.TXRX = TXRX
        self.blocks_selector_0.set_output_index(self.TXRX)

    def get_TED(self):
        return self.TED

    def set_TED(self, TED):
        self.TED = TED
        self.digital_symbol_sync_xx_0.set_ted_gain(self.TED)

    def get_FREQ(self):
        return self.FREQ

    def set_FREQ(self, FREQ):
        self.FREQ = FREQ
        self.freq_xlating_fir_filter_xxx_0.set_center_freq(self.FREQ)




def main(top_block_cls=FSK_HDLC_TRX, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
