/*
 * Copyright 2022 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(1)                                                       */
/* BINDTOOL_USE_PYGCCXML(1)                                                        */
/* BINDTOOL_HEADER_FILE(range_doppler_sink.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(6fbb7f4d3d5fcfbe2d66aa53b32f9063)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/plasma/range_doppler_sink.h>
// pydoc.h is automatically generated in the build directory
#include <range_doppler_sink_pydoc.h>

void bind_range_doppler_sink(py::module& m)
{

    using range_doppler_sink = ::gr::plasma::range_doppler_sink;


    py::class_<range_doppler_sink,
               gr::block,
               gr::basic_block,
               std::shared_ptr<range_doppler_sink>>(
        m, "range_doppler_sink", D(range_doppler_sink))

        .def(py::init(&range_doppler_sink::make),
             py::arg("samp_rate"),
             py::arg("num_pulse_cpi"),
             py::arg("center_freq"),
             py::arg("parent") = nullptr,
             D(range_doppler_sink, make))


        .def("exec_", &range_doppler_sink::exec_, D(range_doppler_sink, exec_))


        .def(
            "qwidget",
            [](range_doppler_sink& self) {
                return reinterpret_cast<uintptr_t>(self.qwidget());
            },
            D(range_doppler_sink, qwidget))


        .def(
            "pyqwidget",
            [](std::shared_ptr<range_doppler_sink> p) {
                return PyLong_AsLongLong(p->pyqwidget());
            },
            D(range_doppler_sink, pyqwidget))


        .def("set_dynamic_range",
             &range_doppler_sink::set_dynamic_range,
             py::arg("arg0"),
             D(range_doppler_sink, set_dynamic_range))


        .def("set_msg_queue_depth",
             &range_doppler_sink::set_msg_queue_depth,
             py::arg("depth"),
             D(range_doppler_sink, set_msg_queue_depth))

        ;
}
