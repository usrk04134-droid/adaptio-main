#!/usr/bin/python3

import argparse
import json
import math
import os
import pathlib
from datetime import datetime
from typing import List

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import yaml

# Min samples for bead, layer, sequence
MIN_SAMPLES = 60


def ema(values: List[float], alpha: float) -> List[float]:
    val = values[0]

    res = []
    for v in values:
        val = alpha * v + (1 - alpha) * val

        res.append(val)

    return res


def polygon_area(x_y) -> float:
    x_y = np.array(x_y)
    x_y = x_y.reshape(-1, 2)

    x = x_y[:, 0]
    y = x_y[:, 1]

    s1 = np.sum(x * np.roll(y, -1))
    s2 = np.sum(y * np.roll(x, -1))

    return np.absolute(s1 - s2) / 2


def cm_min_to_mm_sec(val: float) -> float:
    return val / 6.0


def mm_sec_to_cm_min(val: float) -> float:
    return val * 6.0


class Samples:
    def __init__(
        self,
        reference,
        samples,
        weld_sessions,
        name="",
        joined=False,
    ):
        self.samples = samples
        self.name = name
        self.reference = reference
        self.joined = joined
        self.weld_sessions = []

        if not self.name and self.samples:
            self.name = self.get_first_ts().strftime("adaptio_%Y%m%d_%H%M%S")

        if self.samples:
            first_ts = self.get_first_ts()
            last_ts = self.get_last_ts()
            for ts in weld_sessions:
                if ts >= first_ts and ts < last_ts:
                    self.weld_sessions.append(ts)

    def get_first_ts(self) -> datetime:
        return self.samples[0].ts

    def get_last_ts(self) -> datetime:
        return self.samples[-1].ts

    def get_timeline(self, relative: bool = True):
        if self.joined or (self.get_last_ts() - self.get_first_ts()).seconds < 60:
            return ([indx for indx, _ in enumerate(self.samples)], "Samples")

        return (
            (
                [(s.ts - self.samples[0].ts).seconds / 60 for s in self.samples]
                if relative
                else [(s.ts).seconds / 60 for s in self.samples]
            ),
            "Time (min)",
        )

    def get_timestamp_weld_session_samples(self):
        def find_closest_sample_index(target):
            closest_index = 0
            min_difference = abs(self.samples[0].ts - target)

            for i in range(1, len(self.samples)):
                difference = abs(self.samples[i].ts - target)
                if difference < min_difference:
                    min_difference = difference
                    closest_index = i
                elif difference >= min_difference:
                    break

            return closest_index

        return [find_closest_sample_index(wsts) for wsts in self.weld_sessions]

    def get_abp_samples(self):
        seqs = []
        seq = None
        for s in self.samples:
            if (
                s.mode == "abp"
                and s.weld_systems[1].state == "arcing"
                and s.weld_systems[2].state == "arcing"
                and s.weld_systems[1].current > 600.0
                and s.weld_systems[2].current > 600.0
                and s.weld_object_ang_velocity_actual > 0.0
            ):
                # if (s.mode == "abp" or s.mode == "jt") and s.mcs:
                if seq is None:
                    seq = []
                seq.append(s)
            else:
                if seq:
                    seqs.append(
                        Samples(
                            reference=self.reference,
                            samples=seq,
                            weld_sessions=self.weld_sessions,
                        )
                    )
                    seq = None
        if seq:
            seqs.append(
                Samples(
                    reference=self.reference,
                    samples=seq,
                    weld_sessions=self.weld_sessions,
                )
            )

        return seqs

        seqs = []
        seq = None
        for s in self.samples:
            # if s.mode == 'abp':
            if 1 in s.weld_systems and s.weld_systems[1].state == "arcing":
                if seq is None:
                    seq = []
                seq.append(s)
                # else:
                # if seq:
                #    seqs.append(Samples(seq))
                #    seq = None
        if seq:
            seqs.append(Samples(seq))

        return seqs

    def join_abp_samples(self):
        layers = self.get_layers()

        seq = []

        layer_no = 0
        last_layer_no = 0
        for layer in layers:
            if len(layer.samples) < MIN_SAMPLES:
                continue

            beads = layer.get_beads()
            for bead in beads:
                if len(bead.samples) < MIN_SAMPLES:
                    continue

                for s in bead.samples:
                    if (
                        s.mode == "abp"
                        and s.weld_systems[1].state == "arcing"
                        and s.weld_systems[2].state == "arcing"
                        and s.weld_systems[1].current > 600.0
                        and s.weld_systems[2].current > 600.0
                        and s.weld_object_ang_velocity_actual > 0.0
                    ):
                        # print(f"layer_no: {layer_no}")
                        if s.layer_no != last_layer_no:
                            layer_no += 1

                        last_layer_no = s.layer_no
                        s.layer_no = layer_no

                        seq.append(s)

        return Samples(
            reference=self.reference,
            weld_sessions=self.weld_sessions,
            samples=seq,
            joined=True,
        )

    def get_layers(self):
        layers = []
        layer = []
        last_layer = None
        for s in self.samples:
            if s.layer_no == 0:
                continue
            if s.layer_no != last_layer and layer:
                layers.append(
                    Samples(
                        reference=self.reference,
                        weld_sessions=self.weld_sessions,
                        samples=layer,
                    )
                )
                layer = []

            layer.append(s)
            last_layer = s.layer_no

        if layer:
            layers.append(
                Samples(
                    reference=self.reference,
                    samples=layer,
                    weld_sessions=self.weld_sessions,
                )
            )

        return layers

    def get_beads(self):
        beads = []
        bead = []
        last_bead = None
        for s in self.samples:
            if s.bead_no == 0:
                continue
            if s.bead_no != last_bead and bead:
                beads.append(
                    Samples(
                        reference=self.reference,
                        samples=bead,
                        weld_sessions=self.weld_sessions,
                    )
                )
                bead = []

            bead.append(s)
            last_bead = s.bead_no

        if bead:
            beads.append(
                Samples(
                    reference=self.reference,
                    samples=bead,
                    weld_sessions=self.weld_sessions,
                )
            )

        return beads

    def get_bead_op_samples(self, op):
        samples = []
        for s in self.samples:
            if s.abp_op == op:
                samples.append(s)

        return Samples(
            reference=self.reference, samples=samples, weld_sessions=self.weld_sessions
        )

    def get_ts(self):
        return [s.ts for s in self.samples]

    def get_bead_op(self):
        res = []

        for s in self.samples:
            match s.abp_op:
                case "idle":
                    res.append(0)
                case "repositioning":
                    res.append(1)
                case "steady":
                    res.append(2)
                case "overlapping":
                    res.append(3)

        return res

    def get_sample(self, pos):
        sample = None
        min_dist = float("inf")

        for s in self.samples:
            dist = abs(s.weld_object_pos - pos)
            if dist < min_dist:
                min_dist = dist
                sample = s

        return sample

    def get_mode(self):
        return [s.mode for s in self.samples]

    def get_layer_no(self):
        return [s.layer_no for s in self.samples]

    def get_bead_no(self):
        return [s.bead_no for s in self.samples]

    def get_weld_object_lin_velocity(self, cm_per_min=True):
        return [
            (
                mm_sec_to_cm_min(s.get_weld_object_lin_velocity())
                if cm_per_min
                else s.get_weld_object_lin_velocity()
            )
            for s in self.samples
        ]

    def get_weld_object_lin_velocity_desired(self, cm_per_min=True):
        return [
            (
                mm_sec_to_cm_min(s.get_weld_object_lin_velocity_desired())
                if cm_per_min
                else s.get_weld_object_lin_velocity_desired()
            )
            for s in self.samples
        ]

    def get_weld_object_ang_velocity(self):
        return [s.weld_object_ang_velocity_actual for s in self.samples]

    def get_weld_object_ang_velocity_desired(self):
        return [s.weld_object_ang_velocity_desired for s in self.samples]

    def get_bead_area(self):
        return [s.get_bead_area() for s in self.samples]

    def get_total_bead_area(self):
        total = 0
        for s in self.samples:
            area = s.get_bead_area()
            total += sum(area.values()) / len(self.samples)

        return total

    def get_wire_speed(self):
        return [s.get_wire_speed() for s in self.samples]

    def get_weld_object_pos(self, deg=True):

        return [
            s.weld_object_pos * 180 / math.pi if deg else s.weld_object_pos
            for s in self.samples
        ]

    def get_groove_depth(self):
        groove_depth_left = []
        groove_depth_right = []
        groove_depth_avg = []

        for s in self.samples:
            left, right, avg = s.get_groove_depth()
            groove_depth_left.append(left)
            groove_depth_right.append(right)
            groove_depth_avg.append(avg)

        return groove_depth_left, groove_depth_right, groove_depth_avg

    def get_groove_depth_diff(self):

        res = []
        for s in self.samples:
            left, right, _ = s.get_groove_depth()
            res.append(abs(left - right))

        return res

    def get_groove_area(self):
        groove_area_scanner = [s.groove_area * 1e6 for s in self.samples]

        groove_area_abw = []
        for s in self.samples:
            v = np.linspace(s.mcs_delayed[0], s.mcs_delayed[6], num=5)

            groove_area_abw.append(
                sum(
                    [
                        polygon_area((v[0], s.mcs_delayed[1], s.mcs_delayed[2], v[1])),
                        polygon_area((v[1], s.mcs_delayed[2], s.mcs_delayed[3], v[2])),
                        polygon_area((v[2], s.mcs_delayed[3], s.mcs_delayed[4], v[3])),
                        polygon_area((v[3], s.mcs_delayed[4], s.mcs_delayed[5], v[4])),
                    ]
                )
            )

        return groove_area_scanner, groove_area_abw

    def get_groove_area_ratio(self):
        return [s.groove_area_ratio for s in self.samples]

    def get_weld_object_radius(self):
        return [s.weld_object_radius for s in self.samples]

    def get_slides(self, actual=False):
        return [s.slides_actual if actual else s.slides_desired for s in self.samples]

    def get_upper_groove_width_fluctuations(self):
        groove_width = [s.mcs[0][0] - s.mcs[-1][0] for s in self.samples]
        avg_groove_width = sum(groove_width) / len(groove_width)

        return [s.mcs[0][0] - s.mcs[-1][0] - avg_groove_width for s in self.samples]

    def get_groove_width(self):
        upper = []
        lower = []
        for s in self.samples:
            upper.append(s.mcs[0][0] - s.mcs[-1][0])
            lower.append(s.mcs[1][0] - s.mcs[-2][0])

        return (upper, lower)

    def get_abw_point(self, cs, point, x, delta=True):
        point_index = 0 if x else 1

        avg = 0
        if delta:
            all_points = [
                s.mcs[point][point_index] if cs == "mcs" else s.lpcs[point][point_index]
                for s in self.samples
            ]
            avg = sum(all_points) / len(all_points)

        data = []
        for s in self.samples:
            val = (
                s.mcs[point][point_index] if cs == "mcs" else s.lpcs[point][point_index]
            )

            data.append(val - avg)

        return data

    def add_plot_reference(self, ax, legend_loc=0):

        # add vertical lines for weld sessions
        weld_session_samples = self.get_timestamp_weld_session_samples()
        for weld_session_sample in weld_session_samples:
            ax.axvline(
                x=weld_session_sample,
                color="r",
                linestyle="--",
                linewidth=1.0,
            )

        # add plot reference on right side y axis
        ax2 = None
        match self.reference:
            case "abpop":
                ax2 = self.add_abp_op_to_plot(ax, color="green", linestyle="dashed")
            case "bead":
                ax2 = self.add_bead_number_to_plot(
                    ax, color="green", linestyle="dashed"
                )
            case "pos":
                ax2 = self.add_weld_axis_position_to_plot(
                    ax, color="green", linestyle="dashed"
                )

        if ax2:
            self.merge_plot_legends(ax, ax2, loc=legend_loc)

    def plot_abp_summary(self):
        x, xlabel = self.get_timeline()

        _, (ax1, ax2, ax3, ax4) = plt.subplots(4, layout="constrained")
        ax1.set_title("Bead operation")
        ax1.plot(x, self.get_bead_op())
        ax1.set_xlabel(xlabel)
        ax1.set_yticks([0, 1, 2, 3])
        ax1.set_yticklabels(["idle", "repositioning", "steady", "overlapping"])
        ax1.grid()
        self.add_plot_reference(ax=ax1, legend_loc=4)

        ax2.set_title("Layer number")
        ax2.plot(x, self.get_layer_no())
        ax2.set_xlabel(xlabel)
        ax2.grid()
        self.add_plot_reference(ax=ax2, legend_loc=4)

        ax3.set_title("Bead number")
        ax3.plot(x, self.get_bead_no())
        ax3.set_xlabel(xlabel)
        ax3.grid()
        self.add_plot_reference(ax=ax3, legend_loc=4)

        ax4.set_title("Linear velocity")
        ax4.plot(x, self.get_weld_object_lin_velocity())
        ax4.grid()
        ax4.set_ylabel("cm/min")
        ax4.set_xlabel(xlabel)
        self.add_plot_reference(ax=ax4, legend_loc=4)

    def add_weld_axis_position_to_plot(self, ax, color, linestyle):
        x, _ = self.get_timeline()

        ax2 = ax.twinx()
        ax2.plot(
            x,
            self.get_weld_object_pos(),
            color=color,
            linestyle=linestyle,
            linewidth=0.75,
            label="weld object pos",
        )
        ax2.set_ylabel("Position (deg)")

        return ax2

    def add_bead_number_to_plot(self, ax, color="green", linestyle="dashed"):
        x, _ = self.get_timeline()

        ax2 = ax.twinx()
        ax2.plot(
            x,
            self.get_bead_no(),
            color=color,
            linestyle=linestyle,
            linewidth=0.75,
            label="bead number",
        )
        ax2.set_ylabel("Bead number")

        return ax2

    def add_abp_op_to_plot(self, ax, color="green", linestyle="dashed"):
        x, _ = self.get_timeline()

        ax2 = ax.twinx()
        ax2.plot(
            x,
            self.get_bead_op(),
            color=color,
            linestyle=linestyle,
            linewidth=0.75,
            label="bead number",
        )
        ax2.set_yticks([0, 1, 2, 3])
        ax2.set_yticklabels(["idle", "repositioning", "steady", "overlapping"])
        ax2.set_ylabel("ABP Operation")

        return ax2

    def merge_plot_legends(self, ax, ax2, loc=0):
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, loc=loc)

    def plot_groove_width(self, ax):
        x, xlabel = self.get_timeline()
        upper, lower = self.get_groove_width()

        ax.set_title("Groove width")
        ax.plot(x, upper, "r-", label="upper groove width")
        ax.plot(x, lower, "b-", label="lower groove width")
        ax.set_ylabel("Width (mm)")
        ax.set_xlabel(xlabel)
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_groove_depth(self, ax):
        x, xlabel = self.get_timeline()

        ax.set_title("Groove depth")
        groove_depth_left, groove_depth_right, groove_depth_avg = (
            self.get_groove_depth()
        )
        ax.plot(x, groove_depth_left, "b-", label="groove depth left")
        ax.plot(x, groove_depth_right, "r-", label="groove depth right")
        ax.plot(x, groove_depth_avg, "c--", label="groove depth average")

        ax.grid()
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Depth (mm)")

        self.add_plot_reference(ax=ax, legend_loc=1)

    def plot_groove_area(self, ax):
        x, xlabel = self.get_timeline()

        _, groove_area_abw = self.get_groove_area()
        ax.plot(x, groove_area_abw, "r-", label="ABW area")

        ax.set_title("Groove area")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Area (mm2)")
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=1)

    def plot_groove_top_height(self, ax):
        x, xlabel = self.get_timeline()

        h = [s.mcs_delayed[0][1] - s.mcs_delayed[-1][1] for s in self.samples]

        ax.plot(x, h, "r-", label="height diff")

        ax.set_title("Groove top height diff")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Diff (mm)")
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=1)

    def plot_groove_summary(self):
        fig = plt.figure(layout="constrained")
        fig.suptitle("Groove")

        self.plot_groove_width(fig.add_subplot(411))
        self.plot_groove_depth(fig.add_subplot(412))
        self.plot_groove_area(fig.add_subplot(413))
        self.plot_groove_top_height(fig.add_subplot(414))

    def plot_abw_point_position(self, ax, cs, point, isx):
        x, xlabel = self.get_timeline()
        ax.plot(
            x,
            self.get_abw_point(point=point, cs=cs, x=isx, delta=True),
            "r-",
            label=f"abw{point}-{'X' if isx else 'Z' if cs == 'mcs' else 'Y'}",
        )
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Diff (mm)")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=1)

    def plot_abw_position_summary(self):
        fig = plt.figure()
        cs = "mcs"
        fig.suptitle(f"ABW 0 and 6 - {cs} coordinates wrt its average position")

        self.plot_abw_point_position(fig.add_subplot(221), cs, 0, True)
        self.plot_abw_point_position(fig.add_subplot(222), cs, 0, False)
        self.plot_abw_point_position(fig.add_subplot(223), cs, 6, True)
        self.plot_abw_point_position(fig.add_subplot(224), cs, 6, False)

    def get_mcs_limits(self):
        s = self.samples[0]
        xmin = s.mcs[0][0]
        xmax = s.mcs[-1][0]
        zmax = max(s.mcs[0][1], s.mcs[-1][1])
        zmin = min(s.mcs[1][1], s.mcs[-2][1])

        for s in self.samples:
            xmin = max(xmin, s.mcs[0][0])
            xmax = min(xmax, s.mcs[-1][0])
            zmax = max(zmax, s.mcs[0][1], s.mcs[-1][1])
            zmin = min(zmin, s.mcs[1][1], s.mcs[-2][1])

        return xmin, xmax, zmin, zmax

    def groove_animation(self):
        fig, ax = plt.subplots()

        s = self.samples[0]

        # def get_abw_points(sample):
        #     xs = []
        #     ys = []
        #     for x, y in sample.mcs:
        #         xs.append(x)
        #         ys.append(y)
        #
        #     return xs, ys

        (p,) = ax.plot(0, 0, color="g", linestyle="-", marker="*")

        px = []
        for _ in range(20):
            px.append(ax.plot(0, 0, linestyle=":")[0])
        ax2 = ax.scatter(
            s.slides_desired[0], s.slides_desired[1], color="black", marker="o"
        )

        xmin, xmax, zmin, zmax = self.get_mcs_limits()
        time_text = ax.text(
            0.5, 0.5, "", fontsize=8, bbox={"facecolor": "green", "alpha": 0.5}
        )

        groove_pos = [[]]
        last_pos = s.weld_object_pos
        tot = 0.0

        def update(frame):
            nonlocal tot
            nonlocal last_pos
            nonlocal px

            s = self.samples[frame]

            dist = s.weld_object_pos - last_pos
            if dist < 0:
                dist += 2 * math.pi

            tot += dist

            lap = int(tot / (2 * math.pi))

            if len(groove_pos) <= lap:
                groove_pos.append([])
            groove_pos[lap].append(s)

            last_pos = s.weld_object_pos

            xs, ys = s.get_abw_points()

            center_groove = True
            if center_groove:
                d = xs[0] - xs[-1]
                p.axes.set_xlim(xs[0] + (0.3 * d), xs[-1] - (0.1 * d))
                time_text.set_position((xs[0] + d * 0.25, zmin * 1.07))
            else:
                p.axes.set_xlim(xmin * 0.25, xmax * 1.02)
                time_text.set_position((xmin * 0.28, zmin * 1.07))
            p.axes.set_ylim(zmin * 1.1, zmax * 0.9)

            stickout = s.mode_data.stickout if s.mode_data else 25.0
            ax2.set_offsets((s.slides_desired[0], s.slides_desired[1] - stickout))

            desc = f"Time: {s.ts.strftime('%H:%M:%S')}"
            desc += f"\nMode: {s.mode.upper()}"
            desc += f"\nPosition(deg): {s.get_weld_object_pos():.1f}"
            desc += f"\nAvg depth(mm): {s.get_groove_depth()[2]:.1f}"
            if s.mode == "abp":
                desc += f"\nBead no: {s.bead_no}"
                desc += f"\nLayer no: {s.layer_no}"
                desc += f"\nOperation: {s.abp_op}"
                desc += f"\nProgress: {s.progress * 100:.1f}%"
                desc += f"\nBeadSliceAreaRatio: {s.bead_slice_area_ratio * 100:.1f}%"
            time_text.set_text(desc)

            px[len(groove_pos) - 1].set_linestyle("-")
            px[len(groove_pos) - 1].set_marker(".")

            for i in range(len(groove_pos) - 1):
                px[i].set_linestyle(":")
                px[i].set_marker("")

            for i in range(0, len(groove_pos)):
                pppp = groove_pos[i]
                initial_groove_sample = min(
                    pppp, key=lambda x: abs(x.weld_object_pos - s.weld_object_pos)
                )
                xss, yss = initial_groove_sample.get_abw_points()
                # connect previous beads ABW0 and ABW6 to this layers ABW1 and ABW5 to make the animation look nicer
                # if i != len(groove_pos) -1:
                #     xss[0] = xs[1]
                #     xss[6] = xs[5]
                #     yss[0] = ys[1]
                #     yss[6] = ys[5]
                px[i].set_data(xss, yss)

            return p, ax2, time_text

        ani = animation.FuncAnimation(
            fig=fig, func=update, frames=len(self.samples), interval=10, repeat=False
        )
        plt.show()

        # uncomment to store animation
        ani.save(filename=f"{self.name}.mp4", writer="ffmpeg", dpi=512, fps=60)

    def plot_heat_input(self, ax):
        x, xlabel = self.get_timeline()
        wss = [s.weld_systems for s in self.samples]

        ws1_heat_input = [ws[1].heat_input for ws in wss]
        ws2_heat_input = [ws[2].heat_input for ws in wss]
        ws_heat_input = np.array(
            [x + y for x, y in zip(ws1_heat_input, ws2_heat_input)]
        ).flatten()
        ax.plot(x, ws1_heat_input, label="weld-system-1")
        ax.plot(x, ws2_heat_input, label="weld-system-2")
        ax.plot(x, ws_heat_input, label="total")
        ax.set_title("Weld-systems heat input")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("kj/mm")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=3)

    def plot_heat_input(self, ax, only_total=False, show_range=False):
        x, xlabel = self.get_timeline()
        wss = [s.weld_systems for s in self.samples]

        ws1_heat_input = [ws[1].heat_input for ws in wss]
        ws2_heat_input = [ws[2].heat_input for ws in wss]
        ws_heat_input = [x + y for x, y in zip(ws1_heat_input, ws2_heat_input)]
        if not only_total:
            ax.plot(x, ws1_heat_input, label="weld-system-1")
            ax.plot(x, ws2_heat_input, label="weld-system-2")
        ax.plot(x, ws_heat_input, label="total")
        ax.set_title("Weld-systems heat input")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("kj/mm")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=3)

        mode_data = self.samples[-1].mode_data
        if show_range and mode_data:
            range_min = mode_data.heat_input[0]
            range_max = mode_data.heat_input[1]
            ax.axhspan(
                range_min,
                range_max,
                facecolor="green",
                alpha=0.3,
            )

            space = 0.1
            ylim_min = min(ws_heat_input + [range_min]) - space
            ylim_max = max(ws_heat_input + [range_max]) + space

            ax.set_ylim(ylim_min, ylim_max)

    def plot_weld_systems(self):
        fig = plt.figure(layout="constrained")

        x, xlabel = self.get_timeline()

        wss = [s.weld_systems for s in self.samples]

        ax = fig.add_subplot(321)
        ax.plot(x, [ws[1].voltage for ws in wss], label="weld-system-1")
        ax.plot(x, [ws[2].voltage for ws in wss], label="weld-system-2")
        ax.set_title("Weld-systems voltage")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Voltage (V)")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=3)

        ax = fig.add_subplot(322)
        ax.plot(x, [ws[1].current for ws in wss], label="weld-system-1")
        ax.plot(x, [ws[2].current for ws in wss], label="weld-system-2")
        ax.set_title("Weld-systems current")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Ampere (A)")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=3)

        self.plot_heat_input(fig.add_subplot(323))

        ax = fig.add_subplot(324)
        ax.plot(x, [ws[1].deposition_rate for ws in wss], label="weld-system-1")
        ax.plot(x, [ws[2].deposition_rate for ws in wss], label="weld-system-2")
        ax.set_title("Weld-systems deposition rate")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("kg/h")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=3)

        ax = fig.add_subplot(325)
        ax.plot(x, [ws[1].wire_speed for ws in wss], label="weld-system-1")
        ax.plot(x, [ws[2].wire_speed for ws in wss], label="weld-system-2")
        ax.set_title("Weld-systems wire-speed")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("mm/s")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=3)

        data = self.get_bead_area()
        ws1_bead_area = np.array([x[1] for x in data]).flatten()
        ws2_bead_area = np.array([x[2] for x in data]).flatten()
        ws_bead_area = np.array(
            [x + y for x, y in zip(ws1_bead_area, ws2_bead_area)]
        ).flatten()

        ax = fig.add_subplot(326)
        ax.set_title("Bead Area")
        ax.set_xlabel(xlabel)
        ax.grid()
        ax.plot(x, ws1_bead_area, label="weld-system-1")
        ax.plot(x, ws2_bead_area, label="weld-system-2")
        ax.plot(x, ws_bead_area, label="total")
        ax.set_ylabel("mm2")
        self.add_plot_reference(ax=ax, legend_loc=2)

    def plot_weld_speed(self, ax, show_range=False):
        x, xlabel = self.get_timeline()

        weld_speed = self.get_weld_object_lin_velocity_desired()
        ax.set_title("Weld speed")
        ax.plot(
            x,
            weld_speed,
            "r-",
            label="desired",
        )
        ax.plot(x, self.get_weld_object_lin_velocity(), "b-", label="actual")
        ax.set_ylabel("Weld speed (cm/min)")
        ax.set_xlabel(xlabel)
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=4)

        mode_data = self.samples[-1].mode_data
        if show_range and mode_data:
            range_min = mode_data.weld_speed[0]
            range_max = mode_data.weld_speed[1]
            ax.axhspan(
                range_min,
                range_max,
                facecolor="green",
                alpha=0.3,
            )

            space = 5
            ylim_min = min(weld_speed + [range_min]) - space
            ylim_max = max(weld_speed + [range_max]) + space

            ax.set_ylim(ylim_min, ylim_max)

    def plot_weld_speed_diff(self, ax):
        x, xlabel = self.get_timeline()

        ax.set_title("Weld speed diff")
        ax.plot(
            x,
            [
                mm_sec_to_cm_min(
                    (
                        s.weld_object_ang_velocity_desired
                        - s.weld_object_ang_velocity_actual
                    )
                    * s.weld_object_radius
                )
                for s in self.samples
            ],
            "b-",
            label="desired - actual",
        )
        ax.set_ylabel("Weld speed (cm/min)")
        ax.set_xlabel(xlabel)
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_groove_area_ratio(self, ax):
        x, xlabel = self.get_timeline()

        ax.set_title("Groove area ratio")
        ax.plot(x, [v * 100.0 for v in self.get_groove_area_ratio()], "b-", label="")
        ax.set_ylabel("(%)")
        ax.set_xlabel(xlabel)
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_groove_depth_diff(self, ax):
        x, xlabel = self.get_timeline()

        res = []
        for s in self.samples:
            if s.bead_no == 1:
                left, right, _ = s.get_groove_depth()
                res.append(left - right)
            else:
                res.append(0.0)

        ax.set_title("Groove depth diff")
        ax.plot(x, ema(res, 0.1), "b-", label="")
        ax.set_ylabel("(mm)")
        ax.set_xlabel(xlabel)
        ax.grid()

        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_position_adaptivity(self, ax):
        x, xlabel = self.get_timeline()

        layer_bead_pos = {}

        for layer in self.get_layers():
            layer_no = layer.samples[0].layer_no
            layer_bead_pos[layer_no] = {}
            for bead in layer.get_beads():
                bead_no = bead.samples[0].bead_no
                hpos = []
                for s in bead.samples:
                    if s.abp_op == "steady":
                        hpos.append(s.bead_control_horizontal_offset)

                if hpos:
                    layer_bead_pos[layer_no][bead_no] = sum(hpos) / len(hpos)

        rel_pos = []
        for s in self.samples:
            if s.abp_op == "steady":
                rel_pos.append(
                    layer_bead_pos[s.layer_no][s.bead_no]
                    - s.bead_control_horizontal_offset
                )
            else:
                rel_pos.append(0.0)

        ax.set_title("Bead position adjustment")
        ax.plot(x, rel_pos, "b-", label="")
        ax.set_ylabel("mm")
        ax.set_xlabel(xlabel)
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_bead_slice_area_ratio(self, ax):
        x, xlabel = self.get_timeline()

        res = [s.bead_slice_area_ratio * 100.0 for s in self.samples]

        ax.set_title("Bead slice area ratio")
        ax.plot(x, res, "b-", label="")
        ax.set_ylabel("%")
        ax.set_xlabel(xlabel)
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_weld_system2_current(self, ax):
        x, xlabel = self.get_timeline()

        wss = [s.weld_systems for s in self.samples]

        ws2_current = [ws[2].current for ws in wss]
        # ax.plot(x, ws2_current, label="weld-system-2")
        ax.plot(x, ws2_current, label="weld-system-2")
        ax.set_title("Weld-system-2 current")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Ampere (A)")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=4)

        mode_data = self.samples[-1].mode_data
        if mode_data:
            range_min = mode_data.ws2_current[0]
            range_max = mode_data.ws2_current[1]
            ax.axhspan(
                range_min,
                range_max,
                facecolor="green",
                alpha=0.3,
            )

            space = 10
            ylim_min = min(ws2_current + [range_min]) - space
            ylim_max = max(ws2_current + [range_max]) + space

            ax.set_ylim(ylim_min, ylim_max)

    def plot_slides(self, ax, horizontal=True):
        x, xlabel = self.get_timeline()

        indx = 0 if horizontal else 1

        slides_actual = self.get_slides(actual=True)
        slides_desired = self.get_slides(actual=False)
        ax.plot(x, [s[indx] for s in slides_actual], "b-", label="actual")
        ax.plot(x, [s[indx] for s in slides_desired], "r-", label="desired")
        ax.set_title(f"Slides {"horizontal" if horizontal else "vertical"} position")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("mm")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_slides_diff(self, ax, horizontal=True):
        x, xlabel = self.get_timeline()

        indx = 0 if horizontal else 1

        diff = [s.slides_desired[indx] - s.slides_actual[indx] for s in self.samples]

        ax.plot(x, diff, "r-", label="delta")
        ax.set_title(
            f"Slides {"horizontal" if horizontal else "vertical"} desired - actual"
        )
        ax.set_xlabel(xlabel)
        ax.set_ylabel("position (mm)")
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=4)

    def plot_adaptivity(self):
        fig = plt.figure(layout="constrained")
        fig.suptitle("Adaptivity - weld-speed")
        self.plot_groove_area(fig.add_subplot(311))
        self.plot_groove_area_ratio(fig.add_subplot(312))
        self.plot_weld_speed(fig.add_subplot(313), show_range=True)

        fig = plt.figure(layout="constrained")
        fig.suptitle("Adaptivity - bead position")
        self.plot_groove_top_height(fig.add_subplot(211))
        self.plot_position_adaptivity(fig.add_subplot(212))

        fig = plt.figure(layout="constrained")
        fig.suptitle("Adaptivity - weld current")
        self.plot_bead_slice_area_ratio(fig.add_subplot(211))
        self.plot_weld_system2_current(fig.add_subplot(212))

        fig = plt.figure(layout="constrained")
        fig.suptitle("Adaptivity - result")
        self.plot_groove_depth_diff(fig.add_subplot(211))
        self.plot_heat_input(fig.add_subplot(212), only_total=True, show_range=True)

    def plot_slides_summary(self):
        fig = plt.figure(layout="constrained")
        fig.suptitle("Slides")
        self.plot_slides(fig.add_subplot(211))
        self.plot_slides_diff(fig.add_subplot(212))

    def plot_weld_axis(self):
        fig = plt.figure(layout="constrained")
        fig.suptitle("Weld axis")
        self.plot_weld_speed(fig.add_subplot(211))
        self.plot_weld_speed_diff(fig.add_subplot(212))

    def plot_groove_progress(self):

        pos = 0.3 * 2 * math.pi
        fig = plt.figure(layout="constrained")
        fig.suptitle(f"Groove progress - pos: {pos:.2f}")

        layer_steady_samples = {}

        max_beads = 0
        layers = self.get_layers()
        for layer in layers:
            if len(layer.samples) <= MIN_SAMPLES:
                continue
            layer_no = layer.samples[0].layer_no
            beads = layer.get_beads()
            max_beads = max(max_beads, len(beads))

            layer_steady_samples[layer_no] = []
            for bead in beads:
                layer_steady_samples[layer_no].append(
                    bead.get_bead_op_samples("steady")
                )

        cols = max_beads - 1
        rows = len(layer_steady_samples)

        xmin = 0
        xmax = 0
        ymin = 0
        ymax = 0

        plot_indx = 0
        empty_groove = None
        prev_groove = {}
        prev_layer_no = 0
        prev_bead_no = 0
        prev_s = None

        for layer_indx, (layer_no, beads) in enumerate(layer_steady_samples.items()):
            for bead_indx, bead in enumerate(beads):
                s = bead.get_sample(pos)
                bead_no = s.bead_no

                if layer_indx == 0 and bead_indx == 0:
                    xmin = s.mcs_delayed[0][0] + 5
                    xmax = s.mcs_delayed[-1][0] - 5
                    ymin = min(s.mcs_delayed[1][1], s.mcs_delayed[-2][1]) - 5
                    ymax = max(s.mcs_delayed[0][1], s.mcs_delayed[-1][1]) + 5

                    empty_groove = s

                if bead_no == 2:
                    plot_indx = (layer_no * cols) + 1
                else:
                    plot_indx += 1

                ax = fig.add_subplot(rows + 1, cols, plot_indx)
                ax.set_title(f"Layer: {prev_layer_no} bead: {prev_bead_no}")
                ax.plot(
                    [p[0] for p in s.mcs_delayed], [p[1] for p in s.mcs_delayed], "r-"
                )
                ax.plot(
                    [p[0] for p in empty_groove.mcs_delayed],
                    [p[1] for p in empty_groove.mcs_delayed],
                    "b--",
                )

                if False and bead_no in prev_groove:
                    ax.plot(
                        [p[0] for p in prev_groove[bead_no].mcs_delayed],
                        [p[1] for p in prev_groove[bead_no].mcs_delayed],
                        "g--",
                    )
                if True and prev_s:
                    ax.plot(
                        [p[0] for p in prev_s.mcs_delayed],
                        [p[1] for p in prev_s.mcs_delayed],
                        "g--",
                    )

                ax.set_xlim(xmin, xmax)
                ax.set_ylim(ymin, ymax)
                # ax.set_ylabel("Weld speed (cm/min)")
                # ax.set_xlabel(xlabel)
                ax.grid()

                prev_groove[bead_no] = s
                prev_layer_no = layer_no
                prev_bead_no = bead_no
                prev_s = s

    def plot_misc(self):
        fig = plt.figure(layout="constrained")
        fig.suptitle("Misc")

        x, xlabel = self.get_timeline()

        # plot weld-system current to wire-speed
        ax = fig.add_subplot(212)

        wss = [s.weld_systems for s in self.samples]
        ws1 = [ws[1].wire_speed / ws[1].current for ws in wss]
        ws2 = [ws[2].wire_speed / ws[2].current for ws in wss]

        ax.set_title("Wire-speed to current (smooth)")
        ax.plot(x, ema(ws1, 0.1), "b-", label="weld-system-1")
        ax.plot(x, ema(ws2, 0.1), "r-", label="weld-system-2")
        ax.set_ylabel("")
        ax.set_xlabel(xlabel)
        ax.grid()
        self.add_plot_reference(ax=ax, legend_loc=4)

    def abp_summary(self):
        layers = self.get_layers()
        groove_area = []

        if len(self.samples) < MIN_SAMPLES:
            print("-- too few samples ignore sequence --")
            return

        for layer in layers:
            if len(layer.samples) < MIN_SAMPLES:
                print("-- too few samples ignore layer --")
                continue

            print(
                f"   layer: {layer.samples[0].layer_no}: (log samples: {len(layer.samples)})"
            )

            for bead in layer.get_beads():
                if len(bead.samples) < MIN_SAMPLES:
                    print("-- too few samples ignore bead --")
                    continue

                bead_area = bead.get_total_bead_area()
                groove_area_abw = bead.get_groove_area()[1]
                groove_area_total = sum(groove_area_abw) / len(groove_area_abw)

                print(
                    f"     bead: {bead.samples[0].bead_no}: (log samples: {len(bead.samples)})"
                )
                print(f"       calculated bead area: {bead_area:.1f} mm2")
                print(f"       ABW groove area: {groove_area_total:.1f} mm2")
                if len(groove_area) > 0:
                    print(
                        f"       diff ABW groove area compared to prev lap: {groove_area[-1] - groove_area_total:.1f} mm2"
                    )

                groove_area.append(groove_area_total)

            empty_groove = []
            for s in layer.samples:
                if s.bead_no == 1:
                    empty_groove.append(s)
                elif s.bead_no > 1:
                    break


class Sample:
    def __init__(
        self,
        ts,
        ts_scanner,
        mode,
        abp_op,
        bead_no,
        layer_no,
        progress,
        weld_object_pos,
        weld_object_radius,
        weld_object_ang_velocity_actual,
        weld_object_ang_velocity_desired,
        mcs,
        mcs_delayed,
        lpcs,
        slides_desired,
        slides_actual,
        groove_area,
        groove_area_ratio,
        bead_control_horizontal_offset,
        bead_slice_area_ratio,
        mode_data,
    ):
        self.ts = ts
        self.ts_scanner = ts_scanner
        self.mode = mode
        self.abp_op = abp_op
        self.bead_no = bead_no
        self.layer_no = layer_no
        self.progress = progress
        self.weld_systems = {}
        self.weld_object_pos = weld_object_pos
        self.weld_object_radius = weld_object_radius
        self.weld_object_ang_velocity_actual = weld_object_ang_velocity_actual
        self.weld_object_ang_velocity_desired = weld_object_ang_velocity_desired
        self.mcs = mcs
        self.mcs_delayed = mcs_delayed
        self.lpcs = lpcs
        self.slides_desired = slides_desired
        self.slides_actual = slides_actual
        self.groove_area = groove_area
        self.groove_area_ratio = groove_area_ratio
        self.bead_control_horizontal_offset = bead_control_horizontal_offset
        self.bead_slice_area_ratio = bead_slice_area_ratio
        self.mode_data = mode_data

    def scanner_delay(self):
        return self.ts - self.ts_scanner

    def add_weld_system(self, index, weld_system):
        self.weld_systems[index] = weld_system

    def get_weld_object_lin_velocity(self):
        # convert rad/sec to mm/sec
        return self.weld_object_ang_velocity_actual * self.weld_object_radius

    def get_weld_object_lin_velocity_desired(self):
        # convert rad/sec to mm/sec
        return self.weld_object_ang_velocity_desired * self.weld_object_radius

    def get_bead_area(self):
        bead_area = {}
        for index, ws in self.weld_systems.items():
            area = 0
            if self.get_weld_object_lin_velocity() > 3:
                area = (
                    math.pi
                    * ws.wire_speed
                    * pow((ws.wire_diameter / 2), 2)
                    / self.get_weld_object_lin_velocity()
                )
                area = 2 * area if ws.twin_wire else area

            bead_area[index] = area

        return bead_area

    def get_wire_speed(self):
        res = {}
        for index, ws in self.weld_systems.items():
            res[index] = ws.wire_speed
        return res

    def get_weld_object_pos(self, deg=True):
        return self.weld_object_pos * 180 / math.pi if deg else self.weld_object_pos

    def get_groove_depth(self):
        if self.mcs:
            left = self.mcs[0][1] - self.mcs[1][1]
            right = self.mcs[-1][1] - self.mcs[-2][1]
            return left, right, (left + right) / 2
        return 0, 0, 0

    def get_abw_points(self):
        xs = []
        ys = []
        for x, y in self.mcs_delayed:
            xs.append(x)
            ys.append(y)

        return xs, ys


class WeldSystem:
    def __init__(
        self,
        state,
        current,
        voltage,
        twin_wire,
        wire_diameter,
        wire_speed,
        heat_input,
        deposition_rate,
    ):
        self.state = state
        self.current = current
        self.voltage = voltage
        self.twin_wire = twin_wire
        self.wire_diameter = wire_diameter
        self.wire_speed = wire_speed
        self.heat_input = heat_input
        self.deposition_rate = deposition_rate


class ModeData:
    def __init__(
        self, ts, mode, stickout=0.0, heat_input=None, weld_speed=None, ws2_current=None
    ):
        self.ts = ts
        self.mode = mode
        self.stickout = stickout
        self.heat_input = heat_input
        self.weld_speed = weld_speed
        self.ws2_current = ws2_current


def smooth(y, box_pts):
    box = np.ones(box_pts) / box_pts
    y_smooth = np.convolve(y, box, mode="same")
    return y_smooth


def weldcontrolparser(
    paths,
    reference,
    plot_abp,
    plot_groove,
    plot_abw_points,
    plot_groove_animation,
    plot_weld_systems,
    plot_adaptivity,
    plot_slides,
    plot_weld_axis,
    plot_groove_progress,
    plot_misc,
):
    samples = []
    weld_sessions = []

    for path in paths:
        mode_data = None
        for indx, ln in enumerate(open(file=path, mode="r", encoding="utf-8")):
            data = json.loads(ln)

            ts = datetime.strptime(data["timestamp"], "%Y-%m-%dT%H:%M:%S.%f")
            logtype = data["type"] if "type" in data else "data"

            if indx == 0:
                weld_sessions.append(ts)

            if logtype == "data":
                ts_scanner = datetime.strptime(
                    data["timestampScanner"], "%Y-%m-%dT%H:%M:%S.%f"
                )

                mcs = []
                for m in data["mcs"]:
                    x = m["x"]
                    z = m["z"]
                    mcs.append((x, z))

                mcs_delayed = []
                if "mcsDelayed" in data:
                    for m in data["mcsDelayed"]:
                        x = m["x"]
                        z = m["z"]
                        mcs_delayed.append((x, z))

                lpcs = []
                for m in data["lpcs"]:
                    x = m["x"]
                    y = m["y"]
                    lpcs.append((x, y))

                slides_desired = (0.0, 0.0)
                slides_actual = (0.0, 0.0)
                if "slides" in data:
                    slides_desired = (
                        data["slides"]["desired"]["horizontal"],
                        data["slides"]["desired"]["vertical"],
                    )
                    slides_actual = (
                        data["slides"]["actual"]["horizontal"],
                        data["slides"]["actual"]["vertical"],
                    )

                s = Sample(
                    ts=ts,
                    ts_scanner=ts_scanner,
                    mode=data["mode"],
                    abp_op=data["beadControl"]["state"],
                    bead_no=data["beadControl"]["beadNo"],
                    layer_no=data["beadControl"]["layerNo"],
                    progress=data["beadControl"]["progress"],
                    weld_object_pos=data["weldAxis"]["position"],
                    weld_object_ang_velocity_actual=data["weldAxis"]["velocity"][
                        "actual"
                    ],
                    weld_object_ang_velocity_desired=data["weldAxis"]["velocity"][
                        "desired"
                    ],
                    weld_object_radius=data["weldAxis"]["radius"],
                    mcs=mcs,
                    mcs_delayed=mcs_delayed,
                    lpcs=lpcs,
                    slides_desired=slides_desired,
                    slides_actual=slides_actual,
                    groove_area=data["grooveArea"] if "grooveArea" in data else 0.0,
                    groove_area_ratio=(
                        data["beadControl"]["grooveAreaRatio"]
                        if "grooveAreaRatio" in data["beadControl"]
                        else 0.0
                    ),
                    bead_control_horizontal_offset=(
                        data["beadControl"]["horizontalOffset"]
                        if "horizontalOffset" in data["beadControl"]
                        else 0.0
                    ),
                    bead_slice_area_ratio=(
                        data["beadControl"]["beadSliceAreaRatio"]
                        if "beadSliceAreaRatio" in data["beadControl"]
                        else 0.0
                    ),
                    mode_data=mode_data,
                )

                for indx, wsdata in enumerate(data["weldSystems"]):
                    s.add_weld_system(
                        indx + 1,
                        WeldSystem(
                            state=wsdata["state"],
                            current=wsdata["current"],
                            voltage=wsdata["voltage"],
                            twin_wire=wsdata["twinWire"],
                            wire_diameter=wsdata["wireDiameter"],
                            wire_speed=wsdata["wireSpeed"],
                            heat_input=wsdata["heatInput"],
                            deposition_rate=wsdata["depositionRate"],
                        ),
                    )
                samples.append(s)

            elif logtype == "modeChange":
                mode = data["mode"]

                if mode == "jt":
                    mode_data = ModeData(
                        ts=ts, mode=mode, stickout=data["verticalOffset"]
                    )
                elif mode == "abp":
                    mode_data = ModeData(
                        ts=ts,
                        mode=mode,
                        stickout=data["verticalOffset"],
                        heat_input=(
                            data["abpParameters"]["heatInput"]["min"],
                            data["abpParameters"]["heatInput"]["max"],
                        ),
                        weld_speed=(
                            data["abpParameters"]["weldSpeed"]["min"],
                            data["abpParameters"]["weldSpeed"]["max"],
                        ),
                        ws2_current=(
                            data["abpParameters"]["weldSystem2Current"]["min"],
                            data["abpParameters"]["weldSystem2Current"]["max"],
                        ),
                    )
                else:
                    mode_data = ModeData(ts=ts, mode=mode)

    # abp_seqs = samples.get_abp_samples()
    # for indx, seq in enumerate(abp_seqs):
    # seq = samples.filter_mode(mode="abp")

    samples = Samples(
        reference=reference,
        samples=samples,
        weld_sessions=weld_sessions,
    )

    seq = samples.join_abp_samples()
    indx = 0

    if seq:
        first_ts = seq.get_first_ts()
        last_ts = seq.get_last_ts()
        print(f"ABP seq: {indx + 1}")
        print(f" start time: {first_ts.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f" end time:   {last_ts.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f" duration:   {last_ts - first_ts}")

        seq.abp_summary()

        if plot_abp:
            seq.plot_abp_summary()

        if plot_groove:
            seq.plot_groove_summary()

        if plot_abw_points:
            seq.plot_abw_position_summary()

        if plot_groove_animation:
            seq.groove_animation()

        if plot_weld_systems:
            seq.plot_weld_systems()

        if plot_adaptivity:
            seq.plot_adaptivity()

        if plot_slides:
            seq.plot_slides_summary()

        if plot_weld_axis:
            seq.plot_weld_axis()

        if plot_groove_progress:
            seq.plot_groove_progress()

        if plot_misc:
            seq.plot_misc()

    plt.show()


def get_file_list_from_yaml(yaml_filepath: str, groove_selection: str) -> list[str]:
    try:
        yaml_dir = os.path.dirname(os.path.abspath(yaml_filepath))

        print(f"yaml_dir: {yaml_dir}")
        with open(file=yaml_filepath, mode="r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
            if (
                data
                and groove_selection in data
                and isinstance(data[groove_selection], list)
            ):
                relative_paths = [
                    os.path.join(yaml_dir, path) for path in data[groove_selection]
                ]
                return relative_paths
            else:
                return []
    except FileNotFoundError:
        print(f"Error: File not found at {yaml_filepath}")
        return []
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return []


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Weld control log parser")
    parser.add_argument("--all", action="store_true", help="Plot all graphs.")
    parser.add_argument("--abp", action="store_true", help="Plot ABP summary graphs.")
    parser.add_argument("--groove", action="store_true", help="Plot Groove graphs.")
    parser.add_argument(
        "--adaptivity", action="store_true", help="Plot Adaptivity graphs."
    )
    parser.add_argument(
        "--abw-points", action="store_true", help="Plot ABW 0 and 6 point MCS X/Z"
    )
    parser.add_argument(
        "--weld-systems", action="store_true", help="Plot weld-system graphs"
    )
    parser.add_argument(
        "--groove-animation", action="store_true", help="Plot groove animation"
    )
    parser.add_argument("--slides", action="store_true", help="Plot slides graphs")
    parser.add_argument(
        "--weld-axis", action="store_true", help="Plot weld axis graphs"
    )
    parser.add_argument(
        "--groove-progress", action="store_true", help="Plot groove progress graphs"
    )
    parser.add_argument("--join", action="store_true", help="Join multiple ")
    parser.add_argument("--misc", action="store_true", help="Plot misc graphs")
    parser.add_argument(
        "--groove-selection",
        help="Groove selection from groove.yaml input file",
    )
    parser.add_argument(
        "--reference",
        default="none",
        choices=["pos", "abpop", "bead", "none"],
        help="Plot reference",
    )
    parser.add_argument(
        "path", type=pathlib.Path, help="weld control log location or groove.yaml file"
    )

    args = parser.parse_args()

    plot_abp = args.abp
    plot_groove = args.groove
    plot_abw_points = args.abw_points
    plot_groove_animation = args.groove_animation
    plot_weld_systems = args.weld_systems
    plot_adaptivity = args.adaptivity
    plot_slides = args.slides
    plot_weld_axis = args.weld_axis
    plot_groove_progress = args.groove_progress
    plot_misc = args.misc

    paths = []

    groove_selection = args.groove_selection
    if groove_selection:
        paths = get_file_list_from_yaml(args.path, groove_selection)
    else:
        paths = [args.path]

    if args.all:
        plot_abp = True
        plot_groove = True
        plot_abw_points = True
        plot_weld_systems = True
        plot_adaptivity = True

    try:
        weldcontrolparser(
            paths,
            args.reference,
            plot_abp,
            plot_groove,
            plot_abw_points,
            plot_groove_animation,
            plot_weld_systems,
            plot_adaptivity,
            plot_slides,
            plot_weld_axis,
            plot_groove_progress,
            plot_misc,
        )
    except KeyboardInterrupt:
        pass
    finally:
        print("-= Exit =-")
