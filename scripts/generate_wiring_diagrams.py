#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: generate_wiring_diagrams.py
Description: Generates publication-grade vector (SVG) and high-resolution (PNG)
             hardware wiring schematics for:
               1. Current Prototype (v0.8.0): Waveshare ESP32-S3-Touch-LCD-3.5B +
                  Holybro UM982 (Single-UART COM1 + Master & Slave Antennas) +
                  Ebyte E220-900T30D/S (900 MHz LoRa + Antenna) +
                  SparkFun Qwiic MMC5983MA 3-Axis 18-Bit AMR Compass (0x30).
               2. Next-Generation Smart Pole: Waveshare ESP32-S3-Touch-LCD-3.49-V2
                  (J3 22-Pin Header + 18650 Battery) + Discrete High-Side Power Switch
                  (NDP6020P P-FET + 2N3904 NPN + MT3608 5.0V Boost) +
                  Holybro UM982 (Single-UART COM1 + Master & Slave Antennas) +
                  Ebyte E220-900T30D/S (900 MHz LoRa + Antenna) +
                  SparkFun Qwiic MMC5983MA Compass (0x30).
Author: Roland L. Guilmet & Antigravity
License: GPL-3.0
"""

import argparse
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Polygon, Rectangle

__filename__ = os.path.basename(__file__)
__version__ = "v1.3.1"


def draw_chip_box(ax, x, y, w, h, title, subtitle, facecolor="#1E293B", edgecolor="#38BDF8", title_color="#F8FAFC", sub_color="#94A3B8", lw=2.2):
    box = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.18",
        facecolor=facecolor, edgecolor=edgecolor, linewidth=lw, zorder=3
    )
    ax.add_patch(box)
    hdr_h = 0.72 if subtitle else 0.52
    hdr = FancyBboxPatch(
        (x + 0.05, y + h - hdr_h - 0.05), w - 0.10, hdr_h,
        boxstyle="round,pad=0.01,rounding_size=0.12",
        facecolor="#0F172A", edgecolor="none", zorder=4
    )
    ax.add_patch(hdr)
    ax.text(
        x + w / 2.0, y + h - 0.26, title,
        color=title_color, fontsize=11, fontweight="bold",
        ha="center", va="center", zorder=5, family="monospace"
    )
    if subtitle:
        ax.text(
            x + w / 2.0, y + h - 0.55, subtitle,
            color=sub_color, fontsize=8.4, fontweight="normal",
            ha="center", va="center", zorder=5, family="sans-serif"
        )


def draw_pin_left(ax, x, y, label, pin_color="#E2E8F0", dot_color="#38BDF8"):
    c = Circle((x, y), 0.075, facecolor=dot_color, edgecolor="#0F172A", linewidth=1.2, zorder=6)
    ax.add_patch(c)
    ax.text(
        x + 0.16, y, label,
        color=pin_color, fontsize=8.2, fontweight="bold",
        ha="left", va="center", zorder=6, family="monospace"
    )


def draw_pin_right(ax, x, y, label, pin_color="#E2E8F0", dot_color="#38BDF8"):
    c = Circle((x, y), 0.075, facecolor=dot_color, edgecolor="#0F172A", linewidth=1.2, zorder=6)
    ax.add_patch(c)
    ax.text(
        x - 0.16, y, label,
        color=pin_color, fontsize=8.2, fontweight="bold",
        ha="right", va="center", zorder=6, family="monospace"
    )


def draw_pin_top(ax, x, y, label, pin_color="#E2E8F0", dot_color="#F59E0B"):
    c = Circle((x, y), 0.08, facecolor=dot_color, edgecolor="#0F172A", linewidth=1.2, zorder=6)
    ax.add_patch(c)
    ax.text(
        x + 0.14, y + 0.10, label,
        color=pin_color, fontsize=7.8, fontweight="bold",
        ha="left", va="bottom", zorder=7, family="monospace",
        bbox=dict(boxstyle="round,pad=0.10", facecolor="#0B1120", edgecolor="none", alpha=0.88)
    )


def draw_pin_bottom(ax, x, y, label, pin_color="#E2E8F0", dot_color="#38BDF8"):
    c = Circle((x, y), 0.075, facecolor=dot_color, edgecolor="#0F172A", linewidth=1.2, zorder=6)
    ax.add_patch(c)
    ax.text(
        x, y + 0.16, label,
        color=pin_color, fontsize=7.8, fontweight="bold",
        ha="center", va="bottom", zorder=6, family="monospace"
    )


def draw_antenna(ax, x, y, title, subtitle, color="#F59E0B"):
    """Draws a schematic RF antenna symbol at (x, y) base with a mast and radiating cone."""
    ax.plot([x, x], [y, y + 0.68], color=color, linewidth=2.4, zorder=5)
    ax.plot([x - 0.34, x, x + 0.34], [y + 1.06, y + 0.58, y + 1.06], color=color, linewidth=2.2, zorder=5)
    ax.plot([x - 0.34, x + 0.34], [y + 1.06, y + 1.06], color=color, linewidth=1.8, zorder=5)
    pill = FancyBboxPatch(
        (x - 1.60, y + 1.18), 3.20, 0.72,
        boxstyle="round,pad=0.02,rounding_size=0.12",
        facecolor="#0F172A", edgecolor=color, linewidth=1.6, zorder=5
    )
    ax.add_patch(pill)
    ax.text(x, y + 1.66, title, color="#F8FAFC", fontsize=9.0, fontweight="bold", ha="center", va="center", zorder=6, family="monospace")
    ax.text(x, y + 1.38, subtitle, color="#CBD5E1", fontsize=7.8, ha="center", va="center", zorder=6, family="sans-serif")


def draw_wire(ax, pts, color="#38BDF8", lw=2.0, ls="-", label=None, label_pos=None, arrow_end=False, arrow_start=False):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, linewidth=lw, linestyle=ls, solid_capstyle="round", zorder=4)
    if arrow_end and len(pts) >= 2:
        p0, p1 = pts[-2], pts[-1]
        ax.annotate(
            "", xy=p1, xytext=p0,
            arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=13),
            zorder=5
        )
    if arrow_start and len(pts) >= 2:
        p0, p1 = pts[1], pts[0]
        ax.annotate(
            "", xy=p1, xytext=p0,
            arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=13),
            zorder=5
        )
    if label and label_pos:
        lx, ly = label_pos
        ax.text(
            lx, ly, label,
            color=color, fontsize=7.6, fontweight="bold",
            ha="center", va="bottom", zorder=7, family="monospace",
            bbox=dict(boxstyle="round,pad=0.12", facecolor="#0B1120", edgecolor="none", alpha=0.92)
        )


def generate_3_5b_diagram(out_dir: str):
    fig, ax = plt.subplots(figsize=(21, 14.0), dpi=200)
    fig.patch.set_facecolor("#0B1120")
    ax.set_facecolor("#0B1120")
    ax.set_xlim(0, 21)
    ax.set_ylim(0, 14.0)
    ax.axis("off")

    # Title Banner
    title_box = FancyBboxPatch(
        (0.4, 12.80), 20.2, 0.92,
        boxstyle="round,pad=0.02,rounding_size=0.15",
        facecolor="#1E293B", edgecolor="#38BDF8", linewidth=2.0, zorder=3
    )
    ax.add_patch(title_box)
    ax.text(
        10.5, 13.42,
        "RTK ROVER (v0.8.0) - CURRENT GENERATION HARDWARE WIRING SCHEMATIC",
        color="#F8FAFC", fontsize=14, fontweight="bold", ha="center", va="center", family="monospace", zorder=5
    )
    ax.text(
        10.5, 13.02,
        "Waveshare ESP32-S3-Touch-LCD-3.5B  +  Holybro UM982 (Single-UART COM1)  +  Ebyte E220-900T30D/S (1W LoRa)  +  SparkFun MMC5983MA Compass (0x30)",
        color="#38BDF8", fontsize=10, fontweight="bold", ha="center", va="center", family="sans-serif", zorder=5
    )

    # Crossbar Beam Visual Guide at Top
    beam_bar = FancyBboxPatch(
        (0.5, 10.25), 20.0, 0.36,
        boxstyle="round,pad=0.01,rounding_size=0.08",
        facecolor="#334155", edgecolor="#94A3B8", linewidth=1.5, linestyle="--", zorder=2
    )
    ax.add_patch(beam_bar)
    ax.text(
        10.5, 10.43,
        "RIGID ALUMINUM T-BAR CROSSBAR BEAM  (Master <-> Slave Baseline B = 55.5 cm | Pole Centerline at Midpoint B/2 = 27.75 cm | Height H = 1.80 m)",
        color="#E2E8F0", fontsize=8.8, fontweight="bold", ha="center", va="center", family="monospace", zorder=4
    )

    # 3 Antennas on the Crossbar Beam
    draw_antenna(ax, 2.1, 10.61, "ANT1: MASTER GNSS", "L1/L2/L5 Position (PVT)", color="#22C55E")
    draw_antenna(ax, 6.0, 10.61, "ANT2: SLAVE GNSS", "Moving-Baseline Heading", color="#10B981")
    draw_antenna(ax, 18.2, 10.61, "900 MHz LoRa ANTENNA", "CH 75 (925.125 MHz ISM)", color="#A855F7")

    # 1. Holybro UM982 (Top-Left)
    um_x, um_y, um_w, um_h = 0.5, 5.40, 5.4, 4.10
    draw_chip_box(
        ax, um_x, um_y, um_w, um_h,
        "HOLYBRO H-RTK UNICORE UM982",
        "Dual-Antenna Multi-Band RTK GNSS (Single-UART COM1)",
        facecolor="#1E293B", edgecolor="#22C55E"
    )
    draw_pin_top(ax, 2.1, um_y + um_h, "ANT1 (MASTER)", dot_color="#22C55E")
    draw_pin_top(ax, 4.3, um_y + um_h, "ANT2 (SLAVE)", dot_color="#10B981")
    draw_wire(ax, [(2.1, um_y + um_h), (2.1, 10.61)], color="#22C55E", lw=2.5, label="50 Ohm Coax", label_pos=(1.35, 9.85))
    draw_wire(ax, [(4.3, um_y + um_h), (4.3, 10.0), (6.0, 10.0), (6.0, 10.61)], color="#10B981", lw=2.5, label="50 Ohm Coax", label_pos=(5.25, 10.02))

    um_p_5v   = 8.30
    um_p_gnd  = 7.75
    um_p_txd0 = 7.10
    um_p_rxd0 = 6.45
    um_p_com2 = 5.80
    draw_pin_right(ax, um_x + um_w, um_p_5v,   "5V_IN (VCC 5.0V)", dot_color="#EF4444")
    draw_pin_right(ax, um_x + um_w, um_p_gnd,  "GND", dot_color="#64748B")
    draw_pin_right(ax, um_x + um_w, um_p_txd0, "COM1 TX (TXD0 Out)", dot_color="#22C55E")
    draw_pin_right(ax, um_x + um_w, um_p_rxd0, "COM1 RX (RXD0 In)", dot_color="#38BDF8")
    draw_pin_right(ax, um_x + um_w, um_p_com2, "COM2 TXD1/RXD1 (NC)", pin_color="#64748B", dot_color="#475569")

    ax.text(
        um_x + 0.18, 6.20,
        "v0.8.0 Single-UART COM1\n"
        "@ 115200 8N1:\n"
        "• TXD0 -> IO17:\n"
        "  NMEA + UNIHEADINGA\n"
        "  + BASEINFOA + Cmd ACK\n"
        "• RXD0 <- IO18:\n"
        "  Setup/Query Cmds\n"
        "  + Verified RTCM3",
        color="#CBD5E1", fontsize=7.4, ha="left", va="bottom", family="monospace",
        bbox=dict(boxstyle="round,pad=0.22", facecolor="#0F172A", edgecolor="#334155")
    )

    # 2. Waveshare ESP32-S3-Touch-LCD-3.5B (Center — Widened so left & right pins have ample room)
    esp_x, esp_y, esp_w, esp_h = 7.35, 2.25, 6.3, 7.25
    draw_chip_box(
        ax, esp_x, esp_y, esp_w, esp_h,
        "WAVESHARE ESP32-S3-TOUCH-LCD-3.5B",
        "ESP32-S3R8 (16MB Flash / 8MB OPI PSRAM) + 320x480 IPS",
        facecolor="#1E293B", edgecolor="#38BDF8"
    )

    # ESP32 Left-side pins (facing UM982 & MMC5983MA)
    draw_pin_left(ax, esp_x, um_p_5v,   "5V / VBUS IN", dot_color="#EF4444")
    draw_pin_left(ax, esp_x, um_p_gnd,  "GND", dot_color="#64748B")
    draw_pin_left(ax, esp_x, um_p_txd0, "GPIO 17 (Serial1 RX)", dot_color="#22C55E")
    draw_pin_left(ax, esp_x, um_p_rxd0, "GPIO 18 (Serial1 TX)", dot_color="#38BDF8")
    draw_pin_left(ax, esp_x, um_p_com2, "GPIO 47/48 (Freed/NC)", pin_color="#64748B", dot_color="#475569")

    mag_p_3v3 = 4.35
    mag_p_gnd = 3.88
    mag_p_scl = 3.41
    mag_p_sda = 2.94
    draw_pin_left(ax, esp_x, mag_p_3v3, "3V3 OUT (+3.3V)", dot_color="#F97316")
    draw_pin_left(ax, esp_x, mag_p_gnd, "GND", dot_color="#64748B")
    draw_pin_left(ax, esp_x, mag_p_scl, "GPIO 7 (I2C0_SCL)", dot_color="#FACC15")
    draw_pin_left(ax, esp_x, mag_p_sda, "GPIO 8 (I2C0_SDA)", dot_color="#38BDF8")

    # ESP32 Right-side pins (facing Ebyte E220-900T30D/S)
    e220_p_5v  = 8.30
    e220_p_gnd = 7.75
    e220_p_rx  = 7.10
    e220_p_tx  = 6.45
    e220_p_aux = 5.80
    e220_p_m0  = 5.15
    e220_p_m1  = 4.50
    e220_p_rst = 3.85
    draw_pin_right(ax, esp_x + esp_w, e220_p_5v,  "5V RAIL", dot_color="#EF4444")
    draw_pin_right(ax, esp_x + esp_w, e220_p_gnd, "GND", dot_color="#64748B")
    draw_pin_right(ax, esp_x + esp_w, e220_p_rx,  "GPIO 41 (Serial0 RX)", dot_color="#A855F7")
    draw_pin_right(ax, esp_x + esp_w, e220_p_tx,  "GPIO 42 (Serial0 TX)", dot_color="#EC4899")
    draw_pin_right(ax, esp_x + esp_w, e220_p_aux, "GPIO 21 (E220_AUX)", dot_color="#F59E0B")
    draw_pin_right(ax, esp_x + esp_w, e220_p_m0,  "GPIO 39 (E220_M0)", dot_color="#14B8A6")
    draw_pin_right(ax, esp_x + esp_w, e220_p_m1,  "GPIO 40 (E220_M1)", dot_color="#14B8A6")
    draw_pin_right(ax, esp_x + esp_w, e220_p_rst, "GPIO 38 (E220_RST)", pin_color="#94A3B8", dot_color="#64748B")

    # Internal I2C0 & Peripherals summary box placed cleanly at bottom of ESP32 box (below all pins!)
    ax.text(
        esp_x + esp_w / 2.0, esp_y + 0.48,
        "ONBOARD INTERNAL BUSES:  QSPI LCD (IO1..5, IO12, BL=IO6)  |  MicroSD SPI (IO9..11)\n"
        "Shared I2C0 (IO7/IO8 @ 400kHz):  0x18 ES8311 | 0x20 TCA9554 | 0x34 AXP2101 | 0x3B Touch | 0x51 RTC | 0x6B QMI8658 IMU",
        color="#CBD5E1", fontsize=7.1, ha="center", va="center", family="monospace",
        bbox=dict(boxstyle="round,pad=0.22", facecolor="#0F172A", edgecolor="#38BDF8", linewidth=1.0),
        zorder=5
    )

    # 3. Ebyte E220-900T30D / E220-900T30S (Top-Right)
    lora_x, lora_y, lora_w, lora_h = 15.1, 3.35, 5.4, 6.15
    draw_chip_box(
        ax, lora_x, lora_y, lora_w, lora_h,
        "EBYTE E220-900T30D / T30S",
        "900 MHz 1W (30 dBm) LoRa Transceiver (LLCC68)",
        facecolor="#1E293B", edgecolor="#A855F7"
    )
    draw_pin_top(ax, 18.2, lora_y + lora_h, "SMA / IPEX RF ANT", dot_color="#A855F7")
    draw_wire(ax, [(18.2, lora_y + lora_h), (18.2, 10.61)], color="#A855F7", lw=2.5, label="50 Ohm Coax", label_pos=(17.35, 9.85))

    draw_pin_left(ax, lora_x, e220_p_5v,  "VCC (5.0V + 100uF/0.1uF)", dot_color="#EF4444")
    draw_pin_left(ax, lora_x, e220_p_gnd, "GND", dot_color="#64748B")
    draw_pin_left(ax, lora_x, e220_p_rx,  "TXD (Module UART Out)", dot_color="#A855F7")
    draw_pin_left(ax, lora_x, e220_p_tx,  "RXD (Module UART In)", dot_color="#EC4899")
    draw_pin_left(ax, lora_x, e220_p_aux, "AUX (Busy=LOW, Idle=HIGH)", dot_color="#F59E0B")
    draw_pin_left(ax, lora_x, e220_p_m0,  "M0 (0=Transp, 1=Cfg)", dot_color="#14B8A6")
    draw_pin_left(ax, lora_x, e220_p_m1,  "M1 (0=Transp, 1=Cfg)", dot_color="#14B8A6")
    draw_pin_left(ax, lora_x, e220_p_rst, "NRST (Optional Reset)", pin_color="#94A3B8", dot_color="#64748B")

    # 4. SparkFun Qwiic MMC5983MA Compass (Bottom-Left)
    mag_x, mag_y, mag_w, mag_h = 0.5, 2.25, 5.4, 3.00
    draw_chip_box(
        ax, mag_x, mag_y, mag_w, mag_h,
        "SPARKFUN QWIIC MMC5983MA",
        "3-Axis 18-Bit AMR Magnetometer (I2C Addr: 0x30)",
        facecolor="#1E293B", edgecolor="#14B8A6"
    )
    draw_pin_right(ax, mag_x + mag_w, mag_p_3v3, "3.3V (Qwiic Red)", dot_color="#F97316")
    draw_pin_right(ax, mag_x + mag_w, mag_p_gnd, "GND (Qwiic Black)", dot_color="#64748B")
    draw_pin_right(ax, mag_x + mag_w, mag_p_scl, "SCL (Qwiic Yellow)", dot_color="#FACC15")
    draw_pin_right(ax, mag_x + mag_w, mag_p_sda, "SDA (Qwiic Blue)", dot_color="#38BDF8")
    ax.text(
        mag_x + 0.18, mag_y + 0.16,
        "Mounted 6\"-10\" outboard on beam.\n"
        "Auto-detected (Reg 0x2F == 0x30).\n"
        "SET/RESET coil + 9-DOF HDG CMP.",
        color="#CBD5E1", fontsize=7.3, ha="left", va="bottom", family="sans-serif"
    )

    # 5. Dual-Power Hot-Swap UPS Stage (Bottom Full-Width)
    pwr_x, pwr_y, pwr_w, pwr_h = 0.5, 0.35, 20.0, 1.65
    draw_chip_box(
        ax, pwr_x, pwr_y, pwr_w, pwr_h,
        "PHASE 6 DUAL-POWER HOT-SWAP UPS ARCHITECTURE",
        "External 5.0V USB Pack (UM982 + E220 + VBUS)  +  Internal 1S 3.7V LiPo Buffer via AXP2101 PMIC (500 mA CC Charge)",
        facecolor="#1E293B", edgecolor="#EF4444"
    )
    ax.text(
        pwr_x + pwr_w / 2.0, pwr_y + 0.46,
        "• External 5V USB Power Bank feeds 5V Rail (UM982 + E220) and ESP32 VBUS (monitored via AXP2101 14-bit ADC >= 4.0V).\n"
        "• Internal 1S LiPo (JST-PH 2.0) keeps ESP32-S3 & LCD alive across 5V battery swaps with zero rebooting (Auto-Shutdown timer when idle on Bat).",
        color="#E2E8F0", fontsize=8.0, ha="center", va="center", family="sans-serif", zorder=5
    )

    # Draw Wires: UM982 <-> ESP32-S3
    draw_wire(ax, [(um_x + um_w, um_p_5v), (esp_x, um_p_5v)], color="#EF4444", lw=2.2, label="5.0V", label_pos=(6.62, um_p_5v + 0.05))
    draw_wire(ax, [(um_x + um_w, um_p_gnd), (esp_x, um_p_gnd)], color="#64748B", lw=2.0, label="GND", label_pos=(6.62, um_p_gnd + 0.05))
    draw_wire(ax, [(um_x + um_w, um_p_txd0), (esp_x, um_p_txd0)], color="#22C55E", lw=2.3, label="NMEA/Hdg/ACK", label_pos=(6.62, um_p_txd0 + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x, um_p_rxd0), (um_x + um_w, um_p_rxd0)], color="#38BDF8", lw=2.3, label="Cmds + RTCM3", label_pos=(6.62, um_p_rxd0 + 0.05), arrow_end=True)

    # Draw Wires: MMC5983MA <-> ESP32-S3
    draw_wire(ax, [(esp_x, mag_p_3v3), (mag_x + mag_w, mag_p_3v3)], color="#F97316", lw=2.0, label="3.3V (Red)", label_pos=(6.62, mag_p_3v3 + 0.05), arrow_end=True)
    draw_wire(ax, [(mag_x + mag_w, mag_p_gnd), (esp_x, mag_p_gnd)], color="#64748B", lw=2.0, label="GND (Blk)", label_pos=(6.62, mag_p_gnd + 0.05))
    draw_wire(ax, [(esp_x, mag_p_scl), (mag_x + mag_w, mag_p_scl)], color="#FACC15", lw=2.0, label="SCL (Yel)", label_pos=(6.62, mag_p_scl + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x, mag_p_sda), (mag_x + mag_w, mag_p_sda)], color="#38BDF8", lw=2.0, label="SDA (Blu)", label_pos=(6.62, mag_p_sda + 0.05), arrow_end=True, arrow_start=True)

    # Draw Wires: ESP32-S3 <-> Ebyte E220
    draw_wire(ax, [(esp_x + esp_w, e220_p_5v), (lora_x, e220_p_5v)], color="#EF4444", lw=2.2, label="5.0V", label_pos=(14.38, e220_p_5v + 0.05))
    draw_wire(ax, [(esp_x + esp_w, e220_p_gnd), (lora_x, e220_p_gnd)], color="#64748B", lw=2.0, label="GND", label_pos=(14.38, e220_p_gnd + 0.05))
    draw_wire(ax, [(lora_x, e220_p_rx), (esp_x + esp_w, e220_p_rx)], color="#A855F7", lw=2.3, label="RTCM3 RX", label_pos=(14.38, e220_p_rx + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_tx), (lora_x, e220_p_tx)], color="#EC4899", lw=2.3, label="Telem TX", label_pos=(14.38, e220_p_tx + 0.05), arrow_end=True)
    draw_wire(ax, [(lora_x, e220_p_aux), (esp_x + esp_w, e220_p_aux)], color="#F59E0B", lw=2.0, label="AUX ISR", label_pos=(14.38, e220_p_aux + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_m0), (lora_x, e220_p_m0)], color="#14B8A6", lw=1.9, label="M0", label_pos=(14.38, e220_p_m0 + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_m1), (lora_x, e220_p_m1)], color="#14B8A6", lw=1.9, label="M1", label_pos=(14.38, e220_p_m1 + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_rst), (lora_x, e220_p_rst)], color="#64748B", lw=1.5, ls="--", label="RST (Opt)", label_pos=(14.38, e220_p_rst + 0.05), arrow_end=True)

    plt.tight_layout(pad=0.3)
    svg_path = os.path.join(out_dir, "wiring_diagram_3_5B_v0_8_0.svg")
    png_path = os.path.join(out_dir, "wiring_diagram_3_5B_v0_8_0.png")
    fig.savefig(svg_path, format="svg", facecolor=fig.get_facecolor())
    fig.savefig(png_path, format="png", dpi=200, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"[+] Saved: {svg_path}")
    print(f"[+] Saved: {png_path}")


def generate_3_49_v2_diagram(out_dir: str):
    fig, ax = plt.subplots(figsize=(21, 14.5), dpi=200)
    fig.patch.set_facecolor("#0B1120")
    ax.set_facecolor("#0B1120")
    ax.set_xlim(0, 21)
    ax.set_ylim(0, 14.5)
    ax.axis("off")

    # Title Banner
    title_box = FancyBboxPatch(
        (0.4, 13.30), 20.2, 0.95,
        boxstyle="round,pad=0.02,rounding_size=0.15",
        facecolor="#1E293B", edgecolor="#10B981", linewidth=2.2, zorder=3
    )
    ax.add_patch(title_box)
    ax.text(
        10.5, 13.94,
        "NEXT-GENERATION SMART POLE - HARDWARE & POWER WIRING SCHEMATIC",
        color="#F8FAFC", fontsize=14, fontweight="bold", ha="center", va="center", family="monospace", zorder=5
    )
    ax.text(
        10.5, 13.54,
        "Waveshare ESP32-S3-Touch-LCD-3.49-V2 (J3 22-Pin Header) + NDP6020P/MT3608 5V Power Stage + Holybro UM982 (Single-UART COM1) + Ebyte E220 + MMC5983MA",
        color="#10B981", fontsize=9.8, fontweight="bold", ha="center", va="center", family="sans-serif", zorder=5
    )

    # 1" Square Aluminum Crossbar Beam Banner
    beam_bar = FancyBboxPatch(
        (0.5, 10.75), 20.0, 0.36,
        boxstyle="round,pad=0.01,rounding_size=0.08",
        facecolor="#334155", edgecolor="#94A3B8", linewidth=1.5, linestyle="--", zorder=2
    )
    ax.add_patch(beam_bar)
    ax.text(
        10.5, 10.93,
        "1\" SQUARE ALUMINUM TUBE REMOVABLE CROSSBAR BEAM  (Master <-> Slave Baseline B = 610..762 mm | Cable-Free Quick-Release Pole Mount)",
        color="#E2E8F0", fontsize=8.8, fontweight="bold", ha="center", va="center", family="monospace", zorder=4
    )

    # 3 Antennas on the 1" Square Aluminum Beam
    draw_antenna(ax, 2.1, 11.11, "ANT1: MASTER GNSS", "Helical L1/L2/L5 Position", color="#22C55E")
    draw_antenna(ax, 6.0, 11.11, "ANT2: SLAVE GNSS", "Helical Moving-Baseline Hdg", color="#10B981")
    draw_antenna(ax, 18.2, 11.11, "900 MHz LoRa ANTENNA", "CH 75 (925.125 MHz ISM)", color="#A855F7")

    # 1. Holybro UM982 (Top-Left)
    um_x, um_y, um_w, um_h = 0.6, 6.40, 5.3, 3.85
    draw_chip_box(
        ax, um_x, um_y, um_w, um_h,
        "HOLYBRO H-RTK UNICORE UM982",
        "Dual-Antenna RTK GNSS (Single-UART COM1 Consolidated)",
        facecolor="#1E293B", edgecolor="#22C55E"
    )
    draw_pin_top(ax, 2.1, um_y + um_h, "ANT1 (MASTER)", dot_color="#22C55E")
    draw_pin_top(ax, 4.3, um_y + um_h, "ANT2 (SLAVE)", dot_color="#10B981")
    draw_wire(ax, [(2.1, um_y + um_h), (2.1, 11.11)], color="#22C55E", lw=2.5, label="50 Ohm Coax", label_pos=(1.35, 10.42))
    draw_wire(ax, [(4.3, um_y + um_h), (4.3, 10.5), (6.0, 10.5), (6.0, 11.11)], color="#10B981", lw=2.5, label="50 Ohm Coax", label_pos=(5.25, 10.52))

    # Put 5V_IN on left side of UM982 so the +5.00V rail routes cleanly up the outer-left margin!
    um_p_5v   = 8.35
    um_p_gnd  = 8.35
    um_p_txd0 = 7.75
    um_p_rxd0 = 7.15
    um_p_com2 = 6.60
    draw_pin_left(ax, um_x, um_p_5v, "5V_IN (Switched +5.0V)", dot_color="#EF4444")
    draw_pin_right(ax, um_x + um_w, um_p_gnd,  "GND", dot_color="#64748B")
    draw_pin_right(ax, um_x + um_w, um_p_txd0, "COM1 TX (TXD0 Out)", dot_color="#22C55E")
    draw_pin_right(ax, um_x + um_w, um_p_rxd0, "COM1 RX (RXD0 In)", dot_color="#38BDF8")
    draw_pin_right(ax, um_x + um_w, um_p_com2, "COM2 (NC / Freed)", pin_color="#64748B", dot_color="#475569")

    # 2. Waveshare ESP32-S3-Touch-LCD-3.49-V2 (Center — Widened so left & right pins never collide)
    esp_x, esp_y, esp_w, esp_h = 7.35, 2.85, 6.3, 7.40
    draw_chip_box(
        ax, esp_x, esp_y, esp_w, esp_h,
        "WAVESHARE ESP32-S3-TOUCH-LCD-3.49-V2",
        "640x172 IPS Bar HUD + Rear 18650 Holder + J3 22-Pin Header",
        facecolor="#1E293B", edgecolor="#10B981"
    )

    # J3 Left-side pins on 3.49-V2
    draw_pin_left(ax, esp_x, um_p_gnd,  "J3-Pin 21: GND", dot_color="#64748B")
    draw_pin_left(ax, esp_x, um_p_txd0, "J3-Pin 5: GPIO 15 (RX1)", dot_color="#22C55E")
    draw_pin_left(ax, esp_x, um_p_rxd0, "J3-Pin 6: GPIO 16 (TX1)", dot_color="#38BDF8")
    draw_pin_left(ax, esp_x, um_p_com2, "J3-Pin 9: GPIO 6 (Freed)", pin_color="#64748B", dot_color="#475569")

    # J3 Lower-Left pins for MMC5983MA Compass
    mag_p_3v3 = 5.10
    mag_p_gnd = 4.62
    mag_p_scl = 4.14
    mag_p_sda = 3.66
    mag_p_int = 3.22
    draw_pin_left(ax, esp_x, mag_p_3v3, "J3-Pin 2: 3V3 (Switched)", dot_color="#F97316")
    draw_pin_left(ax, esp_x, mag_p_gnd, "J3-Pin 3: GND", dot_color="#64748B")
    draw_pin_left(ax, esp_x, mag_p_scl, "J3-Pin 17: GPIO 18 (SCL)", dot_color="#FACC15")
    draw_pin_left(ax, esp_x, mag_p_sda, "J3-Pin 18: GPIO 17 (SDA)", dot_color="#38BDF8")
    draw_pin_left(ax, esp_x, mag_p_int, "J3-Pin 11: GPIO 47 (INT)", pin_color="#94A3B8", dot_color="#64748B")

    # J3 Right-side pins for Ebyte E220
    e220_p_tx  = 7.95
    e220_p_rx  = 7.30
    e220_p_aux = 6.65
    e220_p_m0  = 6.00
    e220_p_m1  = 5.35
    e220_p_gnd = 4.70
    draw_pin_right(ax, esp_x + esp_w, e220_p_tx,  "J3-Pin 7: GPIO 43 (TXD0)", dot_color="#EC4899")
    draw_pin_right(ax, esp_x + esp_w, e220_p_rx,  "J3-Pin 8: GPIO 44 (RXD0)", dot_color="#A855F7")
    draw_pin_right(ax, esp_x + esp_w, e220_p_aux, "J3-Pin 10: GPIO 48 (AUX)", dot_color="#F59E0B")
    draw_pin_right(ax, esp_x + esp_w, e220_p_m0,  "J3-Pin 15: GPIO 2 (M0)", dot_color="#14B8A6")
    draw_pin_right(ax, esp_x + esp_w, e220_p_m1,  "J3-Pin 16: GPIO 1 (M1)", dot_color="#14B8A6")
    draw_pin_right(ax, esp_x + esp_w, e220_p_gnd, "J3-Pin 22: GND", dot_color="#64748B")

    # Internal subsystems summary box placed cleanly below header bar and above J3-Pin 21
    ax.text(
        esp_x + esp_w / 2.0, esp_y + 6.18,
        "ONBOARD: 640x172 QSPI IPS + Touch (0x3B) | 18650 Holder + ETA6098 USB-C\n"
        "VBAT Sense: IO4 (3.0x) | SD_MMC (IO39..41) | BLE 5.0 NUS <-> Android\n"
        "I2C0 (IO17/18 @ 400k): 0x6B QMI8658A | 0x51 PCF85063A | 0x20 TCA9554",
        color="#CBD5E1", fontsize=6.9, ha="center", va="center", family="monospace",
        bbox=dict(boxstyle="round,pad=0.18", facecolor="#0F172A", edgecolor="#10B981", linewidth=1.0),
        zorder=5
    )

    # J3 Bottom pins for High-Side Power Switch (spaced wide so labels never touch)
    draw_pin_bottom(ax, 8.5,  esp_y, "J3-Pin 1: VBAT", dot_color="#EF4444")
    draw_pin_bottom(ax, 10.5, esp_y, "J3-Pin 2: 3V3 (Gate En)", dot_color="#F97316")
    draw_pin_bottom(ax, 12.5, esp_y, "J3-Pin 3: GND", dot_color="#64748B")

    # 3. Ebyte E220-900T30D / E220-900T30S (Top-Right)
    lora_x, lora_y, lora_w, lora_h = 15.1, 4.10, 5.3, 6.15
    draw_chip_box(
        ax, lora_x, lora_y, lora_w, lora_h,
        "EBYTE E220-900T30D / T30S",
        "900 MHz 1W LoRa Transceiver (RTCM Downlink + Telem Uplink)",
        facecolor="#1E293B", edgecolor="#A855F7"
    )
    draw_pin_top(ax, 18.2, lora_y + lora_h, "SMA / IPEX RF ANT", dot_color="#A855F7")
    draw_wire(ax, [(18.2, lora_y + lora_h), (18.2, 11.11)], color="#A855F7", lw=2.5, label="50 Ohm Coax", label_pos=(17.35, 10.42))

    e220_p_5v = 8.35
    draw_pin_right(ax, lora_x + lora_w, e220_p_5v, "VCC (+5.0V + 100uF)", dot_color="#EF4444")
    draw_pin_left(ax, lora_x, e220_p_tx,  "RXD (Module UART In)", dot_color="#EC4899")
    draw_pin_left(ax, lora_x, e220_p_rx,  "TXD (Module UART Out)", dot_color="#A855F7")
    draw_pin_left(ax, lora_x, e220_p_aux, "AUX (Busy=LOW, Idle=HIGH)", dot_color="#F59E0B")
    draw_pin_left(ax, lora_x, e220_p_m0,  "M0 (Mode Bit 0)", dot_color="#14B8A6")
    draw_pin_left(ax, lora_x, e220_p_m1,  "M1 (Mode Bit 1)", dot_color="#14B8A6")
    draw_pin_left(ax, lora_x, e220_p_gnd, "GND", dot_color="#64748B")

    # 4. SparkFun Qwiic MMC5983MA Compass (Mid-Left)
    mag_x, mag_y, mag_w, mag_h = 0.6, 2.85, 5.3, 3.25
    draw_chip_box(
        ax, mag_x, mag_y, mag_w, mag_h,
        "SPARKFUN QWIIC MMC5983MA",
        "Outboard 3-Axis 18-Bit AMR Compass (0x30 on Beam)",
        facecolor="#1E293B", edgecolor="#14B8A6"
    )
    draw_pin_right(ax, mag_x + mag_w, mag_p_3v3, "3.3V (Qwiic Red)", dot_color="#F97316")
    draw_pin_right(ax, mag_x + mag_w, mag_p_gnd, "GND (Qwiic Black)", dot_color="#64748B")
    draw_pin_right(ax, mag_x + mag_w, mag_p_scl, "SCL (Qwiic Yellow)", dot_color="#FACC15")
    draw_pin_right(ax, mag_x + mag_w, mag_p_sda, "SDA (Qwiic Blue)", dot_color="#38BDF8")
    draw_pin_right(ax, mag_x + mag_w, mag_p_int, "INT (Data-Ready Opt)", pin_color="#94A3B8", dot_color="#64748B")

    # 5. Discrete High-Side Power Switch + MT3608 5.0V Boost Stage (Bottom Full-Width)
    pwr_x, pwr_y, pwr_w, pwr_h = 0.6, 0.35, 19.8, 2.10
    draw_chip_box(
        ax, pwr_x, pwr_y, pwr_w, pwr_h,
        "DISCRETE HIGH-SIDE SWITCH (NDP6020P P-FET + 2N3904 NPN) & MT3608 +5.00V BOOST CONVERTER STAGE",
        "Zero Quiescent Drain (0.00 uA) When 3.49-V2 Power Button Is OFF  |  Powers UM982 (5V) + E220 (5V) From Single 18650 Cell",
        facecolor="#1E293B", edgecolor="#EF4444"
    )
    ax.text(
        pwr_x + pwr_w / 2.0, pwr_y + 0.68,
        "J3-Pin 1 (VBAT 3.5-4.2V) ──┬──► NDP6020P Source (Pin 3) & [R1: 10k Pull-Up] ──► NDP6020P Gate (Pin 1)  ==>  NDP6020P Drain (Pin 2) ──► [MT3608 VIN+]\n"
        "J3-Pin 2 (3V3 Switched)  ──┴──► [R2: 10k] ──► 2N3904 Base (Pin 2) [Emitter Pin 1 -> GND, Collector Pin 3 -> NDP6020P Gate]  ==>  [MT3608 VOUT+ (+5.00V + 100uF Cap)] ──► UM982 5V & E220 VCC",
        color="#F8FAFC", fontsize=8.0, fontweight="bold", ha="center", va="center", family="monospace",
        bbox=dict(boxstyle="round,pad=0.22", facecolor="#0F172A", edgecolor="#EF4444", linewidth=1.2),
        zorder=5
    )

    # Wires from J3 Bottom pins into Power Stage
    draw_wire(ax, [(8.5, esp_y), (8.5, pwr_y + pwr_h)], color="#EF4444", lw=2.3, arrow_end=True)
    draw_wire(ax, [(10.5, esp_y), (10.5, pwr_y + pwr_h)], color="#F97316", lw=2.1, arrow_end=True)
    draw_wire(ax, [(12.5, esp_y), (12.5, pwr_y + pwr_h)], color="#64748B", lw=2.0, arrow_end=True)

    # Switched +5.00V Rail routed cleanly along outer left & right margins (zero crossing of signal wires!)
    draw_wire(
        ax,
        [(pwr_x, 1.40), (0.22, 1.40), (0.22, um_p_5v), (um_x, um_p_5v)],
        color="#EF4444", lw=2.5, label="+5.00V", label_pos=(0.35, 6.15), arrow_end=True
    )
    draw_wire(
        ax,
        [(pwr_x + pwr_w, 1.40), (20.78, 1.40), (20.78, e220_p_5v), (lora_x + lora_w, e220_p_5v)],
        color="#EF4444", lw=2.5, label="+5.00V", label_pos=(20.62, 3.85), arrow_end=True
    )

    # Wires: UM982 <-> 3.49-V2 (J3)
    draw_wire(ax, [(um_x + um_w, um_p_gnd), (esp_x, um_p_gnd)], color="#64748B", lw=2.0, label="GND", label_pos=(6.62, um_p_gnd + 0.05))
    draw_wire(ax, [(um_x + um_w, um_p_txd0), (esp_x, um_p_txd0)], color="#22C55E", lw=2.3, label="NMEA/Hdg/ACK", label_pos=(6.62, um_p_txd0 + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x, um_p_rxd0), (um_x + um_w, um_p_rxd0)], color="#38BDF8", lw=2.3, label="Cmds + RTCM3", label_pos=(6.62, um_p_rxd0 + 0.05), arrow_end=True)

    # Wires: MMC5983MA <-> 3.49-V2 (J3)
    draw_wire(ax, [(esp_x, mag_p_3v3), (mag_x + mag_w, mag_p_3v3)], color="#F97316", lw=2.0, label="3.3V (Red)", label_pos=(6.62, mag_p_3v3 + 0.05), arrow_end=True)
    draw_wire(ax, [(mag_x + mag_w, mag_p_gnd), (esp_x, mag_p_gnd)], color="#64748B", lw=2.0, label="GND (Blk)", label_pos=(6.62, mag_p_gnd + 0.05))
    draw_wire(ax, [(esp_x, mag_p_scl), (mag_x + mag_w, mag_p_scl)], color="#FACC15", lw=2.0, label="SCL (400k)", label_pos=(6.62, mag_p_scl + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x, mag_p_sda), (mag_x + mag_w, mag_p_sda)], color="#38BDF8", lw=2.0, label="SDA (400k)", label_pos=(6.62, mag_p_sda + 0.05), arrow_end=True, arrow_start=True)
    draw_wire(ax, [(mag_x + mag_w, mag_p_int), (esp_x, mag_p_int)], color="#64748B", lw=1.5, ls="--", label="DRDY (Opt)", label_pos=(6.62, mag_p_int + 0.05), arrow_end=True)

    # Wires: 3.49-V2 (J3) <-> Ebyte E220
    draw_wire(ax, [(esp_x + esp_w, e220_p_tx), (lora_x, e220_p_tx)], color="#EC4899", lw=2.3, label="Telem TX", label_pos=(14.38, e220_p_tx + 0.05), arrow_end=True)
    draw_wire(ax, [(lora_x, e220_p_rx), (esp_x + esp_w, e220_p_rx)], color="#A855F7", lw=2.3, label="RTCM3 RX", label_pos=(14.38, e220_p_rx + 0.05), arrow_end=True)
    draw_wire(ax, [(lora_x, e220_p_aux), (esp_x + esp_w, e220_p_aux)], color="#F59E0B", lw=2.0, label="AUX ISR", label_pos=(14.38, e220_p_aux + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_m0), (lora_x, e220_p_m0)], color="#14B8A6", lw=1.9, label="M0", label_pos=(14.38, e220_p_m0 + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_m1), (lora_x, e220_p_m1)], color="#14B8A6", lw=1.9, label="M1", label_pos=(14.38, e220_p_m1 + 0.05), arrow_end=True)
    draw_wire(ax, [(esp_x + esp_w, e220_p_gnd), (lora_x, e220_p_gnd)], color="#64748B", lw=2.0, label="GND", label_pos=(14.38, e220_p_gnd + 0.05))

    plt.tight_layout(pad=0.3)
    svg_path = os.path.join(out_dir, "wiring_diagram_3_49_V2_smart_pole.svg")
    png_path = os.path.join(out_dir, "wiring_diagram_3_49_V2_smart_pole.png")
    fig.savefig(svg_path, format="svg", facecolor=fig.get_facecolor())
    fig.savefig(png_path, format="png", dpi=200, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"[+] Saved: {svg_path}")
    print(f"[+] Saved: {png_path}")


def _rect_shadow(ax, x, y, w, h, r=0.14, dx=0.10, dy=-0.10):
    sh = FancyBboxPatch(
        (x + dx, y + dy), w, h,
        boxstyle=f"round,pad=0.0,rounding_size={r}",
        facecolor="#CBD5E1", edgecolor="none", alpha=0.45, zorder=1
    )
    ax.add_patch(sh)


def _pad_hole(ax, x, y, radius=0.085, ring="#D4AF37", hole="#F8FAFC", z=4):
    ax.add_patch(Circle((x, y), radius * 1.45, facecolor=ring, edgecolor="#78350F", linewidth=0.5, zorder=z))
    ax.add_patch(Circle((x, y), radius * 0.65, facecolor=hole, edgecolor="#475569", linewidth=0.4, zorder=z + 0.1))


def _junc_dot(ax, x, y, r=0.085, color="#000000", z=8):
    ax.add_patch(Circle((x, y), r, facecolor=color, edgecolor="#FFFFFF", linewidth=0.5, zorder=z))


def _ortho_wire(ax, pts, color="#000000", lw=3.0, ls="-", stripe_color=None, z=6):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, linewidth=lw, linestyle=ls, solid_capstyle="round", solid_joinstyle="round", zorder=z)
    if stripe_color:
        ax.plot(xs, ys, color=stripe_color, linewidth=max(1.2, lw * 0.52), linestyle=(0, (3.2, 3.2)),
                solid_capstyle="butt", solid_joinstyle="round", zorder=z + 0.1)


def _resistor_zigzag(ax, x0, y0, x1, y1, value_label, label_offset=(0.0, 0.25), horizontal=True, z=7):
    """Draws a crisp IEEE zigzag resistor symbol between two orthogonal points."""
    if horizontal:
        if x1 < x0:
            x0, x1 = x1, x0
        mx = 0.5 * (x0 + x1)
        span = min(1.05, max(0.60, (x1 - x0) * 0.58))
        zx0 = mx - span / 2.0
        zx1 = mx + span / 2.0
        ax.plot([x0, zx0], [y0, y0], color="#0F172A", lw=2.0, zorder=z)
        ax.plot([zx1, x1], [y0, y0], color="#0F172A", lw=2.0, zorder=z)
        n = 6
        pts_x = [zx0]
        pts_y = [y0]
        step = span / n
        for i in range(n):
            pts_x.append(zx0 + (i + 0.5) * step)
            pts_y.append(y0 + (0.18 if i % 2 == 0 else -0.18))
        pts_x.append(zx1)
        pts_y.append(y0)
        ax.plot(pts_x, pts_y, color="#0F172A", lw=2.0, solid_joinstyle="miter", zorder=z)
        ax.text(mx + label_offset[0], y0 + label_offset[1], value_label,
                color="#0F172A", fontsize=8.5, fontweight="bold", ha="center", va="center", zorder=z + 1)
    else:
        if y1 < y0:
            y0, y1 = y1, y0
        my = 0.5 * (y0 + y1)
        span = min(1.05, max(0.60, (y1 - y0) * 0.58))
        zy0 = my - span / 2.0
        zy1 = my + span / 2.0
        ax.plot([x0, x0], [y0, zy0], color="#0F172A", lw=2.0, zorder=z)
        ax.plot([x0, x0], [zy1, y1], color="#0F172A", lw=2.0, zorder=z)
        n = 6
        pts_x = [x0]
        pts_y = [zy0]
        step = span / n
        for i in range(n):
            pts_x.append(x0 + (0.18 if i % 2 == 0 else -0.18))
            pts_y.append(zy0 + (i + 0.5) * step)
        pts_x.append(x0)
        pts_y.append(zy1)
        ax.plot(pts_x, pts_y, color="#0F172A", lw=2.0, solid_joinstyle="miter", zorder=z)
        ax.text(x0 + label_offset[0], my + label_offset[1], value_label,
                color="#0F172A", fontsize=8.5, fontweight="bold", ha="left", va="center", zorder=z + 1)


def _draw_waveshare_35b_pcb(ax, x, y, w, h):
    """Draws realistic top-down Fritzing-style Waveshare ESP32-S3-Touch-LCD-3.5B PCB."""
    _rect_shadow(ax, x, y, w, h, r=0.22)
    # LCD bezel slightly protruding under PCB
    bezel = FancyBboxPatch(
        (x - 0.16, y - 0.16), w + 0.32, h + 0.32,
        boxstyle="round,pad=0.0,rounding_size=0.26",
        facecolor="#1E293B", edgecolor="#0F172A", linewidth=1.2, zorder=2
    )
    ax.add_patch(bezel)
    # Royal blue PCB substrate
    pcb = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.16",
        facecolor="#164E96", edgecolor="#0C356A", linewidth=1.5, zorder=2.5
    )
    ax.add_patch(pcb)
    # M2.5 mounting holes
    for mx, my in [(x + 0.32, y + h - 0.32), (x + w - 0.32, y + h - 0.32),
                   (x + 0.32, y + 0.32), (x + w - 0.32, y + 0.32)]:
        _pad_hole(ax, mx, my, radius=0.10, z=3)

    # Top edge: USB-C metal shroud
    usbc = FancyBboxPatch(
        (x + 3.25, y + h - 0.52), 1.05, 0.68,
        boxstyle="round,pad=0.0,rounding_size=0.08",
        facecolor="#CBD5E1", edgecolor="#475569", linewidth=1.2, zorder=3.5
    )
    ax.add_patch(usbc)
    ax.text(x + 3.77, y + h - 0.20, "USB-C", color="#1E293B", fontsize=6.2, fontweight="bold",
            ha="center", va="center", family="monospace", zorder=3.8)

    # Top-center: BAT JST-PH 2-pin white jack + AXP2101 + MicroSD in clean center bay (x+2.0 .. x+4.8)
    bat_jst = Rectangle((x + 1.85, y + h - 0.68), 0.78, 0.50, facecolor="#F8FAFC", edgecolor="#64748B", lw=1.0, zorder=3.5)
    ax.add_patch(bat_jst)
    ax.text(x + 2.24, y + h - 0.88, "BAT JST", color="#E2E8F0", fontsize=5.5, fontweight="bold",
            ha="center", va="center", family="monospace", zorder=3.6)

    axp = Rectangle((x + 2.35, y + h - 2.05), 0.78, 0.78, facecolor="#1E293B", edgecolor="#94A3B8", lw=0.8, zorder=3)
    ax.add_patch(axp)
    ax.text(x + 2.74, y + h - 1.66, "AXP\n2101", color="#E2E8F0", fontsize=5.2,
            ha="center", va="center", family="monospace", zorder=3.2)

    sd_cage = Rectangle((x + 3.55, y + h - 2.35), 1.25, 1.55, facecolor="#E2E8F0", edgecolor="#475569", lw=1.1, zorder=3)
    ax.add_patch(sd_cage)
    ax.text(x + 4.17, y + h - 1.57, "uSD", color="#334155", fontsize=6.8, fontweight="bold",
            ha="center", va="center", family="monospace", zorder=3.2)

    # Display FPC amber ribbon (middle-center)
    fpc = Rectangle((x + 2.85, y + 4.00), 1.00, 1.80, facecolor="#D97706", edgecolor="#92400E", lw=0.8, alpha=0.88, zorder=3)
    ax.add_patch(fpc)

    # Center decorative 2x16 pin header silhouette (matching real board layout)
    cx_hdr = x + 1.95
    ax.add_patch(Rectangle((cx_hdr, y + 1.45), 0.62, 6.80, facecolor="#0F172A", edgecolor="#475569", lw=0.9, zorder=3))
    for rr in range(16):
        ryy = y + 7.95 - rr * 0.415
        ax.add_patch(Circle((cx_hdr + 0.19, ryy), 0.055, facecolor="#D4AF37", edgecolor="none", zorder=3.2))
        ax.add_patch(Circle((cx_hdr + 0.43, ryy), 0.055, facecolor="#D4AF37", edgecolor="none", zorder=3.2))

    # ESP32-S3 QFN chip (bottom-center-right, clear of edge pins)
    esp = Rectangle((x + 2.95, y + 1.35), 1.25, 1.25, facecolor="#1E293B", edgecolor="#94A3B8", lw=0.9, zorder=3)
    ax.add_patch(esp)
    ax.text(x + 3.57, y + 1.97, "ESP32\n-S3R8", color="#F8FAFC", fontsize=5.8, fontweight="bold",
            ha="center", va="center", family="monospace", zorder=3.2)

    # Board silkscreen branding
    ax.text(x + 3.65, y + 6.70, "Waveshare\n3.5B", color="#E2E8F0", fontsize=7.6, fontweight="bold",
            ha="center", va="center", family="sans-serif", zorder=3)

    # Edge-Breakout Terminal Pads (Left = Odd 1..31, Right = Even 2..32)
    # Labels sit INWARD on the PCB; wires exit OUTWARD from PCB edge -> ZERO OVERLAP!
    rows_y = [y + h - 2.15 - 0.50 * r for r in range(16)]
    odd_labels = [
        "1:BAT", "3:GND", "5:IO21", "7:IO38", "9:IO39", "11:IO40", "13:IO41", "15:IO42",
        "17:IO45", "19:IO46", "21:IO47", "23:IO48", "25:IO44", "27:IO43", "29:GND", "31:3V3"
    ]
    even_labels = [
        "2:5V", "4:GND", "6:IO19", "8:IO20", "10:IO11", "12:IO10", "14:IO9", "16:IO17",
        "18:IO18", "20:IO0", "22:RST", "24:PWR", "26:SCL7", "28:SDA8", "30:GND", "32:3V3"
    ]
    active_odd = {0, 2, 3, 4, 5, 6, 7, 14, 15}
    active_even = {7, 8, 12, 13, 14, 15}

    pin_coords = {}
    px_l = x + 0.16
    px_r = x + w - 0.16
    # Subtle header breakout rail shading along left/right edges
    ax.add_patch(Rectangle((x + 0.04, rows_y[-1] - 0.25), 1.48, rows_y[0] - rows_y[-1] + 0.50,
                           facecolor="#0F3A73", edgecolor="#60A5FA", lw=0.6, alpha=0.65, zorder=2.8))
    ax.add_patch(Rectangle((x + w - 1.52, rows_y[-1] - 0.25), 1.48, rows_y[0] - rows_y[-1] + 0.50,
                           facecolor="#0F3A73", edgecolor="#60A5FA", lw=0.6, alpha=0.65, zorder=2.8))

    for r, ry in enumerate(rows_y):
        _pad_hole(ax, px_l, ry, radius=0.078,
                  ring="#FACC15" if r in active_odd else "#64748B",
                  hole="#0F172A", z=4)
        _pad_hole(ax, px_r, ry, radius=0.078,
                  ring="#FACC15" if r in active_even else "#64748B",
                  hole="#0F172A", z=4)
        ax.text(x + 0.34, ry, odd_labels[r],
                color="#FFFFFF" if r in active_odd else "#93C5FD",
                fontsize=6.5, fontweight="bold" if r in active_odd else "normal",
                ha="left", va="center", family="monospace", zorder=4.5)
        ax.text(x + w - 0.34, ry, even_labels[r],
                color="#FFFFFF" if r in active_even else "#93C5FD",
                fontsize=6.5, fontweight="bold" if r in active_even else "normal",
                ha="right", va="center", family="monospace", zorder=4.5)
        pin_coords[2 * r + 1] = (x, ry)
        pin_coords[2 * r + 2] = (x + w, ry)

    pin_coords["BAT_JST+"] = (x + 2.06, y + h - 0.18)
    pin_coords["BAT_JST-"] = (x + 2.42, y + h - 0.18)
    pin_coords["USBC_TOP"] = (x + 3.77, y + h + 0.16)
    return pin_coords


def _draw_e220_module(ax, x, y, w, h):
    """Draws Ebyte E220-900T30S transceiver PCB with brushed aluminum EMI shield."""
    _rect_shadow(ax, x, y, w, h, r=0.14)
    pcb = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.12",
        facecolor="#1E293B", edgecolor="#0F172A", lw=1.4, zorder=2.5
    )
    ax.add_patch(pcb)
    # Brushed silver RF shield can (leaves clean left/top/right PCB border for VCC/GND & signal pads)
    can = FancyBboxPatch(
        (x + 0.85, y + 0.65), w - 1.78, h - 1.65,
        boxstyle="round,pad=0.0,rounding_size=0.08",
        facecolor="#CBD5E1", edgecolor="#64748B", lw=1.2, zorder=3
    )
    ax.add_patch(can)
    ax.text(x + 0.85 + (w - 1.78) / 2.0, y + 0.65 + (h - 1.65) / 2.0,
            "EBYTE\nE220-900T30S\n900MHz 1W\nCH75 (925.125M)",
            color="#1E293B", fontsize=7.0, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.5)
    # Gold coax connector at top
    ant_x = x + 1.90
    ax.add_patch(Rectangle((ant_x - 0.25, y + h - 0.35), 0.50, 0.50, facecolor="#D4AF37", edgecolor="#78350F", lw=1.0, zorder=3.5))
    ax.text(ant_x + 0.38, y + h - 0.32, "RF ANT", color="#FACC15", fontsize=6.4, fontweight="bold", ha="left", va="center", family="monospace", zorder=3.6)

    pins = {}
    # Dedicated Power Input Pair on left edge (from Power Module)
    vcc_y = y + h - 0.90
    gnd_pwr_y = y + h - 1.55
    _pad_hole(ax, x + 0.16, vcc_y, radius=0.08, z=4)
    ax.text(x + 0.34, vcc_y, "VCC", color="#FCA5A5", fontsize=6.6, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
    pins["VCC"] = (x, vcc_y)

    _pad_hole(ax, x + 0.16, gnd_pwr_y, radius=0.08, z=4)
    ax.text(x + 0.34, gnd_pwr_y, "GND", color="#CBD5E1", fontsize=6.6, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
    pins["GND_PWR"] = (x, gnd_pwr_y)

    # Right edge signal pins aligned 1:1 horizontally with 3.5B pins (20.85 down to 18.35 when y=17.0)
    pin_rows = [
        ("AUX",  y + 3.85),
        ("NRST", y + 3.35),
        ("M0",   y + 2.85),
        ("M1",   y + 2.35),
        ("TXD",  y + 1.85),
        ("RXD",  y + 1.35),
    ]
    for lbl, py in pin_rows:
        _pad_hole(ax, x + w - 0.18, py, radius=0.08, z=4)
        ax.text(x + w - 0.36, py, lbl, color="#F8FAFC", fontsize=6.6, fontweight="bold", ha="right", va="center", family="monospace", zorder=4)
        pins[lbl] = (x + w, py)
    pins["ANT"] = (ant_x, y + h + 0.15)
    return pins


def _draw_um982_module(ax, x, y, w, h):
    """Draws Holybro H-RTK Unicore UM982 receiver housing with dual coax jacks, COM1 JST-GH & 5V/GND pair."""
    _rect_shadow(ax, x, y, w, h, r=0.18)
    case = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.18",
        facecolor="#1E242B", edgecolor="#0F172A", lw=1.5, zorder=2.5
    )
    ax.add_patch(case)
    ax.add_patch(Rectangle((x + 0.25, y + h - 0.95), w - 0.50, 0.14, facecolor="#F97316", edgecolor="none", zorder=3))
    for sx, sy in [(x + 0.30, y + h - 0.30), (x + w - 0.30, y + h - 0.30), (x + 0.30, y + 0.30), (x + w - 0.30, y + 0.30)]:
        ax.add_patch(Circle((sx, sy), 0.09, facecolor="#94A3B8", edgecolor="#334155", lw=0.6, zorder=3))

    ax.text(x + w * 0.50, y + h * 0.62, "Holybro\nH-RTK UM982\nDual-Ant RTK",
            color="#F8FAFC", fontsize=8.8, fontweight="bold", ha="center", va="center", family="sans-serif", zorder=3.5)

    ant1_x = x + 1.00
    ant2_x = x + w - 1.00
    for axp, albl in [(ant1_x, "ANT1\nMASTER"), (ant2_x, "ANT2\nSLAVE")]:
        ax.add_patch(Rectangle((axp - 0.26, y + h - 0.28), 0.52, 0.45, facecolor="#D4AF37", edgecolor="#78350F", lw=1.0, zorder=3.5))
        ax.text(axp, y + h - 0.55, albl, color="#FDE68A", fontsize=5.8, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.5)

    pins = {"ANT1": (ant1_x, y + h + 0.17), "ANT2": (ant2_x, y + h + 0.17)}
    ax.add_patch(Rectangle((x - 0.04, y + 0.25), 0.30, 1.75, facecolor="#F8FAFC", edgecolor="#64748B", lw=0.9, zorder=3.5))
    left_pins = [
        ("TXD0 (COM1)", y + 0.95, "#5EEAD4"),
        ("RXD0 (COM1)", y + 0.45, "#7DD3FC"),
    ]
    for lbl, py, col in left_pins:
        _pad_hole(ax, x + 0.14, py, radius=0.075, z=4)
        ax.text(x + 0.36, py, lbl, color=col, fontsize=6.5, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
        pins[lbl.split()[0]] = (x, py)
    ax.text(x + 0.36, y + 1.55, "COM2 (NC / Freed)", color="#64748B", fontsize=5.8, style="italic", ha="left", va="center", family="monospace", zorder=4)

    # Dedicated 5V_IN / GND Power Input Pair on right edge (from Power Module)
    p5_y = y + 2.95
    pg_y = y + 2.25
    _pad_hole(ax, x + w - 0.18, p5_y, radius=0.085, z=4)
    ax.text(x + w - 0.36, p5_y, "5V_IN", color="#FCA5A5", fontsize=7.0, fontweight="bold", ha="right", va="center", family="monospace", zorder=4)
    pins["5V_IN"] = (x + w, p5_y)

    _pad_hole(ax, x + w - 0.18, pg_y, radius=0.085, z=4)
    ax.text(x + w - 0.36, pg_y, "GND", color="#CBD5E1", fontsize=7.0, fontweight="bold", ha="right", va="center", family="monospace", zorder=4)
    pins["GND"] = (x + w, pg_y)
    return pins


def _draw_sen0696_pcb(ax, x, y, w, h):
    """Draws DFRobot Gravity SEN0696 10DOF IMU PCB with 3 Bosch sensors, DIP switch, and PH2.0-4P."""
    _rect_shadow(ax, x, y, w, h, r=0.14)
    pcb = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.14",
        facecolor="#155E38", edgecolor="#064E3B", lw=1.4, zorder=2.5
    )
    ax.add_patch(pcb)
    for mx, my in [(x + 0.30, y + h - 0.30), (x + w - 0.30, y + h - 0.30), (x + 0.30, y + 0.30), (x + w - 0.30, y + 0.30)]:
        _pad_hole(ax, mx, my, radius=0.095, z=3)

    for cx, cy, clbl in [(x + 2.1, y + h - 1.10, "BMI323"), (x + 3.6, y + h - 1.10, "BMM350"), (x + 3.6, y + 1.00, "BMP581")]:
        ax.add_patch(Rectangle((cx, cy), 0.90, 0.72, facecolor="#1E293B", edgecolor="#94A3B8", lw=0.7, zorder=3))
        ax.text(cx + 0.45, cy + 0.36, clbl, color="#F8FAFC", fontsize=5.3, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.2)

    ax.add_patch(Rectangle((x + 2.1, y + 0.95), 0.95, 0.72, facecolor="#DC2626", edgecolor="#7F1D1D", lw=0.9, zorder=3))
    ax.add_patch(Rectangle((x + 2.25, y + 1.08), 0.24, 0.46, facecolor="#FFFFFF", edgecolor="#334155", lw=0.5, zorder=3.2))
    ax.add_patch(Rectangle((x + 2.65, y + 1.08), 0.24, 0.46, facecolor="#FFFFFF", edgecolor="#334155", lw=0.5, zorder=3.2))
    ax.text(x + 2.58, y + 0.65, "I2C:0x4A", color="#D1FAE5", fontsize=5.5, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.2)

    ax.text(x + w * 0.64, y + h - 0.32, "DFRobot Gravity 10DOF", color="#ECFDF5", fontsize=6.6, fontweight="bold", ha="center", va="center", zorder=3.2)

    ax.add_patch(Rectangle((x - 0.04, y + 0.30), 0.40, h - 0.60, facecolor="#F8FAFC", edgecolor="#64748B", lw=1.0, zorder=3.5))
    pins = {}
    g_rows = [
        ("SCL", "C/R", y + 3.45, "#93C5FD"),
        ("SDA", "D/T", y + 2.95, "#86EFAC"),
        ("GND", "-",   y + 2.45, "#CBD5E1"),
        ("VCC", "+",   y + 1.95, "#FCA5A5"),
    ]
    for key, lbl, py, col in g_rows:
        _pad_hole(ax, x + 0.18, py, radius=0.075, z=4)
        ax.text(x + 0.45, py, lbl, color=col, fontsize=7.0, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
        pins[key] = (x, py)
    return pins


def _draw_mt3608_module(ax, x, y, w, h, flip_lr=False):
    """Draws MT3608 DC-DC Step-Up Boost module PCB with inductor, SS34 diode, and trimpot."""
    _rect_shadow(ax, x, y, w, h, r=0.10)
    pcb = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.10",
        facecolor="#1D4ED8", edgecolor="#1E3A8A", lw=1.4, zorder=2.5
    )
    ax.add_patch(pcb)
    ax.add_patch(Rectangle((x + 1.10, y + h * 0.45), 1.05, 1.05, facecolor="#475569", edgecolor="#94A3B8", lw=1.0, zorder=3))
    ax.text(x + 1.62, y + h * 0.45 + 0.52, "220", color="#F8FAFC", fontsize=7.0, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.2)
    ax.add_patch(Rectangle((x + 2.45, y + h * 0.44), 0.72, 1.05, facecolor="#2563EB", edgecolor="#93C5FD", lw=0.9, zorder=3))
    ax.add_patch(Circle((x + 2.65, y + h * 0.44 + 0.82), 0.13, facecolor="#D4AF37", edgecolor="#78350F", lw=0.7, zorder=3.3))
    ax.add_patch(Rectangle((x + 1.60, y + 0.48), 0.90, 0.50, facecolor="#1E293B", edgecolor="#94A3B8", lw=0.7, zorder=3))
    ax.text(x + 2.05, y + 0.73, "SS34", color="#CBD5E1", fontsize=5.2, ha="center", va="center", family="monospace", zorder=3.2)

    ax.text(x + w * 0.50, y + h - 0.34, "MT3608 BOOST -> +5.00V", color="#EFF6FF", fontsize=6.6, fontweight="bold",
            ha="center", va="center", family="monospace", zorder=3.5)

    p_top = y + h - 0.90
    p_bot = y + 0.80
    x_in = x if flip_lr else x + w
    x_out = x + w if flip_lr else x
    px_in = x + 0.22 if flip_lr else x + w - 0.22
    px_out = x + w - 0.22 if flip_lr else x + 0.22
    ha_in = "left" if flip_lr else "right"
    ha_out = "right" if flip_lr else "left"

    for px, py, lbl, ha_dir in [
        (px_in,  p_top, "VIN+",  ha_in),
        (px_in,  p_bot, "VIN-",  ha_in),
        (px_out, p_top, "VOUT+", ha_out),
        (px_out, p_bot, "VOUT-", ha_out),
    ]:
        _pad_hole(ax, px, py, radius=0.09, z=4)
        off = 0.18 if ha_dir == "left" else -0.18
        ax.text(px + off, py, lbl, color="#FFFFFF", fontsize=6.4, fontweight="bold", ha=ha_dir, va="center", family="monospace", zorder=4)
    return {
        "VIN+":  (x_in, p_top),
        "VIN-":  (x_in, p_bot),
        "VOUT+": (x_out, p_top),
        "VOUT-": (x_out, p_bot),
    }


def _draw_1s_battery(ax, x, y, w, h, top_edge_term=False):
    """Draws 1S 3.7V Li-Ion/LiPo pouch battery pack with Kapton protection strip."""
    _rect_shadow(ax, x, y, w, h, r=0.18)
    cell = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.18",
        facecolor="#E2E8F0", edgecolor="#64748B", lw=1.4, zorder=2.5
    )
    ax.add_patch(cell)
    ax.add_patch(Rectangle((x + 0.12, y + h - 0.62), w - 0.24, 0.48, facecolor="#D97706", edgecolor="#92400E", lw=0.8, alpha=0.85, zorder=3))
    ax.text(x + w / 2.0, y + h * 0.38, "1S 3.7V\nLi-Ion / LiPo\n3Ah - 10Ah",
            color="#1E293B", fontsize=7.6, fontweight="bold", ha="center", va="center", family="sans-serif", zorder=3.5)
    if top_edge_term:
        p_y = y + h - 0.26
        n_y = y + h - 0.50
        ax.text(x + w - 0.16, p_y, "+", color="#991B1B", fontsize=8.0, fontweight="bold", ha="right", va="center", zorder=4)
        ax.text(x + w - 0.16, n_y, "-", color="#0F172A", fontsize=8.5, fontweight="bold", ha="right", va="center", zorder=4)
        return {"BAT+": (x + w, p_y), "BAT-": (x + w, n_y)}
    p_y = y + h - 0.30
    n_y = y + h - 0.58
    ax.text(x + w - 0.20, p_y, "+", color="#991B1B", fontsize=8.0, fontweight="bold", ha="right", va="center", zorder=4)
    ax.text(x + w - 0.20, n_y, "-", color="#0F172A", fontsize=8.5, fontweight="bold", ha="right", va="center", zorder=4)
    return {"BAT+": (x + w, p_y), "BAT-": (x + w, n_y)}


def _draw_power_module(ax, x, y, w, h):
    """Draws collapsed Switched +5.00V Boost Power Module for the high-level diagram."""
    _rect_shadow(ax, x, y, w, h, r=0.16)
    pcb = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.0,rounding_size=0.16",
        facecolor="#1E293B", edgecolor="#0F172A", lw=1.5, zorder=2.5
    )
    ax.add_patch(pcb)
    # Teal accent header bar inside module
    ax.add_patch(Rectangle((x + 0.18, y + h - 0.68), w - 0.36, 0.50, facecolor="#0F766E", edgecolor="#14B8A6", lw=0.8, zorder=3))
    ax.text(x + w / 2.0, y + h - 0.43, "SWITCHED +5.00V POWER MODULE", color="#F8FAFC",
            fontsize=7.3, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.5)

    # Internal concise sub-block graphic
    ax.text(x + w / 2.0, y + h * 0.52,
            "High-Side Switch:\n  NDP6020P + 2N2222 + 2x10K\n\nBoost & Filter:\n  MT3608 + 470uF + 0.1uF\n\n(See Detail: Fig. 1B)",
            color="#CBD5E1", fontsize=6.4, ha="center", va="center", family="monospace", linespacing=1.22, zorder=3.5)

    pins = {}
    # Left edge: Dedicated Output 2 -> E220
    e_5v_y = y + h - 1.05
    e_gnd_y = y + h - 1.75
    for key, py, lbl, col in [
        ("E220_5V",  e_5v_y,  "E220 +5V", "#FCA5A5"),
        ("E220_GND", e_gnd_y, "E220 GND", "#E2E8F0"),
    ]:
        _pad_hole(ax, x + 0.16, py, radius=0.08, z=4)
        ax.text(x + 0.34, py, lbl, color=col, fontsize=6.2, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
        pins[key] = (x, py)

    # Right edge top: Inputs from Waveshare 3.5B Header
    in_pins = [
        ("VBAT_IN", y + h - 1.05, "VBAT IN", "#FCA5A5"),
        ("GND_IN",  y + h - 1.75, "GND IN",  "#E2E8F0"),
        ("3V3_EN",  y + h - 2.45, "3V3 EN",  "#FDBA74"),
    ]
    for key, py, lbl, col in in_pins:
        _pad_hole(ax, x + w - 0.16, py, radius=0.08, z=4)
        ax.text(x + w - 0.34, py, lbl, color=col, fontsize=6.2, fontweight="bold", ha="right", va="center", family="monospace", zorder=4)
        pins[key] = (x + w, py)

    # Right edge bottom: Dedicated Output 1 -> Holybro UM982
    for key, py, lbl, col in [
        ("UM_5V",  y + 1.40, "UM982 +5V", "#FCA5A5"),
        ("UM_GND", y + 0.65, "UM982 GND", "#E2E8F0"),
    ]:
        _pad_hole(ax, x + w - 0.16, py, radius=0.08, z=4)
        ax.text(x + w - 0.34, py, lbl, color=col, fontsize=6.2, fontweight="bold", ha="right", va="center", family="monospace", zorder=4)
        pins[key] = (x + w, py)
    return pins


def _draw_top_antennas(ax, lora_x, ant1_x, ant2_x, lora_base_y=25.40, bar_y=25.90):
    """Draws realistic LoRa whip antenna and Dual-GNSS T-Bar Crossbar antennas."""
    # LoRa 900MHz Whip Antenna
    ax.add_patch(Rectangle((lora_x - 0.15, lora_base_y), 0.30, 0.32, facecolor="#D4AF37", edgecolor="#78350F", lw=0.9, zorder=4))
    ax.add_patch(FancyBboxPatch((lora_x - 0.13, lora_base_y + 0.32), 0.26, 2.75, boxstyle="round,pad=0.0,rounding_size=0.11",
                                facecolor="#1E293B", edgecolor="#0F172A", lw=1.0, zorder=4))
    ax.text(lora_x + 0.32, lora_base_y + 1.85, "900 MHz LoRa Ant\nCH 75 (925.125 MHz, 1W)",
            color="#0F172A", fontsize=8.3, fontweight="bold", ha="left", va="center", zorder=5)

    # Aluminum T-Bar Crossbar
    bar_x0 = ant1_x - 1.90
    bar_w = (ant2_x + 1.90) - bar_x0
    _rect_shadow(ax, bar_x0, bar_y, bar_w, 0.64, r=0.08, dx=0.06, dy=-0.06)
    ax.add_patch(FancyBboxPatch((bar_x0, bar_y), bar_w, 0.64, boxstyle="round,pad=0.0,rounding_size=0.08",
                                facecolor="#E2E8F0", edgecolor="#64748B", lw=1.3, zorder=3))
    ax.text(0.5 * (ant1_x + ant2_x), bar_y + 0.32,
            "Aluminum Crossbar (B = 55.5 cm | Mid = 27.75 cm | H = 1.80 m)",
            color="#1E293B", fontsize=6.9, fontweight="bold", ha="center", va="center", zorder=4)

    for axp, title, sub in [
        (ant1_x, "ANT1: Master (PVT)", "L1/L2/L5 GNSS"),
        (ant2_x, "ANT2: Slave (Hdg)", "Moving-Baseline"),
    ]:
        ax.add_patch(Rectangle((axp - 0.15, bar_y + 0.64), 0.30, 0.42, facecolor="#94A3B8", edgecolor="#475569", lw=0.8, zorder=3.5))
        ax.add_patch(FancyBboxPatch((axp - 0.54, bar_y + 1.06), 1.08, 1.24, boxstyle="round,pad=0.0,rounding_size=0.28",
                                    facecolor="#F8FAFC", edgecolor="#334155", lw=1.4, zorder=4))
        ax.add_patch(Rectangle((axp - 0.54, bar_y + 1.06), 1.08, 0.30, facecolor="#334155", edgecolor="#1E293B", lw=0.9, zorder=4.2))
        ax.text(axp, bar_y + 2.92, title, color="#0F172A", fontsize=8.6, fontweight="bold", ha="center", va="center", zorder=5)
        ax.text(axp, bar_y + 2.54, sub, color="#475569", fontsize=7.4, ha="center", va="center", zorder=5)


def generate_3_5b_current_diagram(out_dir: str):
    """Generates clean white-background Arduino-style high-level wiring diagram for current v0.8.18."""
    fig, ax = plt.subplots(figsize=(13.5, 15.75), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")
    ax.set_facecolor("#FFFFFF")
    ax.set_xlim(0, 27.0)
    ax.set_ylim(0, 31.5)
    ax.set_aspect("equal")
    ax.axis("off")

    # 1. Top-Left Title & Date (Arduino / MFS Style)
    ax.text(0.9, 30.65, "RTK Rover High-Level Wiring Diagram", color="#0F172A", fontsize=21, fontweight="normal",
            ha="left", va="center", family="sans-serif")
    ax.text(0.9, 29.92, "2026-11-18  |  Firmware v0.8.18  (Waveshare 3.5B + UM982 + E220 + SEN0696 + Switched 5V Power Module)",
            color="#334155", fontsize=10.5, ha="left", va="center", family="sans-serif")

    # 2. Hardware Modules (Waveshare y=13.0..24.0; Header pins at y=21.85 down to 14.35)
    lora_pins = _draw_e220_module(ax, 1.85, 17.00, 4.30, 6.00)
    esp_pins = _draw_waveshare_35b_pcb(ax, 9.85, 13.00, 6.40, 11.00)
    um_pins = _draw_um982_module(ax, 19.00, 17.40, 6.20, 6.00)
    sen_pins = _draw_sen0696_pcb(ax, 19.45, 12.40, 5.65, 3.80)
    pwr_pins = _draw_power_module(ax, 1.85, 7.20, 5.75, 5.05)
    bat_pins = _draw_1s_battery(ax, 7.65, 25.05, 3.05, 2.95, top_edge_term=True)
    _draw_top_antennas(ax, lora_pins["ANT"][0], um_pins["ANT1"][0], um_pins["ANT2"][0], lora_base_y=25.40, bar_y=25.95)

    # Module Captions
    ax.text(4.00, 16.52, "Ebyte E220-900T30S (LoRa 1W)", color="#0F172A", fontsize=9.4, fontweight="bold", ha="center", va="center")
    ax.text(4.00, 16.08, "115200 Mode0 / 9600 Mode3 Cfg", color="#475569", fontsize=7.6, ha="center", va="center")

    ax.text(13.20, 12.42, "Waveshare ESP32-S3 Touch LCD 3.5B", color="#0F172A", fontsize=9.8, fontweight="bold", ha="center", va="center")
    ax.text(13.20, 11.88, "Internal I2C0 (IO7/8): 0x18 ES8311 | 0x20 TCA9554 | 0x34 AXP2101\n0x3B Touch | 0x51 RTC | 0x6B QMI8658",
            color="#475569", fontsize=6.6, ha="center", va="center")

    ax.text(22.10, 16.92, "Holybro H-RTK UM982 (Single-UART COM1)", color="#0F172A", fontsize=9.4, fontweight="bold", ha="center", va="center")
    ax.text(22.27, 11.88, "DFRobot Gravity 10DOF (SEN0696) 0x4A", color="#0F172A", fontsize=9.4, fontweight="bold", ha="center", va="center")
    ax.text(9.18, 24.55, "1S Li-Ion / LiPo Battery\n(Direct to 3.5B BAT JST)", color="#0F172A", fontsize=7.8, fontweight="bold", ha="center", va="center")
    ax.text(4.72, 6.72, "Custom Switched +5.00V Power Module", color="#0F172A", fontsize=9.0, fontweight="bold", ha="center", va="center")

    # 3. Direct 1S LiPo -> Waveshare BAT JST Wires (Uses Onboard AXP2101 Charging)
    _ortho_wire(ax, [bat_pins["BAT+"], (11.91, bat_pins["BAT+"][1]), esp_pins["BAT_JST+"]], color="#DC2626", lw=2.8, z=6)
    _ortho_wire(ax, [bat_pins["BAT-"], (12.27, bat_pins["BAT-"][1]), esp_pins["BAT_JST-"]], color="#0F172A", lw=2.8, z=6)
    ax.text(11.35, 27.95, "JST-PH2.0\nDirect", color="#334155", fontsize=6.6, fontweight="bold", ha="center", va="center")

    # USB-C Note cleanly top-right of BAT JST
    _ortho_wire(ax, [esp_pins["USBC_TOP"], (esp_pins["USBC_TOP"][0], 24.75), (14.65, 24.75), (14.65, 25.35)],
                color="#64748B", lw=1.6, ls="--", z=5)
    ax.text(14.65, 25.85, "USB-C: 5V VBUS\n(500mA Batt Charge / Debug)",
            color="#334155", fontsize=7.4, fontweight="bold", ha="center", va="center")

    # 4. Coax RF Cables
    _ortho_wire(ax, [lora_pins["ANT"], (lora_pins["ANT"][0], 25.40)], color="#1E293B", lw=3.6, z=5)
    ax.text(lora_pins["ANT"][0] + 0.14, 24.25, "50 Ohm Coax", color="#334155", fontsize=7.0, ha="left", va="center")
    _ortho_wire(ax, [um_pins["ANT1"], (um_pins["ANT1"][0], 25.95)], color="#1E293B", lw=3.4, z=5)
    _ortho_wire(ax, [um_pins["ANT2"], (um_pins["ANT2"][0], 25.95)], color="#1E293B", lw=3.4, z=5)
    ax.text(0.5 * (um_pins["ANT1"][0] + um_pins["ANT2"][0]), 24.65, "50 Ohm MMCX/SMA Coax", color="#334155", fontsize=7.2, ha="center", va="center")

    # 5. E220 <-> Waveshare 3.5B Signal Wires (Left Header Side, Clean 1:1 Horizontal!)
    e220_wires = [
        (lora_pins["AUX"],  esp_pins[5],  "#D97706", "-",  "AUX (IO21)"),
        (lora_pins["NRST"], esp_pins[7],  "#64748B", "--", "NRST Opt (IO38)"),
        (lora_pins["M0"],   esp_pins[9],  "#92400E", "-",  "M0 (IO39)"),
        (lora_pins["M1"],   esp_pins[11], "#4D7C0F", "-",  "M1 (IO40)"),
        (lora_pins["TXD"],  esp_pins[13], "#9333EA", "-",  "TXD->RX0 (IO41)"),
        (lora_pins["RXD"],  esp_pins[15], "#DB2777", "-",  "RXD<-TX0 (IO42)"),
    ]
    for p_lora, p_esp, col, ls, lbl in e220_wires:
        _ortho_wire(ax, [p_lora, p_esp], color=col, lw=2.8, ls=ls, z=6)
        ax.text(7.45, p_esp[1] + 0.16, lbl, color="#1E293B", fontsize=6.3, fontweight="bold",
                ha="center", va="center", bbox=dict(boxstyle="round,pad=0.04", facecolor="#FFFFFF", edgecolor="none", alpha=0.90), zorder=7)

    # 6. UM982 <-> Waveshare 3.5B COM1 Wires (Right Header Side, Clean Horizontal!)
    _ortho_wire(ax, [esp_pins[16], um_pins["TXD0"]], color="#0D9488", lw=3.0, z=6)
    ax.text(17.62, esp_pins[16][1] + 0.16, "COM1 TX->IO17", color="#0F172A", fontsize=6.5, fontweight="bold",
            ha="center", va="center", bbox=dict(boxstyle="round,pad=0.05", facecolor="#FFFFFF", edgecolor="none", alpha=0.90), zorder=7)
    _ortho_wire(ax, [esp_pins[18], um_pins["RXD0"]], color="#0284C7", lw=3.0, z=6)
    ax.text(17.62, esp_pins[18][1] - 0.18, "COM1 RX<-IO18", color="#0F172A", fontsize=6.5, fontweight="bold",
            ha="center", va="center", bbox=dict(boxstyle="round,pad=0.05", facecolor="#FFFFFF", edgecolor="none", alpha=0.90), zorder=7)

    # 7. SEN0696 <-> Waveshare 3.5B I2C0 Gravity 4-Pin Cable (Straight Horizontal!)
    _ortho_wire(ax, [esp_pins[26], sen_pins["SCL"]], color="#2563EB", lw=3.2, z=6)
    ax.text(17.05, esp_pins[26][1] + 0.17, "SCL Blu", color="#1E3A8A", fontsize=6.4, fontweight="bold", ha="center", va="center", zorder=7)

    _ortho_wire(ax, [esp_pins[28], sen_pins["SDA"]], color="#16A34A", lw=3.2, z=6)
    ax.text(17.05, esp_pins[28][1] + 0.15, "SDA Grn", color="#14532D", fontsize=6.4, fontweight="bold", ha="center", va="center", zorder=7)

    _ortho_wire(ax, [esp_pins[30], sen_pins["GND"]], color="#0F172A", lw=3.2, z=6)
    _ortho_wire(ax, [esp_pins[32], sen_pins["VCC"]], color="#EF4444", lw=3.2, z=6)
    ax.text(17.05, esp_pins[32][1] - 0.18, "3V3 Red", color="#991B1B", fontsize=6.4, fontweight="bold", ha="center", va="center", zorder=7)

    # 8. Magenta Box: Optional Bench Hardening (4.7K External I2C0 Pull-Ups + Legacy MMC5983MA)
    mag_box = FancyBboxPatch(
        (17.35, 7.95), 8.35, 3.55,
        boxstyle="round,pad=0.0,rounding_size=0.14",
        facecolor="#FFFDFE", edgecolor="#EC4899", lw=1.8, zorder=2
    )
    ax.add_patch(mag_box)
    ax.text(25.45, 11.18, "Optional Bench Hardening / Fallback", color="#DB2777",
            fontsize=8.6, fontweight="bold", ha="right", va="center", zorder=4)

    tx_scl, tx_sda, tx_3v3 = 17.95, 18.45, 18.95
    _junc_dot(ax, tx_scl, sen_pins["SCL"][1])
    _junc_dot(ax, tx_sda, sen_pins["SDA"][1])
    _junc_dot(ax, tx_3v3, sen_pins["VCC"][1])
    _ortho_wire(ax, [(tx_scl, sen_pins["SCL"][1]), (tx_scl, 10.25), (19.55, 10.25)], color="#2563EB", lw=2.2, z=6)
    _ortho_wire(ax, [(tx_sda, sen_pins["SDA"][1]), (tx_sda, 9.25),  (19.55, 9.25)],  color="#16A34A", lw=2.2, z=6)
    _ortho_wire(ax, [(tx_3v3, sen_pins["VCC"][1]), (tx_3v3, 8.55),  (21.85, 8.55), (21.85, 10.25)], color="#EF4444", lw=2.2, z=5.5)
    _resistor_zigzag(ax, 19.55, 10.25, 21.35, 10.25, "4.7K", label_offset=(0.0, 0.28), horizontal=True, z=7)
    _resistor_zigzag(ax, 19.55, 9.25,  21.35, 9.25,  "4.7K", label_offset=(0.0, 0.28), horizontal=True, z=7)
    ax.plot([21.35, 21.85], [10.25, 10.25], color="#EF4444", lw=2.2, zorder=6)
    ax.plot([21.35, 21.85], [9.25,  9.25],  color="#EF4444", lw=2.2, zorder=6)
    _junc_dot(ax, 21.85, 9.25)

    ax.text(22.25, 9.50, "4.7K Pull-Ups to 3V3\n(SEN0696 End)\n\nAlt/Legacy Compass:\nQwiic MMC5983MA\n(I2C Addr 0x30)",
            color="#334155", fontsize=7.2, ha="left", va="center", zorder=5)

    # 9. Power Module Inputs (From 3.5B Header Left: P1 VBAT, P29 GND, P31 3V3)
    _ortho_wire(ax, [esp_pins[1],  (8.85, esp_pins[1][1]),  (8.85, pwr_pins["VBAT_IN"][1]), pwr_pins["VBAT_IN"]], color="#991B1B", lw=3.0, z=6)
    _ortho_wire(ax, [esp_pins[29], (9.20, esp_pins[29][1]), (9.20, pwr_pins["GND_IN"][1]),  pwr_pins["GND_IN"]],  color="#0F172A", lw=2.8, z=6)
    _ortho_wire(ax, [esp_pins[31], (9.52, esp_pins[31][1]), (9.52, pwr_pins["3V3_EN"][1]),  pwr_pins["3V3_EN"]],  color="#F97316", lw=2.8, z=6)
    ax.text(8.68, 16.05, "VBAT\n(P1)", color="#991B1B", fontsize=6.8, fontweight="bold", ha="right", va="center",
            bbox=dict(boxstyle="round,pad=0.04", facecolor="#FFFFFF", edgecolor="none", alpha=0.90), zorder=7)
    ax.text(8.22, pwr_pins["GND_IN"][1] + 0.18, "GND", color="#0F172A", fontsize=6.4, fontweight="bold", ha="center", va="center", zorder=7)
    ax.text(8.22, pwr_pins["3V3_EN"][1] + 0.18, "3V3", color="#C2410C", fontsize=6.4, fontweight="bold", ha="center", va="center", zorder=7)

    # 10. Separate Output Harness 1: Power Module -> Ebyte E220 (Dedicated +5.00V & GND Pair)
    _ortho_wire(ax, [pwr_pins["E220_5V"],  (0.95, pwr_pins["E220_5V"][1]),  (0.95, lora_pins["VCC"][1]),     lora_pins["VCC"]],     color="#DC2626", lw=3.2, z=6)
    _ortho_wire(ax, [pwr_pins["E220_GND"], (1.32, pwr_pins["E220_GND"][1]), (1.32, lora_pins["GND_PWR"][1]), lora_pins["GND_PWR"]], color="#0F172A", lw=3.0, z=6)
    ax.text(0.76, 16.80, "E220 +5.00V / GND Pair", color="#991B1B", fontsize=7.6, fontweight="bold", rotation=90, ha="right", va="center", zorder=7)

    # 11. Separate Output Harness 2: Power Module -> Holybro UM982 (Dedicated +5.00V & GND Pair)
    _ortho_wire(ax, [pwr_pins["UM_5V"],  (8.25, pwr_pins["UM_5V"][1]),  (8.25, 7.08), (26.35, 7.08), (26.35, um_pins["5V_IN"][1]), um_pins["5V_IN"]],
                color="#DC2626", lw=3.2, z=6)
    _ortho_wire(ax, [pwr_pins["UM_GND"], (7.95, pwr_pins["UM_GND"][1]), (7.95, 6.58), (25.95, 6.58), (25.95, um_pins["GND"][1]),   um_pins["GND"]],
                color="#0F172A", lw=3.0, z=6)
    ax.text(17.10, 7.36, "UM982 Dedicated +5.00V & GND Harness (Separate from E220 Pair)",
            color="#991B1B", fontsize=7.6, fontweight="bold", ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.05", facecolor="#FFFFFF", edgecolor="none", alpha=0.92), zorder=7)

    # 12. Bottom-Left Notes Block
    notes = (
        "POWER & HARDWARE NOTES\n"
        "---------------------------------------------------------------\n"
        "- 1S LiPo plugs directly into 3.5B BAT JST (AXP2101 500mA Chg)\n"
        "- NEVER tie +5.00V Boost Output to 3.5B Pin 2 (5V / VBUS)\n"
        "- Power Module feeds SEPARATE +5V/GND pairs to UM982 & E220\n"
        "- Off-State Leak: 0.00 uA; see Figure 1B for Module Schematic\n"
        "- Battery Est (1S 3.7V @ ~806mA): 3Ah ~3.2-3.4h | 10Ah ~10.6-11.2h\n"
        "- COM2 (IO47/48) Freed | BOOT (IO0) Long-Press >= 1.5s Screenshot"
    )
    ax.text(0.95, 3.00, notes, color="#1E293B", fontsize=7.4, family="monospace",
            ha="left", va="center", linespacing=1.35)

    # 13. Bottom-Right Blue Monospaced Pinout Legend (Exact MFS_Wiring_Diagram.png Style!)
    ax.text(12.25, 5.82, "Waveshare ESP32-S3 Touch LCD 3.5B (v0.8.18)",
            color="#0F172A", fontsize=10.8, fontweight="normal", ha="left", va="center", family="sans-serif")
    pin_table = (
        "Pin       GPIO       Function      Target / Connection (Color / Note)\n"
        "-------------------------------------------------------------------------\n"
        "BAT_JST   VBAT       1S_BATT       1S LiPo Direct (AXP2101 500mA Chg)\n"
        "P1        BAT        VBAT_OUT      -> Power Module VBAT IN (DkRed)\n"
        "P5..P15   IO21..42   E220_CTRL     AUX, NRST*, M0, M1, TXD, RXD (Left)\n"
        "P16,P18   IO17,18    UM982_COM1    <- TXD0 / -> RXD0 @ 115200 (Teal/Cyan)\n"
        "P26,P28   IO7,8      I2C0_BUS      SEN0696 SCL/SDA 400k (Grav Blu/Grn)\n"
        "P29,P30   GND        SYS_GND       -> Power Module GND / SEN0696 - (Blk)\n"
        "P31,P32   3V3        +3.3V_OUT     P31->PwrMod EN (Org) | P32->SEN+ (Red)\n"
        "PwrMod    +5V/GND    DUAL_5V_OUT   Two Separate 5V/GND Pairs: UM982 & E220"
    )
    ax.text(12.25, 3.25, pin_table, color="#0018A8", fontsize=7.2, fontweight="bold",
            ha="left", va="center", family="monospace", linespacing=1.36)

    plt.tight_layout(pad=0.25)
    svg_path = os.path.join(out_dir, "wiring_diagram_3_5B_current.svg")
    png_path = os.path.join(out_dir, "wiring_diagram_3_5B_current.png")
    fig.savefig(svg_path, format="svg", facecolor=fig.get_facecolor())
    fig.savefig(png_path, format="png", dpi=300, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"[+] Saved: {svg_path}")
    print(f"[+] Saved: {png_path}")


def generate_power_module_diagram(out_dir: str):
    """Generates clean white-background detail wiring schematic for the Switched +5.00V Power Module."""
    fig, ax = plt.subplots(figsize=(12.0, 7.6), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")
    ax.set_facecolor("#FFFFFF")
    ax.set_xlim(0, 24.0)
    ax.set_ylim(0, 15.2)
    ax.set_aspect("equal")
    ax.axis("off")

    # 1. Title Block
    ax.text(0.8, 14.42, "RTK Rover Switched +5.00V Power Module Wiring Diagram",
            color="#0F172A", fontsize=17.5, fontweight="normal", ha="left", va="center", family="sans-serif")
    ax.text(0.8, 13.78, "2026-11-18  |  High-Side Switch (NDP6020P + 2N2222) + MT3608 Boost + 470 uF / 0.1 uF Filter (Fig. 1B)",
            color="#334155", fontsize=9.8, ha="left", va="center", family="sans-serif")

    # 2. Left Terminal Block: Inputs from Waveshare 3.5B Header
    _rect_shadow(ax, 0.85, 5.55, 3.45, 7.10, r=0.14)
    ax.add_patch(FancyBboxPatch((0.85, 5.55), 3.45, 7.10, boxstyle="round,pad=0.0,rounding_size=0.14",
                                facecolor="#164E96", edgecolor="#0C356A", lw=1.4, zorder=2.5))
    ax.text(2.57, 12.20, "FROM WAVESHARE\n3.5B HEADER", color="#F8FAFC", fontsize=8.0, fontweight="bold",
            ha="center", va="center", family="monospace", zorder=3.5)

    in_pts = {}
    for key, py, lbl, sub, col in [
        ("VBAT", 11.15, "P1: BAT", "3.5-4.2V", "#FCA5A5"),
        ("GND",  8.95,  "P29: GND", "Sys Ref",  "#E2E8F0"),
        ("3V3",  6.55,  "P31: 3V3", "Gate En",  "#FDBA74"),
    ]:
        _pad_hole(ax, 4.10, py, radius=0.095, z=4)
        ax.text(3.86, py + 0.14, lbl, color=col, fontsize=7.6, fontweight="bold", ha="right", va="center", family="monospace", zorder=4)
        ax.text(3.86, py - 0.24, sub, color="#CBD5E1", fontsize=6.4, ha="right", va="center", zorder=4)
        in_pts[key] = (4.30, py)

    # 3. High-Side P-FET (NDP6020P) & NPN Driver (2N2222) + R1/R2 10K
    # NDP6020P TO-220 package diagram (top center-left)
    _rect_shadow(ax, 6.20, 9.75, 3.30, 2.55, r=0.12)
    ax.add_patch(FancyBboxPatch((6.20, 9.75), 3.30, 2.55, boxstyle="round,pad=0.0,rounding_size=0.12",
                                facecolor="#F8FAFC", edgecolor="#334155", lw=1.4, zorder=3))
    ax.add_patch(Rectangle((7.15, 11.65), 1.40, 0.45, facecolor="#94A3B8", edgecolor="#475569", lw=1.0, zorder=3.5))
    ax.add_patch(Circle((7.85, 11.88), 0.12, facecolor="#FFFFFF", edgecolor="#475569", lw=0.8, zorder=3.6))
    ax.add_patch(Rectangle((7.15, 10.65), 1.40, 1.00, facecolor="#1E293B", edgecolor="#0F172A", lw=1.0, zorder=3.5))
    ax.text(7.85, 11.15, "NDP6020P\nP-FET", color="#F8FAFC", fontsize=6.6, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.8)
    for px, plbl, pcol in [(6.70, "1:Gate", "#0F172A"), (7.85, "2:Drain", "#C2410C"), (9.00, "3:Source", "#991B1B")]:
        _pad_hole(ax, px, 10.08, radius=0.075, z=4)
        ax.text(px, 10.38, plbl, color=pcol, fontsize=6.2, fontweight="bold", ha="center", va="center", family="monospace", zorder=4)

    # VBAT (y=11.15) -> Node (5.20, 11.15) -> top (12.60) -> right outside box (9.80) -> bottom into S(3) (9.00, 10.08) so NO text crossed!
    _ortho_wire(ax, [in_pts["VBAT"], (5.20, 11.15), (5.20, 12.60), (9.80, 12.60), (9.80, 9.48), (9.00, 9.48), (9.00, 10.08)],
                color="#991B1B", lw=3.0, z=6)
    _junc_dot(ax, 5.20, 11.15)
    # R1: 10K pull-up from VBAT (5.20, 11.15) to Gate (6.70, 9.20) with label placed cleanly ABOVE resistor
    _ortho_wire(ax, [(5.20, 11.15), (5.20, 9.20), (5.35, 9.20)], color="#991B1B", lw=2.2, z=6)
    _resistor_zigzag(ax, 5.35, 9.20, 6.50, 9.20, "R1: 10K", label_offset=(-0.05, 0.34), horizontal=True, z=7)
    _ortho_wire(ax, [(6.50, 9.20), (6.70, 9.20), (6.70, 10.08)], color="#0F172A", lw=2.4, z=6)
    _junc_dot(ax, 6.70, 9.20)

    # Drain (7.85, 10.08) -> Switched VBAT -> MT3608 VIN+ (11.35, 10.65)
    _ortho_wire(ax, [(7.85, 10.08), (7.85, 9.10), (10.45, 9.10), (10.45, 10.65), (11.35, 10.65)], color="#EA580C", lw=3.2, z=6)
    ax.text(10.50, 11.15, "Switched\nVBAT", color="#C2410C", fontsize=7.0, fontweight="bold", ha="center", va="center", zorder=7)

    # 2N2222 NPN Transistor package diagram (bottom center-left)
    _rect_shadow(ax, 6.50, 5.60, 3.00, 2.25, r=0.12)
    ax.add_patch(FancyBboxPatch((6.50, 5.60), 3.00, 2.25, boxstyle="round,pad=0.0,rounding_size=0.12",
                                facecolor="#F8FAFC", edgecolor="#334155", lw=1.4, zorder=3))
    ax.add_patch(FancyBboxPatch((7.45, 7.05), 1.10, 0.58, boxstyle="round,pad=0.0,rounding_size=0.12",
                                facecolor="#1E293B", edgecolor="#0F172A", lw=1.0, zorder=3.5))
    ax.text(8.00, 7.34, "2N2222\nTO-92", color="#F8FAFC", fontsize=6.4, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.8)
    for px, plbl, pcol in [(7.00, "1:E", "#0F172A"), (8.00, "2:B", "#C2410C"), (9.00, "3:C", "#0F172A")]:
        _pad_hole(ax, px, 6.00, radius=0.075, z=4)
        ax.text(px, 6.32, plbl, color=pcol, fontsize=6.5, fontweight="bold", ha="center", va="center", family="monospace", zorder=4)

    # Collector Pin 3 (9.00, 6.00) -> up to Gate junction (6.70, 9.20)
    _ortho_wire(ax, [(9.00, 6.00), (9.00, 5.80), (9.75, 5.80), (9.75, 8.65), (6.70, 8.65), (6.70, 9.20)], color="#0F172A", lw=2.3, z=6)
    _junc_dot(ax, 6.70, 8.65)
    # 3V3_EN (4.30, 6.55) -> R2: 10K -> Base Pin 2 (8.00, 6.00)
    _ortho_wire(ax, [in_pts["3V3"], (4.65, 6.55)], color="#F97316", lw=2.6, z=6)
    _resistor_zigzag(ax, 4.65, 6.55, 6.10, 6.55, "R2: 10K", label_offset=(0.0, 0.40), horizontal=True, z=7)
    _ortho_wire(ax, [(6.10, 6.55), (6.28, 6.55), (6.28, 5.30), (8.00, 5.30), (8.00, 6.00)], color="#F97316", lw=2.4, z=6)

    # 4. Blue MT3608 Boost PCB (Center-Right, Left=VIN+/-, Right=VOUT+/-)
    mt_pins = _draw_mt3608_module(ax, 11.35, 7.15, 4.35, 4.40, flip_lr=True)
    ax.text(13.52, 6.70, "Adjust Trimpot -> +5.00V No-Load", color="#1E3A8A", fontsize=7.6, fontweight="bold", ha="center", va="center")

    # 5. Output Bus (+5.00V Rail + 470uF/0.1uF Filter + Separate Twin Harnesses)
    vout_y = mt_pins["VOUT+"][1]
    star_gnd_y = 4.75

    # Common Star GND bus across bottom
    _ortho_wire(ax, [in_pts["GND"], (4.85, 8.95), (4.85, star_gnd_y), (19.95, star_gnd_y)], color="#0F172A", lw=3.2, z=5.8)
    _ortho_wire(ax, [(7.00, 6.00), (7.00, star_gnd_y)], color="#0F172A", lw=2.4, z=5.8)
    _junc_dot(ax, 7.00, star_gnd_y)
    _ortho_wire(ax, [mt_pins["VIN-"], (11.00, mt_pins["VIN-"][1]), (11.00, star_gnd_y)], color="#0F172A", lw=2.8, z=5.8)
    _junc_dot(ax, 11.00, star_gnd_y)
    _ortho_wire(ax, [mt_pins["VOUT-"], (16.05, mt_pins["VOUT-"][1]), (16.05, star_gnd_y)], color="#0F172A", lw=3.0, z=5.8)
    _junc_dot(ax, 16.05, star_gnd_y)
    ax.text(12.20, star_gnd_y - 0.34, "Internal Common Star GND Bus", color="#0F172A", fontsize=7.4, fontweight="bold", ha="center", va="center")

    # +5.00V Output Bus feeding Caps + Output 1 + Output 2
    _ortho_wire(ax, [mt_pins["VOUT+"], (19.65, vout_y)], color="#DC2626", lw=3.6, z=6)
    ax.text(17.70, vout_y + 0.30, "+5.00V Rail", color="#B91C1C", fontsize=8.0, fontweight="bold", ha="center", va="center")

    # 470 uF Electrolytic + 0.1 uF Ceramic Decoupling Caps cleanly placed clear of all bus drops!
    for cx, clbl, ha_lbl, off_x, is_elec in [
        (17.15, "470 uF\nElec.",   "right", -0.38, True),
        (18.25, "0.1 uF\nCeram.",  "left",   0.38, False),
    ]:
        _junc_dot(ax, cx, vout_y)
        _junc_dot(ax, cx, star_gnd_y)
        ax.plot([cx, cx], [vout_y, 7.80], color="#DC2626", lw=2.2, zorder=6)
        ax.plot([cx, cx], [7.24, star_gnd_y], color="#0F172A", lw=2.2, zorder=6)
        ax.plot([cx - 0.30, cx + 0.30], [7.80, 7.80], color="#0F172A", lw=2.6, zorder=7)
        ax.plot([cx - 0.30, cx + 0.30], [7.24, 7.24], color="#0F172A", lw=2.6, zorder=7)
        if is_elec:
            ax.text(cx - 0.36, 8.05, "+", color="#DC2626", fontsize=8.5, fontweight="bold", ha="center", va="center", zorder=7)
        ax.text(cx + off_x, 7.52, clbl, color="#1E293B", fontsize=6.9, fontweight="bold",
                ha=ha_lbl, va="center", bbox=dict(boxstyle="round,pad=0.04", facecolor="#FFFFFF", edgecolor="none", alpha=0.90), zorder=7)

    # 6. Right Twin Output Connectors: OUT 1 (Holybro UM982) & OUT 2 (Ebyte E220)
    out_specs = [
        ("OUT 1: HOLYBRO UM982", 9.65, 12.15, "UM982 5V_IN (~350mA)", "UM982 GND"),
        ("OUT 2: EBYTE E220",    5.75, 8.25,  "E220 VCC (<=850mA pk)", "E220 GND"),
    ]
    for title, oy0, oy1, lbl5, lblg in out_specs:
        _rect_shadow(ax, 20.40, oy0, 3.05, oy1 - oy0, r=0.12)
        ax.add_patch(FancyBboxPatch((20.40, oy0), 3.05, oy1 - oy0, boxstyle="round,pad=0.0,rounding_size=0.12",
                                    facecolor="#1E242B", edgecolor="#0F172A", lw=1.4, zorder=3))
        ax.text(21.92, oy1 - 0.40, title, color="#F8FAFC", fontsize=6.9, fontweight="bold", ha="center", va="center", family="monospace", zorder=3.5)
        py5 = oy1 - 1.15
        pyg = oy0 + 0.55
        _pad_hole(ax, 20.61, py5, radius=0.085, z=4)
        _pad_hole(ax, 20.61, pyg, radius=0.085, z=4)
        ax.text(20.82, py5, lbl5, color="#FCA5A5", fontsize=6.2, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
        ax.text(20.82, pyg, lblg, color="#E2E8F0", fontsize=6.2, fontweight="bold", ha="left", va="center", family="monospace", zorder=4)
        # Wire from +5.00V rail & Star GND into Output pads cleanly right of 0.1uF text
        _junc_dot(ax, 19.65, vout_y)
        _junc_dot(ax, 19.95, star_gnd_y)
        _ortho_wire(ax, [(19.65, vout_y), (19.65, py5), (20.40, py5)], color="#DC2626", lw=3.0, z=6)
        _ortho_wire(ax, [(19.95, star_gnd_y), (19.95, pyg), (20.40, pyg)], color="#0F172A", lw=2.8, z=6)

    # 7. Bottom Classic Blue Monospaced BOM / Assembly Reference Table
    ax.text(0.90, 3.65, "Power Module BOM & Assembly Checklist", color="#0F172A", fontsize=10.2,
            fontweight="normal", ha="left", va="center", family="sans-serif")
    bom_txt = (
        "Ref        Part / Value           Function / Electrical Notes\n"
        "-----------------------------------------------------------------------------------------------\n"
        "Q1         NDP6020P (TO-220)      Logic P-FET High-Side Switch (S=VBAT, G=R1/Q2, D=Switched VBAT)\n"
        "Q2         2N2222 (TO-92 E-B-C)   NPN Gate Pull-Down (Base via R2=10K from 3.5B P31 3V3)\n"
        "R1, R2     10 kOhm (1/4W)         R1 holds Q1 OFF (0.00 uA leak); R2 limits 2N2222 Base ~0.26mA\n"
        "REG1       MT3608 Step-Up Boost   Dial VOUT+ = +5.00V unloaded (~85% typ eff @ 3.7V -> 5.0V)\n"
        "C1, C2     470 uF + 0.1 uF        Low-ESR Elec (>=10V) + Ceramic decouple 1W E220 TX bursts\n"
        "OUT1 / 2   Twin +5.00V / GND      Separate power harnesses to UM982 (5V_IN/GND) & E220 (VCC/GND)"
    )
    ax.text(0.90, 1.90, bom_txt, color="#0018A8", fontsize=7.6, fontweight="bold",
            ha="left", va="center", family="monospace", linespacing=1.36)

    plt.tight_layout(pad=0.25)
    svg_path = os.path.join(out_dir, "wiring_diagram_power_module.svg")
    png_path = os.path.join(out_dir, "wiring_diagram_power_module.png")
    fig.savefig(svg_path, format="svg", facecolor=fig.get_facecolor())
    fig.savefig(png_path, format="png", dpi=300, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"[+] Saved: {svg_path}")
    print(f"[+] Saved: {png_path}")


if __name__ == "__main__":
    print(f"=== {__filename__} {__version__} ===")
    parser = argparse.ArgumentParser(description="Generate RTK Rover hardware wiring diagrams.")
    parser.add_argument("--legacy", action="store_true", help="Also generate deprecated v0.8.0 and 3.49-V2 draft diagrams.")
    args = parser.parse_args()
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    schem_dir = os.path.join(repo_root, "docs", "schematics")
    os.makedirs(schem_dir, exist_ok=True)
    generate_3_5b_current_diagram(schem_dir)
    generate_power_module_diagram(schem_dir)
    if args.legacy:
        generate_3_5b_diagram(schem_dir)
        generate_3_49_v2_diagram(schem_dir)
