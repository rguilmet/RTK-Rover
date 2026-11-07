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

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"


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


if __name__ == "__main__":
    print(f"=== {__filename__} {__version__} ===")
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    schem_dir = os.path.join(repo_root, "docs", "schematics")
    os.makedirs(schem_dir, exist_ok=True)
    generate_3_5b_diagram(schem_dir)
    generate_3_49_v2_diagram(schem_dir)
