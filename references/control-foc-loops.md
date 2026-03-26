# Core Control & FOC Loops (Reference)

## Overview
This document covers the cascaded control loops for FOC: Position, Speed, and Current frames.

## 1. Cross-Coupling Decoupling
(TODO: Document Vd = Vd_pi - ω * Lq * Iq and Vq = Vq_pi + ω * (Ld * Id + Flux) equations and limits.)

## 2. Field Weakening (FW) & MTPA
(TODO: Describe MTPA trajectory control and Voltage Feed-forward for High RPM.)

## 3. High-Frequency Current Loop Execution
(TODO: Establish the < 20us execution budget constraint and recommend `.ramfunc` techniques.)
