#pragma once
#include <Arduino.h>

const char calibration_index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Sesame Quadruped Calibration Portal</title>
    <style>
        :root {
            --bg: #090d16;
            --panel: #131d2e;
            --panel-border: #1e293b;
            --card: #182235;
            --accent: #38bdf8;
            --accent-hover: #0ea5e9;
            --accent-glow: rgba(56, 189, 248, 0.25);
            --danger: #ef4444;
            --danger-bg: rgba(239, 68, 68, 0.15);
            --success: #22c55e;
            --warning: #f59e0b;
            --text: #f8fafc;
            --text-muted: #94a3b8;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; }
        body { background: var(--bg); color: var(--text); padding: 16px; max-width: 900px; margin: 0 auto; line-height: 1.4; }
        
        header { text-align: center; margin-bottom: 20px; padding: 12px 0; border-bottom: 1px solid var(--panel-border); }
        h1 { font-size: 1.6rem; color: var(--accent); letter-spacing: 0.5px; margin-bottom: 4px; }
        .subtitle { font-size: 0.85rem; color: var(--text-muted); }
        
        .section-box { background: var(--panel); border: 1px solid var(--panel-border); border-radius: 12px; padding: 16px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
        .section-title { font-size: 1.05rem; font-weight: 700; color: var(--text); margin-bottom: 12px; display: flex; align-items: center; justify-content: space-between; }
        .section-desc { font-size: 0.82rem; color: var(--text-muted); margin-bottom: 14px; }
        
        /* Global Batch Buttons */
        .batch-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; margin-bottom: 10px; }
        button { border: none; border-radius: 8px; font-weight: 600; cursor: pointer; transition: all 0.15s ease; padding: 10px 12px; font-size: 0.9rem; text-align: center; }
        button:active { transform: scale(0.97); }
        
        .btn-zero { background: var(--danger); color: white; }
        .btn-zero:hover { background: #dc2626; box-shadow: 0 0 10px rgba(239,68,68,0.4); }
        .btn-90 { background: #475569; color: white; }
        .btn-90:hover { background: #64748b; }
        .btn-180 { background: #7c3aed; color: white; }
        .btn-180:hover { background: #6d28d9; }
        .btn-stand { background: var(--success); color: white; }
        .btn-stand:hover { background: #16a34a; box-shadow: 0 0 10px rgba(34,197,94,0.4); }
        .btn-secondary { background: #334155; color: var(--text); }
        .btn-secondary:hover { background: #475569; }
        .btn-primary { background: var(--accent); color: #020617; }
        .btn-primary:hover { background: var(--accent-hover); }

        /* Legs Grid */
        .legs-layout { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
        @media (max-width: 680px) { .legs-layout { grid-template-columns: 1fr; } }
        
        .leg-card { background: var(--card); border: 1px solid var(--panel-border); border-radius: 10px; padding: 14px; }
        .leg-header { font-size: 0.95rem; font-weight: bold; color: var(--accent); margin-bottom: 12px; border-bottom: 1px solid rgba(255,255,255,0.08); padding-bottom: 6px; display: flex; justify-content: space-between; }
        
        /* Motor Row Control */
        .motor-control { background: rgba(0,0,0,0.25); border-radius: 8px; padding: 12px; margin-bottom: 12px; border: 1px solid rgba(255,255,255,0.04); }
        .motor-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
        .motor-name { font-weight: 700; font-size: 0.92rem; color: #fff; }
        .motor-pin { font-size: 0.75rem; color: var(--text-muted); margin-left: 6px; }
        .angle-badge { font-family: monospace; font-size: 1.1rem; font-weight: 700; color: var(--accent); background: rgba(56,189,248,0.12); padding: 2px 8px; border-radius: 6px; border: 1px solid rgba(56,189,248,0.3); }
        
        /* Slider */
        .slider-wrapper { margin-bottom: 8px; }
        input[type=range] {
            -webkit-appearance: none;
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #334155;
            outline: none;
        }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: var(--accent);
            cursor: pointer;
            border: 2px solid #fff;
            box-shadow: 0 0 6px rgba(0,0,0,0.5);
            transition: all 0.1s ease;
        }
        input[type=range]::-webkit-slider-thumb:hover {
            transform: scale(1.15);
            box-shadow: 0 0 10px var(--accent-glow);
        }
        
        /* Preset & Step Buttons */
        .btn-row { display: flex; gap: 5px; flex-wrap: wrap; margin-top: 6px; }
        .btn-preset { padding: 4px 8px; font-size: 0.78rem; border-radius: 4px; background: #334155; color: #e2e8f0; flex: 1; min-width: 38px; }
        .btn-preset:hover { background: #475569; }
        .btn-preset-zero { background: var(--danger-bg); color: #f87171; border: 1px solid rgba(239,68,68,0.4); font-weight: bold; }
        .btn-preset-zero:hover { background: var(--danger); color: white; }
        
        .btn-step { padding: 4px 7px; font-size: 0.78rem; border-radius: 4px; background: #1e293b; color: #cbd5e1; border: 1px solid #334155; }
        .btn-step:hover { background: #334155; }

        /* Pose Capture & Code Gen */
        .capture-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 14px; }
        @media (max-width: 600px) { .capture-grid { grid-template-columns: 1fr; } }
        .pose-card { background: var(--card); border: 1px solid var(--panel-border); border-radius: 8px; padding: 12px; }
        .pose-summary { font-family: monospace; font-size: 0.78rem; background: #0f172a; padding: 8px; border-radius: 6px; margin-top: 8px; color: #38bdf8; word-break: break-all; }
        
        pre { background: #0b1120; border: 1px solid #1e293b; border-radius: 8px; padding: 14px; font-size: 0.82rem; color: #e2e8f0; overflow-x: auto; max-height: 280px; }
        code { font-family: 'Consolas', 'Courier New', monospace; }
        
        .action-footer { display: flex; gap: 10px; margin-top: 12px; flex-wrap: wrap; }
        .status-pill { font-size: 0.75rem; padding: 4px 10px; border-radius: 12px; background: rgba(56,189,248,0.15); color: var(--accent); display: inline-flex; align-items: center; gap: 5px; }
        .live-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--success); display: inline-block; animation: pulse 1.5s infinite; }
        @keyframes pulse { 0% { opacity: 0.4; } 50% { opacity: 1; } 100% { opacity: 0.4; } }
    </style>
</head>
<body>

    <header>
        <h1>Sesame Quadruped Calibration</h1>
        <div class="subtitle">Complete 0° - 180° Range of Motion | Instant Servo Control | Hardware Offset Tuning</div>
    </header>

    <!-- SECTION 1: GLOBAL CONTROL BAR -->
    <div class="section-box">
        <div class="section-title">
            <span>Global Motor Presets</span>
            <span class="status-pill"><span class="live-dot"></span> Live ESP32 Ready</span>
        </div>
        <div class="section-desc">
            Quickly set all 8 motors simultaneously to check zero alignment, 90° neutral reference, or default stand height.
        </div>
        <div class="batch-grid">
            <button class="btn-zero" onclick="setAllAngles(0)">Set ALL to 0°</button>
            <button class="btn-90" onclick="setAllAngles(90)">Set ALL to 90° (Flat / Rest)</button>
            <button class="btn-180" onclick="setAllAngles(180)">Set ALL to 180°</button>
            <button class="btn-stand" onclick="setCanonicalStand()">Canonical Stand Pose</button>
        </div>
    </div>

    <!-- SECTION 2: 8-MOTOR CONTROLS (FULL FREEDOM 0-180°) -->
    <div class="section-box">
        <div class="section-title">
            <span>Individual Motor Scrollers & Zeroing</span>
            <span style="font-size: 0.8rem; font-weight: normal; color: var(--text-muted);">Range: 0° - 180° (Unrestricted)</span>
        </div>
        <div class="section-desc">
            Drag the scroller to move any servo in real-time. Use <strong>[0°]</strong> to zero a specific motor horn, or fine-tune with <strong>[±1°]</strong>.
        </div>

        <div class="legs-layout">
            <!-- FRONT LEFT LEG -->
            <div class="leg-card">
                <div class="leg-header">
                    <span>FRONT LEFT (FL) LEG</span>
                    <span style="font-size:0.75rem; color:var(--text-muted);">Pins 23 & 17</span>
                </div>

                <!-- L1: FL Hip -->
                <div class="motor-control" id="ctrl-L1">
                    <div class="motor-header">
                        <div><span class="motor-name">L1 (FL Hip)</span><span class="motor-pin">GPIO 23</span></div>
                        <span class="angle-badge" id="val-L1">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-L1" min="0" max="180" value="90"
                               oninput="onSliderDrag('L1', this.value)"
                               onchange="setMotorAngle('L1', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('L1', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L1', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L1', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L1', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L1', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('L1', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('L1', 1)">+1°</button>
                    </div>
                </div>

                <!-- L3: FL Knee -->
                <div class="motor-control" id="ctrl-L3">
                    <div class="motor-header">
                        <div><span class="motor-name">L3 (FL Knee)</span><span class="motor-pin">GPIO 17</span></div>
                        <span class="angle-badge" id="val-L3">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-L3" min="0" max="180" value="90"
                               oninput="onSliderDrag('L3', this.value)"
                               onchange="setMotorAngle('L3', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('L3', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L3', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L3', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L3', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L3', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('L3', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('L3', 1)">+1°</button>
                    </div>
                </div>
            </div>

            <!-- FRONT RIGHT LEG -->
            <div class="leg-card">
                <div class="leg-header">
                    <span>FRONT RIGHT (FR) LEG</span>
                    <span style="font-size:0.75rem; color:var(--text-muted);">Pins 15 & 4</span>
                </div>

                <!-- R1: FR Hip -->
                <div class="motor-control" id="ctrl-R1">
                    <div class="motor-header">
                        <div><span class="motor-name">R1 (FR Hip)</span><span class="motor-pin">GPIO 15</span></div>
                        <span class="angle-badge" id="val-R1">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-R1" min="0" max="180" value="90"
                               oninput="onSliderDrag('R1', this.value)"
                               onchange="setMotorAngle('R1', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('R1', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R1', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R1', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R1', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R1', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('R1', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('R1', 1)">+1°</button>
                    </div>
                </div>

                <!-- R4: FR Knee -->
                <div class="motor-control" id="ctrl-R4">
                    <div class="motor-header">
                        <div><span class="motor-name">R4 (FR Knee)</span><span class="motor-pin">GPIO 4</span></div>
                        <span class="angle-badge" id="val-R4">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-R4" min="0" max="180" value="90"
                               oninput="onSliderDrag('R4', this.value)"
                               onchange="setMotorAngle('R4', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('R4', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R4', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R4', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R4', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R4', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('R4', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('R4', 1)">+1°</button>
                    </div>
                </div>
            </div>

            <!-- BACK LEFT LEG -->
            <div class="leg-card">
                <div class="leg-header">
                    <span>BACK LEFT (RL) LEG</span>
                    <span style="font-size:0.75rem; color:var(--text-muted);">Pins 19 & 18</span>
                </div>

                <!-- L2: RL Hip -->
                <div class="motor-control" id="ctrl-L2">
                    <div class="motor-header">
                        <div><span class="motor-name">L2 (RL Hip)</span><span class="motor-pin">GPIO 19</span></div>
                        <span class="angle-badge" id="val-L2">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-L2" min="0" max="180" value="90"
                               oninput="onSliderDrag('L2', this.value)"
                               onchange="setMotorAngle('L2', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('L2', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L2', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L2', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L2', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L2', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('L2', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('L2', 1)">+1°</button>
                    </div>
                </div>

                <!-- L4: RL Knee -->
                <div class="motor-control" id="ctrl-L4">
                    <div class="motor-header">
                        <div><span class="motor-name">L4 (RL Knee)</span><span class="motor-pin">GPIO 18</span></div>
                        <span class="angle-badge" id="val-L4">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-L4" min="0" max="180" value="90"
                               oninput="onSliderDrag('L4', this.value)"
                               onchange="setMotorAngle('L4', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('L4', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L4', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L4', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L4', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('L4', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('L4', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('L4', 1)">+1°</button>
                    </div>
                </div>
            </div>

            <!-- BACK RIGHT LEG -->
            <div class="leg-card">
                <div class="leg-header">
                    <span>BACK RIGHT (RR) LEG</span>
                    <span style="font-size:0.75rem; color:var(--text-muted);">Pins 2 & 16</span>
                </div>

                <!-- R2: RR Hip -->
                <div class="motor-control" id="ctrl-R2">
                    <div class="motor-header">
                        <div><span class="motor-name">R2 (RR Hip)</span><span class="motor-pin">GPIO 2</span></div>
                        <span class="angle-badge" id="val-R2">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-R2" min="0" max="180" value="90"
                               oninput="onSliderDrag('R2', this.value)"
                               onchange="setMotorAngle('R2', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('R2', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R2', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R2', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R2', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R2', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('R2', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('R2', 1)">+1°</button>
                    </div>
                </div>

                <!-- R3: RR Knee -->
                <div class="motor-control" id="ctrl-R3">
                    <div class="motor-header">
                        <div><span class="motor-name">R3 (RR Knee)</span><span class="motor-pin">GPIO 16</span></div>
                        <span class="angle-badge" id="val-R3">90°</span>
                    </div>
                    <div class="slider-wrapper">
                        <input type="range" id="slider-R3" min="0" max="180" value="90"
                               oninput="onSliderDrag('R3', this.value)"
                               onchange="setMotorAngle('R3', this.value)">
                    </div>
                    <div class="btn-row">
                        <button class="btn-preset btn-preset-zero" onclick="setMotorAngle('R3', 0)">0°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R3', 45)">45°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R3', 90)">90°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R3', 135)">135°</button>
                        <button class="btn-preset" onclick="setMotorAngle('R3', 180)">180°</button>
                        <button class="btn-step" onclick="stepMotor('R3', -1)">-1°</button>
                        <button class="btn-step" onclick="stepMotor('R3', 1)">+1°</button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- SECTION 3: POSE CAPTURE & CODE GENERATOR -->
    <div class="section-box">
        <div class="section-title">
            <span>Calibrated Poses & Firmware Code</span>
        </div>
        <div class="section-desc">
            Position the robot legs into desired posture, then click <strong>Capture</strong>. The live C++ code is generated automatically below.
        </div>

        <div class="capture-grid">
            <div class="pose-card">
                <button class="btn-primary" style="width:100%; margin-bottom:8px;" onclick="capturePose('Rest')">📌 Capture as HOME (Rest) Pose</button>
                <div style="font-size:0.8rem; color:var(--text-muted);">Recorded Home Angles:</div>
                <div class="pose-summary" id="summary-rest">R1:90 R2:90 L1:90 L2:90 R4:90 R3:90 L3:90 L4:90</div>
            </div>

            <div class="pose-card">
                <button class="btn-stand" style="width:100%; margin-bottom:8px;" onclick="capturePose('Stand')">📌 Capture as STAND Pose</button>
                <div style="font-size:0.8rem; color:var(--text-muted);">Recorded Stand Angles:</div>
                <div class="pose-summary" id="summary-stand">R1:135 R2:45 L1:45 L2:135 R4:0 R3:180 L3:0 L4:180</div>
            </div>
        </div>

        <div style="margin-top:14px;">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:8px;">
                <label style="font-weight:bold; font-size:0.9rem;">Ready-to-Paste C++ Firmware Code:</label>
                <button class="btn-secondary" style="padding:4px 10px; font-size:0.8rem;" id="btn-copy" onclick="copyCppCode()">📋 Copy C++ Code</button>
            </div>
            <pre><code id="code-snippet"></code></pre>
        </div>

        <div class="action-footer">
            <button class="btn-primary" onclick="exportCSV()" style="background:#0284c7; color:white;">💾 Download quadruped_calibration.csv</button>
        </div>
    </div>

    <script>
        const MOTOR_KEYS = ['R1', 'R2', 'L1', 'L2', 'R4', 'R3', 'L3', 'L4'];
        
        let currentAngles = {
            R1: 90, R2: 90, L1: 90, L2: 90,
            R4: 90, R3: 90, L3: 90, L4: 90
        };

        let savedRest = { R1: 90, R2: 90, L1: 90, L2: 90, R4: 90, R3: 90, L3: 90, L4: 90 };
        let savedStand = { R1: 135, R2: 45, L1: 45, L2: 135, R4: 0, R3: 180, L3: 0, L4: 180 };

        // Real-time debounced transmitter (trailing edge ~35ms)
        let pendingUpdates = {};
        let isFlushing = false;

        function onSliderDrag(motor, val) {
            val = parseInt(val);
            document.getElementById('val-' + motor).innerText = val + '°';
            currentAngles[motor] = val;
            updateCodeSnippet();

            pendingUpdates[motor] = val;
            if (!isFlushing) {
                flushSliderUpdates();
            }
        }

        async function flushSliderUpdates() {
            if (Object.keys(pendingUpdates).length === 0) {
                isFlushing = false;
                return;
            }
            isFlushing = true;
            const motor = Object.keys(pendingUpdates)[0];
            const val = pendingUpdates[motor];
            delete pendingUpdates[motor];

            try {
                await fetch(`/raw?motor=${motor}&val=${val}`);
            } catch (e) {
                console.error(e);
            }
            setTimeout(flushSliderUpdates, 35);
        }

        function setMotorAngle(motor, val) {
            val = Math.max(0, Math.min(180, parseInt(val)));
            const slider = document.getElementById('slider-' + motor);
            if (slider) slider.value = val;
            const badge = document.getElementById('val-' + motor);
            if (badge) badge.innerText = val + '°';
            currentAngles[motor] = val;
            updateCodeSnippet();

            fetch(`/raw?motor=${motor}&val=${val}`).catch(e => console.error(e));
        }

        function stepMotor(motor, delta) {
            const current = currentAngles[motor] || 90;
            setMotorAngle(motor, current + delta);
        }

        async function setAllAngles(val) {
            val = Math.max(0, Math.min(180, parseInt(val)));
            MOTOR_KEYS.forEach(m => {
                const slider = document.getElementById('slider-' + m);
                if (slider) slider.value = val;
                const badge = document.getElementById('val-' + m);
                if (badge) badge.innerText = val + '°';
                currentAngles[m] = val;
            });
            updateCodeSnippet();
            try {
                await fetch(`/all?val=${val}`);
            } catch (e) {
                console.error(e);
            }
        }

        const CANONICAL_STAND = {
            R1: 135, R2: 45, L1: 45, L2: 135,
            R4: 0, R3: 180, L3: 0, L4: 180
        };

        async function setCanonicalStand() {
            for (let [m, val] of Object.entries(CANONICAL_STAND)) {
                setMotorAngle(m, val);
                await new Promise(r => setTimeout(r, 30));
            }
        }

        function capturePose(poseName) {
            if (poseName === 'Rest') {
                savedRest = { ...currentAngles };
                document.getElementById('summary-rest').innerText = formatPoseLine(savedRest);
            } else if (poseName === 'Stand') {
                savedStand = { ...currentAngles };
                document.getElementById('summary-stand').innerText = formatPoseLine(savedStand);
            }
            updateCodeSnippet();
        }

        function formatPoseLine(p) {
            return MOTOR_KEYS.map(k => `${k}:${p[k]}`).join('  ');
        }

        function updateCodeSnippet() {
            const code = `// ====== CALIBRATED PHYSICAL JOINT ANGLES ======
// Paste these into firmware/movement-sequences.h

inline void runRestPose() {
  Serial.println(F("REST (Calibrated)"));
  setFaceWithMode("rest", FACE_ANIM_BOOMERANG);
  setServoAngle(R1, ${savedRest.R1});
  setServoAngle(R2, ${savedRest.R2});
  setServoAngle(L1, ${savedRest.L1});
  setServoAngle(L2, ${savedRest.L2});
  setServoAngle(R4, ${savedRest.R4});
  setServoAngle(R3, ${savedRest.R3});
  setServoAngle(L3, ${savedRest.L3});
  setServoAngle(L4, ${savedRest.L4});
}

inline void runStandPose(int face = 1) {
  Serial.println(F("STAND (Calibrated)"));
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE);
  setServoAngle(R1, ${savedStand.R1});
  setServoAngle(R2, ${savedStand.R2});
  setServoAngle(L1, ${savedStand.L1});
  setServoAngle(L2, ${savedStand.L2});
  setServoAngle(R4, ${savedStand.R4});
  setServoAngle(R3, ${savedStand.R3});
  setServoAngle(L3, ${savedStand.L3});
  setServoAngle(L4, ${savedStand.L4});
  if (face == 1) enterIdle();
}`;
            document.getElementById('code-snippet').innerText = code;
        }

        function copyCppCode() {
            const code = document.getElementById('code-snippet').innerText;
            navigator.clipboard.writeText(code).then(() => {
                const btn = document.getElementById('btn-copy');
                const orig = btn.innerText;
                btn.innerText = '✅ Copied to Clipboard!';
                btn.style.background = '#22c55e';
                setTimeout(() => {
                    btn.innerText = orig;
                    btn.style.background = '#334155';
                }, 2000);
            });
        }

        function exportCSV() {
            let csv = "Pose,R1,R2,L1,L2,R4,R3,L3,L4\n";
            csv += `Rest,${savedRest.R1},${savedRest.R2},${savedRest.L1},${savedRest.L2},${savedRest.R4},${savedRest.R3},${savedRest.L3},${savedRest.L4}\n`;
            csv += `Stand,${savedStand.R1},${savedStand.R2},${savedStand.L1},${savedStand.L2},${savedStand.R4},${savedStand.R3},${savedStand.L3},${savedStand.L4}\n`;

            const blob = new Blob([csv], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = 'quadruped_calibration.csv';
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
        }

        // Initialize code on load
        window.onload = () => {
            updateCodeSnippet();
        };
    </script>
</body>
</html>
)rawliteral";
