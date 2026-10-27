#pragma once
#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Quadruped Controller</title>
    <style>
        :root {
            --bg-top: #0a0a0a;
            --bg-bottom: #1a1a2e;
            --panel-bg: rgba(30, 30, 30, 0.8);
            --panel-border: #333;
            --accent: #ff8c42;
            --accent-hover: #ff9e5e;
            --text: #ffffff;
            --text-muted: #aaaaaa;
            --btn-base: #3a3a3a;
            --btn-base-hover: #4a4a4a;
            --stop-btn: #e63946;
            --stop-btn-hover: #f04b58;
            --font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
        }

        body {
            font-family: var(--font-family);
            background: linear-gradient(135deg, var(--bg-top), var(--bg-bottom));
            background-attachment: fixed;
            color: var(--text);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 15px;
            padding-top: 50px;
        }

        h2 {
            font-size: 1.2rem;
            margin-bottom: 15px;
            text-align: center;
            font-weight: 600;
            letter-spacing: 1px;
            text-transform: uppercase;
            color: var(--accent);
        }

        .header-bar {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 40px;
            background: rgba(10, 10, 10, 0.9);
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 0 15px;
            z-index: 100;
            border-bottom: 1px solid var(--panel-border);
        }

        #queue-status {
            font-size: 0.9rem;
            font-weight: bold;
            transition: color 0.3s;
        }
        #queue-status.full {
            color: var(--stop-btn);
        }

        #settings-btn {
            background: none;
            border: none;
            font-size: 1.2rem;
            cursor: pointer;
            color: var(--text);
        }

        .container {
            display: flex;
            flex-direction: column;
            gap: 20px;
            width: 100%;
            max-width: 900px;
        }

        @media (min-width: 768px) {
            .container {
                flex-direction: row;
                align-items: flex-start;
            }
            .col {
                flex: 1;
                display: flex;
                flex-direction: column;
                gap: 20px;
            }
        }

        .panel {
            background: var(--panel-bg);
            border: 1px solid var(--panel-border);
            border-radius: 16px;
            padding: 20px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
            backdrop-filter: blur(10px);
        }

        /* Buttons */
        button {
            font-family: var(--font-family);
            background: linear-gradient(to bottom, var(--btn-base), #2a2a2a);
            color: var(--text);
            border: 1px solid #444;
            border-radius: 12px;
            padding: 12px;
            font-size: 1rem;
            cursor: pointer;
            box-shadow: 0 4px 6px rgba(0,0,0,0.2);
            transition: all 0.2s;
        }
        button:active {
            transform: scale(0.95);
            box-shadow: 0 1px 3px rgba(0,0,0,0.3);
        }

        .pose-btn {
            background: linear-gradient(to bottom, var(--accent), #d16e25);
            border-color: #d16e25;
            font-weight: bold;
            color: #fff;
            text-shadow: 0 1px 2px rgba(0,0,0,0.5);
        }
        .pose-btn:active {
            background: linear-gradient(to bottom, var(--accent-hover), var(--accent));
        }

        /* D-Pad */
        .dpad-container {
            display: flex;
            justify-content: center;
            margin-bottom: 20px;
        }
        .dpad {
            display: grid;
            grid-template-columns: repeat(3, 60px);
            grid-template-rows: repeat(3, 60px);
            gap: 10px;
        }
        .dpad-btn {
            font-size: 1.5rem;
            display: flex;
            align-items: center;
            justify-content: center;
            border-radius: 12px;
        }
        .dpad-btn.up { grid-column: 2; grid-row: 1; }
        .dpad-btn.left { grid-column: 1; grid-row: 2; }
        .dpad-btn.right { grid-column: 3; grid-row: 2; }
        .dpad-btn.down { grid-column: 2; grid-row: 3; }
        .dpad-center { grid-column: 2; grid-row: 2; background: none; border: none; box-shadow: none; }

        .stop-btn {
            width: 100%;
            background: linear-gradient(to bottom, var(--stop-btn), #c32d39);
            border: 2px solid var(--stop-btn);
            font-weight: bold;
            font-size: 1.2rem;
            text-transform: uppercase;
            padding: 15px;
        }

        /* Speed Control */
        .speed-control {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 10px;
        }
        .speed-control span {
            font-size: 0.9rem;
            color: var(--text-muted);
        }
        
        input[type=range] {
            -webkit-appearance: none;
            width: 100%;
            background: transparent;
        }
        input[type=range]:focus {
            outline: none;
        }
        input[type=range]::-webkit-slider-runnable-track {
            width: 100%;
            height: 8px;
            cursor: pointer;
            background: #2a2a2a;
            border-radius: 4px;
            border: 1px solid #444;
        }
        input[type=range]::-webkit-slider-thumb {
            height: 24px;
            width: 24px;
            border-radius: 50%;
            background: var(--accent);
            cursor: pointer;
            -webkit-appearance: none;
            margin-top: -9px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.5);
        }
        
        .val-display {
            text-align: center;
            font-size: 0.9rem;
            color: var(--accent);
            margin-top: 5px;
        }

        /* Poses */
        .poses-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }

        /* Motors */
        .quick-actions {
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }
        .quick-actions button {
            flex: 1;
        }
        #motor-lock-msg {
            text-align: center;
            color: var(--stop-btn);
            font-size: 0.9rem;
            font-weight: bold;
            margin-bottom: 10px;
            display: none;
        }
        .motor-item {
            margin-bottom: 15px;
        }
        .motor-header {
            display: flex;
            justify-content: space-between;
            font-size: 0.9rem;
            margin-bottom: 5px;
        }

        /* Modal */
        .modal-overlay {
            position: fixed;
            top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0,0,0,0.8);
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 200;
            opacity: 0;
            pointer-events: none;
            transition: opacity 0.3s;
        }
        .modal-overlay.active {
            opacity: 1;
            pointer-events: all;
        }
        .modal-content {
            background: var(--panel-bg);
            border: 1px solid var(--panel-border);
            border-radius: 16px;
            padding: 20px;
            width: 90%;
            max-width: 400px;
            backdrop-filter: blur(10px);
        }
        .modal-content h2 { margin-bottom: 20px; }
        .setting-group { margin-bottom: 15px; }
        .setting-group label { display: block; font-size: 0.9rem; margin-bottom: 5px; color: var(--text-muted); }
        .setting-group input[type="number"], .setting-group select {
            width: 100%;
            padding: 10px;
            border-radius: 8px;
            border: 1px solid #444;
            background: #1a1a1a;
            color: white;
            font-family: var(--font-family);
        }
        .theme-picker {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            margin-top: 5px;
        }
        .color-dot {
            width: 30px;
            height: 30px;
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid transparent;
        }
        .color-dot.active { border-color: white; }
        .modal-actions {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }
        .modal-actions button { flex: 1; }
        .btn-primary { background: linear-gradient(to bottom, var(--accent), #d16e25); color: white; border:none; }

        .disabled { opacity: 0.5; pointer-events: none; }
    </style>
</head>
<body>

    <div class="header-bar">
        <div id="queue-status">Command Queue: 0/3</div>
        <button id="settings-btn" onclick="openSettings()">⚙️</button>
    </div>

    <div class="container">
        <!-- Left Column -->
        <div class="col">
            <!-- Movement Panel -->
            <div class="panel">
                <h2>Movement</h2>
                <div class="dpad-container">
                    <div class="dpad">
                        <button class="dpad-btn up" 
                            onmousedown="startMove('forward')" onmouseup="stopMove()" 
                            ontouchstart="tStartMove(event, 'forward')" ontouchend="tStopMove(event)">▲</button>
                        <button class="dpad-btn left" 
                            onmousedown="startMove('left')" onmouseup="stopMove()" 
                            ontouchstart="tStartMove(event, 'left')" ontouchend="tStopMove(event)">◀</button>
                        <div class="dpad-center"></div>
                        <button class="dpad-btn right" 
                            onmousedown="startMove('right')" onmouseup="stopMove()" 
                            ontouchstart="tStartMove(event, 'right')" ontouchend="tStopMove(event)">▶</button>
                        <button class="dpad-btn down" 
                            onmousedown="startMove('backward')" onmouseup="stopMove()" 
                            ontouchstart="tStartMove(event, 'backward')" ontouchend="tStopMove(event)">▼</button>
                    </div>
                </div>
                <button class="stop-btn" onmousedown="stopAll()" ontouchstart="tStopAll(event)">STOP ALL</button>
            </div>

            <!-- Speed Panel -->
            <div class="panel">
                <h2>Speed</h2>
                <div class="speed-control">
                    <span>Slow</span>
                    <input type="range" id="speedSlider" min="30" max="300" value="200" oninput="updateSpeedUi(this.value)" onchange="sendSpeed(this.value)">
                    <span>Fast</span>
                </div>
                <div class="val-display">Frame Delay: <span id="speedVal">100</span> ms</div>
            </div>

            <!-- Poses Panel -->
            <div class="panel">
                <h2>Poses & Actions</h2>
                <div class="poses-grid">
                    <button class="pose-btn" onclick="pose('rest')">Rest</button>
                    <button class="pose-btn" onclick="pose('stand')">Stand</button>
                    <button class="pose-btn" onclick="pose('wave')">Wave</button>
                    <button class="pose-btn" onclick="pose('dance')">Dance</button>
                    <button class="pose-btn" onclick="pose('swim')">Swim</button>
                    <button class="pose-btn" onclick="pose('point')">Point</button>
                    <button class="pose-btn" onclick="pose('pushup')">Pushup</button>
                    <button class="pose-btn" onclick="pose('bow')">Bow</button>
                    <button class="pose-btn" onclick="pose('cute')">Cute</button>
                    <button class="pose-btn" onclick="pose('freaky')">Freaky</button>
                    <button class="pose-btn" onclick="pose('worm')">Worm</button>
                    <button class="pose-btn" onclick="pose('shake')">Shake</button>
                    <button class="pose-btn" onclick="pose('shrug')">Shrug</button>
                    <button class="pose-btn" onclick="pose('dead')">Dead</button>
                    <button class="pose-btn" onclick="pose('crab')">Crab</button>
                </div>
            </div>
        </div>

        <!-- Right Column -->
        <div class="col">
            <div class="panel">
                <h2>Motor Control</h2>
                <div class="quick-actions">
                    <button onclick="pose('rest')">Home</button>
                    <button onclick="pose('stand')">Stand</button>
                </div>
                <div id="motor-lock-msg">Locked during animations</div>
                
                <div id="motor-sliders-container">
                    <!-- Sliders injected by JS -->
                </div>
            </div>
        </div>
    </div>

    <!-- Settings Modal -->
    <div id="settings-modal" class="modal-overlay">
        <div class="modal-content">
            <h2>Settings</h2>
            <div class="setting-group">
                <label>Frame Delay (ms)</label>
                <input type="number" id="set-frameDelay" value="100">
            </div>
            <div class="setting-group">
                <label>Walk Cycles</label>
                <input type="number" id="set-walkCycles" value="1">
            </div>
            <div class="setting-group">
                <label>Motor Current Delay (ms)</label>
                <input type="number" id="set-motorCurrentDelay" value="10">
            </div>
            <div class="setting-group">
                <label>Theme Color</label>
                <div class="theme-picker" id="theme-picker">
                    <div class="color-dot active" style="background: #ff8c42" onclick="setTheme('#ff8c42', this)"></div>
                    <div class="color-dot" style="background: #00e5ff" onclick="setTheme('#00e5ff', this)"></div>
                    <div class="color-dot" style="background: #00e676" onclick="setTheme('#00e676', this)"></div>
                    <div class="color-dot" style="background: #ff1744" onclick="setTheme('#ff1744', this)"></div>
                    <div class="color-dot" style="background: #d500f9" onclick="setTheme('#d500f9', this)"></div>
                    <div class="color-dot" style="background: #f50057" onclick="setTheme('#f50057', this)"></div>
                </div>
            </div>
            <div class="modal-actions">
                <button onclick="closeSettings()">Cancel</button>
                <button class="btn-primary" onclick="saveSettings()">Save</button>
            </div>
        </div>
    </div>

    <script>
        // System State
        let cmdQueue = 0;
        let motorsLocked = false;
        let lockTimeout = null;
        
        const motors = [
            { id: 1, name: "R1 - Right Front Hip" },
            { id: 2, name: "R2 - Right Front Knee" },
            { id: 3, name: "L1 - Left Front Hip" },
            { id: 4, name: "L2 - Left Front Knee" },
            { id: 5, name: "R4 - Right Rear Knee" },
            { id: 6, name: "R3 - Right Rear Hip" },
            { id: 7, name: "L3 - Left Rear Hip" },
            { id: 8, name: "L4 - Left Rear Knee" }
        ];

        let lastMotorUpdate = {};

        // Initialization
        function init() {
            const container = document.getElementById('motor-sliders-container');
            motors.forEach(m => {
                lastMotorUpdate[m.id] = 0;
                container.innerHTML += `
                    <div class="motor-item">
                        <div class="motor-header">
                            <span>${m.name}</span>
                            <span id="val-m${m.id}">90&deg;</span>
                        </div>
                        <input type="range" class="motor-slider" id="slider-m${m.id}" min="0" max="180" value="90" 
                            oninput="updateMotorUi(${m.id}, this.value)" 
                            onchange="sendMotor(${m.id}, this.value)">
                    </div>
                `;
            });

            // Load Theme
            const savedTheme = localStorage.getItem('quadTheme');
            if (savedTheme) {
                document.documentElement.style.setProperty('--accent', savedTheme);
                document.documentElement.style.setProperty('--accent-hover', adjustColor(savedTheme, 20));
                
                // Update dots
                document.querySelectorAll('.color-dot').forEach(dot => {
                    dot.classList.remove('active');
                    if (rgb2hex(dot.style.background) === savedTheme) dot.classList.add('active');
                });
            }
            
            // Set speed default UI based on actual mapping
            // actualSpeed = 330 - sliderValue
            // If actual = 100, sliderValue = 230
            document.getElementById('speedSlider').value = 230;
            updateSpeedUi(230);
        }

        // Helpers
        const queueUi = document.getElementById('queue-status');
        
        function updateQueue() {
            queueUi.textContent = `Command Queue: ${cmdQueue}/3`;
            if (cmdQueue >= 3) queueUi.classList.add('full');
            else queueUi.classList.remove('full');
        }

        function addQueue() {
            if (cmdQueue < 3) {
                cmdQueue++;
                updateQueue();
                setTimeout(() => {
                    if (cmdQueue > 0) cmdQueue--;
                    updateQueue();
                }, 1000);
                return true;
            }
            return false;
        }

        function clearQueue() {
            cmdQueue = 0;
            updateQueue();
        }

        function sendCmd(cmd) {
            if (!addQueue()) return;
            fetch(`/cmd?${cmd}`).catch(e => console.log('Cmd error', e));
        }

        // Movement
        let moveInterval = null;
        function startMove(dir) {
            if (moveInterval) clearInterval(moveInterval);
            sendCmd(`go=${dir}`);
            moveInterval = setInterval(() => {
                if(cmdQueue < 2) sendCmd(`go=${dir}`);
            }, 600);
        }
        function stopMove() {
            if (moveInterval) {
                clearInterval(moveInterval);
                moveInterval = null;
            }
            sendCmd('stop=1');
        }

        // Touch Wrappers
        function tStartMove(e, dir) { e.preventDefault(); startMove(dir); }
        function tStopMove(e) { e.preventDefault(); stopMove(); }
        function tStopAll(e) { e.preventDefault(); stopAll(); }

        function stopAll() {
            clearQueue();
            if (moveInterval) { clearInterval(moveInterval); moveInterval = null; }
            fetch('/cmd?stop=1').catch(e => {});
        }

        // Poses
        function pose(name) {
            if (!addQueue()) return;
            fetch(`/cmd?pose=${name}`).catch(e => {});
            
            // Lock motors
            motorsLocked = true;
            document.getElementById('motor-lock-msg').style.display = 'block';
            document.querySelectorAll('.motor-slider').forEach(el => el.classList.add('disabled'));
            
            if (lockTimeout) clearTimeout(lockTimeout);
            lockTimeout = setTimeout(() => {
                motorsLocked = false;
                document.getElementById('motor-lock-msg').style.display = 'none';
                document.querySelectorAll('.motor-slider').forEach(el => el.classList.remove('disabled'));
            }, 3000);
        }

        // Speed
        // Mapping: speedSlider ranges 30 to 300. We want right (300) to be fast (30ms).
        // actualSpeed = 330 - sliderValue
        function calcSpeed(val) {
            return 330 - parseInt(val);
        }
        function updateSpeedUi(val) {
            document.getElementById('speedVal').textContent = calcSpeed(val);
        }
        function sendSpeed(val) {
            let actualMs = calcSpeed(val);
            sendCmd(`speed=${actualMs}`);
        }

        // Motors
        function updateMotorUi(id, val) {
            if (motorsLocked) return;
            document.getElementById(`val-m${id}`).innerHTML = `${val}&deg;`;
            
            const now = Date.now();
            if (now - lastMotorUpdate[id] > 80) {
                lastMotorUpdate[id] = now;
                sendMotorFast(id, val);
            }
        }
        
        function sendMotorFast(id, val) {
            if (motorsLocked || cmdQueue >= 3) return;
            fetch(`/cmd?motor=${id}&value=${val}`).catch(e => {});
        }

        function sendMotor(id, val) {
            if (motorsLocked) {
                // reset visual if locked
                document.getElementById(`slider-m${id}`).value = 90;
                document.getElementById(`val-m${id}`).innerHTML = `90&deg;`;
                return;
            }
            sendCmd(`motor=${id}&value=${val}`);
        }

        // Settings Modal
        const modal = document.getElementById('settings-modal');
        function openSettings() {
            modal.classList.add('active');
            fetch('/getSettings')
                .then(res => res.json())
                .then(data => {
                    if(data.frameDelay) document.getElementById('set-frameDelay').value = data.frameDelay;
                    if(data.walkCycles) document.getElementById('set-walkCycles').value = data.walkCycles;
                    if(data.motorCurrentDelay) document.getElementById('set-motorCurrentDelay').value = data.motorCurrentDelay;
                }).catch(e => console.log('Failed to get settings'));
        }
        function closeSettings() {
            modal.classList.remove('active');
        }
        function saveSettings() {
            const fd = document.getElementById('set-frameDelay').value;
            const wc = document.getElementById('set-walkCycles').value;
            const mcd = document.getElementById('set-motorCurrentDelay').value;
            
            fetch(`/setSettings?frameDelay=${fd}&walkCycles=${wc}&motorCurrentDelay=${mcd}`)
                .then(() => closeSettings())
                .catch(e => { alert('Failed to save settings'); closeSettings(); });
        }

        // Theme
        function setTheme(color, el) {
            document.documentElement.style.setProperty('--accent', color);
            document.documentElement.style.setProperty('--accent-hover', adjustColor(color, 20));
            localStorage.setItem('quadTheme', color);
            
            document.querySelectorAll('.color-dot').forEach(dot => dot.classList.remove('active'));
            if(el) el.classList.add('active');
        }

        function adjustColor(color, amount) {
            return '#' + color.replace(/^#/, '').replace(/../g, color => 
                ('0'+Math.min(255, Math.max(0, parseInt(color, 16) + amount)).toString(16)).substr(-2));
        }

        function rgb2hex(rgb) {
            if (rgb.search("rgb") == -1) return rgb;
            rgb = rgb.match(/^rgba?\((\d+),\s*(\d+),\s*(\d+)(?:,\s*(\d+))?\)$/);
            function hex(x) { return ("0" + parseInt(x).toString(16)).slice(-2); }
            return "#" + hex(rgb[1]) + hex(rgb[2]) + hex(rgb[3]);
        }

        window.onload = init;
    </script>
</body>
</html>
)rawliteral";
