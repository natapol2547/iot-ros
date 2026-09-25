// IoT Robot console. Talks to web_controller over one WebSocket (/ws) and shows the
// camera as MJPEG (/stream.mjpg). The server enforces every limit; this page only asks.
"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const app = $("app");
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  // Phones and tablets: no keyboard hints
  const touchScreen = window.matchMedia("(pointer: coarse)");

  const SEND_INTERVAL_MS = 50; // drive commands while a control is held (20 Hz)
  const HOLD_TO_RELEASE_MS = 1000;
  const ESTOP_CONFIRM_MS = 1500; // a local E-stop press waits this long for the robot to agree
  const STATE_STALE_MS = 1500; // no telemetry for this long: show sensors as no data
  const VIDEO_STALE_S = 2.0;
  const BLANK_IMAGE = "data:image/gif;base64,R0lGODlhAQABAAAAACw=";
  const MODE_NAMES = { drive: "Drive", ball: "Follow ball", person: "Follow person" };

  const state = {
    ws: null,
    connected: false,
    retries: 0,
    uiVersion: null,
    hello: null,
    t: null,
    lastState: 0,
    estop: false,
    estopPending: null,
    mode: "drive",
    modePending: null,
    driver: "none",
    speed: 0.5,
  };

  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
  const cm = (m) => Math.round(m * 100);
  const deg = (rad) => Math.round((rad * 180) / Math.PI);
  // Two decimals without "-0.00" for tiny negative odometry noise
  const fixed2 = (v) => (Math.abs(v) < 0.005 ? 0 : v).toFixed(2);

  // Connection ----------------------------------------------------------------------

  function connect() {
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${scheme}://${location.host}/ws`);
    state.ws = ws;
    ws.onmessage = (event) => {
      let msg;
      try {
        msg = JSON.parse(event.data);
      } catch {
        return;
      }
      if (msg.type === "hello") onHello(msg);
      else if (msg.type === "state") onState(msg);
      else if (msg.type === "notice") onNotice(msg);
      else if (msg.type === "result") onResult(msg);
    };
    ws.onclose = () => {
      if (state.ws !== ws) return;
      state.ws = null;
      state.connected = false;
      stopDriving();
      stopVideo();
      const delay = Math.min(5000, 500 * 2 ** state.retries);
      state.retries += 1;
      setConnection("down");
      setTimeout(connect, delay);
      refreshControls();
    };
    ws.onerror = () => ws.close();
  }

  function send(msg) {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
      state.ws.send(JSON.stringify(msg));
      return true;
    }
    return false;
  }

  function setConnection(status) {
    app.dataset.conn = status;
    const pill = $("conn");
    pill.dataset.state = status;
    $("conn-text").textContent =
      status === "live" ? "Live" : status === "down" ? "Offline" : "Connecting";
    $("offline-text").textContent = "Reconnecting to the robot";
  }

  function onHello(msg) {
    // A STOP pressed while offline, or lost with the connection, is sent now. Stopping
    // late is better than not stopping; telemetry then confirms it
    const pending = state.estopPending;
    if (pending && pending.value && send({ type: "estop", engaged: true })) {
      pending.until = performance.now() + ESTOP_CONFIRM_MS;
    }
    // The robot restarted with a different page: load it
    if (state.uiVersion && state.uiVersion !== msg.ui_version) {
      location.reload();
      return;
    }
    state.uiVersion = msg.ui_version;
    state.hello = msg;
    state.connected = true;
    state.retries = 0;
    state.lastState = performance.now();
    setConnection("live");
    updateSpeed();
    drawThresholds();
    startVideo();
    refreshControls();
  }

  // Telemetry -----------------------------------------------------------------------

  function onState(t) {
    const previous = state.t;
    state.t = t;
    state.lastState = performance.now();

    // Keep a local e-stop press on screen until the server confirms it
    const pending = state.estopPending;
    if (pending && performance.now() < pending.until && t.estop !== pending.value) {
      // waiting for the server
    } else {
      if (pending && t.estop !== pending.value) {
        // Never let an unconfirmed STOP quietly turn back into "Stop"
        toast(pending.value
          ? "The robot did not confirm the stop. Press STOP again or use the robot's switch."
          : "The robot did not confirm the release.");
      } else if (pending === null && previous && previous.estop !== t.estop) {
        toast(t.estop ? "Someone engaged the E-stop" : "Someone released the E-stop");
      }
      state.estopPending = null;
      setEstop(t.estop);
    }

    state.mode = t.mode;
    if (state.modePending &&
        (t.mode === state.modePending.mode || performance.now() > state.modePending.until)) {
      state.modePending = null;
    }
    state.driver = t.driver;

    renderStatus(t);
    renderModes(t);
    renderVideo(t);
    renderSonar(t);
    renderGizmo(t);
    renderPerson(t);
    refreshControls();
  }

  function renderStatus(t) {
    $("viewers-n").textContent = t.clients;
    const base = $("base");
    base.dataset.state = t.odom ? "online" : "offline";
    base.lastChild.textContent = t.odom ? "Base" : "Base offline";

    const battery = $("battery");
    if (t.battery) {
      const pct = t.battery.percent;
      battery.dataset.level = pct < 10 ? "critical" : pct < 30 ? "low" : "ok";
      $("battery-text").textContent = `${t.battery.voltage.toFixed(1)} V, ${pct}%`;
      $("battery-fill").style.setProperty("--pct", pct / 100);
      battery.title = "Battery voltage reported by the STM32";
    } else {
      battery.dataset.level = "none";
      $("battery-text").textContent = "--";
      battery.title = "No battery reading. The STM32 has not reported a voltage.";
      $("battery-fill").style.setProperty("--pct", 0);
    }

    $("speed").textContent = t.odom ? fixed2(t.odom.linear) : "--";
    $("turn").textContent = t.odom ? fixed2(t.odom.angular) : "--";

    const tag = $("driver-tag");
    tag.dataset.driver = t.driver;
    tag.textContent = { you: "You are driving", other: "Someone else is driving",
      none: "Controls free" }[t.driver];
    $("drive-note").textContent = { you: "You have the controls",
      other: "Wait for your turn", none: "Hold and drag" }[t.driver];

    const guard = $("guard");
    if (t.guard.obstacle) {
      guard.hidden = false;
      guard.textContent = t.guard.distance === null
        ? `No data from the ${t.guard.side} sensor. Forward is blocked.`
        : `Obstacle ${cm(t.guard.distance)} cm on the ${t.guard.side}. Forward is blocked.`;
      guard.classList.toggle("blocking", t.guard.blocking);
    } else {
      guard.hidden = true;
    }
    $("joystick").dataset.blocked = String(t.guard.obstacle);
  }

  function modeText(t) {
    if (t.mode_status === "stopping") return "Stopping the follower";
    if (t.mode === "drive") {
      if (t.external_follower) {
        return "Manual control. A follower started outside this page is also running; " +
          "the joystick overrides it.";
      }
      return touchScreen.matches
        ? "Manual control. Drag the joystick to drive."
        : "Manual control. Drag the joystick or use the keyboard.";
    }
    const what = t.mode === "ball" ? "ball" : "person";
    if (t.mode_status === "starting") {
      return t.mode === "person"
        ? "Starting the person follower. Loading the detection models."
        : "Starting the ball follower";
    }
    if (t.mode_status === "failed") {
      return `The ${what} follower stopped: ${t.mode_detail || "unknown error"}`;
    }
    if (t.mode === "ball") return "Following the yellow ball. The joystick overrides it.";
    // The detector's own status is shown in the person panel
    return "Following the enrolled person. The joystick overrides it.";
  }

  function renderModes(t) {
    app.dataset.mode = t.mode;
    $("mode-tag").textContent = MODE_NAMES[t.mode];
    for (const button of document.querySelectorAll(".segmented button")) {
      const mode = button.dataset.mode;
      button.setAttribute("aria-checked", String(mode === t.mode));
      button.toggleAttribute("data-pending",
        Boolean(state.modePending && state.modePending.mode === mode && mode !== t.mode));
      button.disabled = t.mode_status === "stopping";
    }
    const status = $("mode-status");
    status.dataset.status = t.mode_status;
    $("mode-status-text").textContent = modeText(t);
  }

  // Video ---------------------------------------------------------------------------

  const video = $("video");
  let videoRetry = null;

  function startVideo() {
    clearTimeout(videoRetry);
    if (document.hidden || !state.connected) return;
    video.src = `/stream.mjpg?t=${Date.now()}`;
  }

  function stopVideo() {
    clearTimeout(videoRetry);
    // Replacing the source closes the stream connection
    video.src = BLANK_IMAGE;
  }

  video.addEventListener("error", () => {
    clearTimeout(videoRetry);
    videoRetry = setTimeout(startVideo, 2000);
  });

  function renderVideo(t) {
    const live = t.video_age !== null && t.video_age < VIDEO_STALE_S;
    app.dataset.video = live ? "live" : "waiting";
    if (live) return;
    let title = "Waiting for video";
    let sub = "The camera stream starts in a moment";
    if (t.mode !== "drive" && t.mode_status === "starting") {
      title = `Starting the ${t.mode} detector`;
      sub = "Its annotated view appears here once it runs";
    } else if (t.mode !== "drive") {
      sub = "Waiting for the detector's annotated view";
    }
    $("video-empty-title").textContent = title;
    $("video-empty-sub").textContent = sub;
  }

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      stopVideo();
      stopDriving();
    } else {
      startVideo();
    }
  });

  // Ultrasonic sensors ------------------------------------------------------------------

  const LEVEL_TEXT = { green: "Safe", yellow: "Caution", red: "Too close", none: "No data" };
  // Sketch of the robot from above: sensors on the front chamfers, facing 45° outwards.
  // Distance maps to the cone with a square root so the interesting first metre gets room
  const MAP = { length: 70, span: 2.0, half: (12 * Math.PI) / 180 };
  const SENSORS = {
    left: { x: 67, y: 79, a: (-3 * Math.PI) / 4 },
    right: { x: 93, y: 79, a: -Math.PI / 4 },
  };

  function makeSide(name) {
    const root = $(`us-${name}`);
    const lamps = {};
    for (const level of ["red", "yellow", "green"]) {
      lamps[level] = root.querySelector(`.lamp.${level}`);
    }
    const side = {
      name,
      root,
      lamps,
      value: root.querySelector(".sonar-value b"),
      unit: root.querySelector(".sonar-value .unit"),
      label: root.querySelector(".sonar-state"),
      gauge: root.querySelector(".gauge"),
      mark: root.querySelector(".gauge-mark"),
      bg: $(`cone-${name}-bg`),
      fill: $(`cone-${name}-fill`),
      echo: $(`cone-${name}-echo`),
      stop: $(`cone-${name}-stop`),
      level: null,
      period: 1,
      phase: 0,
      key: null,
    };
    side.bg.setAttribute("d", wedge(SENSORS[name], MAP.length));
    return side;
  }

  function point(s, r, a) {
    return `${(s.x + r * Math.cos(a)).toFixed(2)} ${(s.y + r * Math.sin(a)).toFixed(2)}`;
  }

  function wedge(s, r) {
    return `M${s.x} ${s.y} L${point(s, r, s.a - MAP.half)} ` +
      `A${r} ${r} 0 0 1 ${point(s, r, s.a + MAP.half)} Z`;
  }

  function arc(s, r) {
    return `M${point(s, r, s.a - MAP.half)} A${r} ${r} 0 0 1 ${point(s, r, s.a + MAP.half)}`;
  }

  function mapRadius(distance) {
    return Math.max(3, MAP.length * Math.sqrt(clamp(distance, 0, MAP.span) / MAP.span));
  }

  // Thresholds come from the server: the guard's stop line on the map, the zones on the gauge
  function drawThresholds() {
    const u = state.hello.ultrasonic;
    const share = (d) => `${(clamp(d / MAP.span, 0, 1) * 100).toFixed(1)}%`;
    for (const side of Object.values(sides)) {
      side.stop.setAttribute("d", u.stop > 0 ? arc(SENSORS[side.name], mapRadius(u.stop)) : "");
      side.gauge.style.setProperty("--warn", share(u.warn));
      side.gauge.style.setProperty("--danger", share(u.danger));
    }
  }

  const sides = { left: makeSide("left"), right: makeSide("right") };

  function setSide(side, reading) {
    const level = reading ? reading.level : "none";
    const distance = reading ? reading.distance : null;
    const key = `${level}:${distance}`;
    if (key === side.key) return;
    side.key = key;

    if (level !== side.level) {
      for (const lamp of Object.values(side.lamps)) {
        lamp.classList.remove("lit", "dim");
        lamp.style.opacity = "";
      }
      if (side.lamps[level]) side.lamps[level].classList.add("lit");
      side.level = level;
      side.root.dataset.level = level;
    }
    side.period = reading && reading.period ? reading.period : 1;

    const s = SENSORS[side.name];
    side.bg.dataset.level = level;
    side.fill.dataset.level = level;
    side.echo.dataset.level = level;
    if (distance === "clear") {
      side.value.textContent = "Clear";
      side.unit.hidden = true;
      side.label.textContent = "No echo";
      side.mark.style.setProperty("--at", "100%");
      side.fill.setAttribute("d", wedge(s, MAP.length));
      side.echo.setAttribute("d", "");
    } else if (typeof distance === "number") {
      side.value.textContent = cm(distance);
      side.unit.hidden = false;
      side.label.textContent = LEVEL_TEXT[level];
      side.mark.style.setProperty("--at", `${(clamp(distance / MAP.span, 0, 1) * 100).toFixed(1)}%`);
      const r = mapRadius(distance);
      side.fill.setAttribute("d", wedge(s, r));
      side.echo.setAttribute("d", distance <= MAP.span ? arc(s, r) : "");
    } else {
      side.value.textContent = "--";
      side.unit.hidden = true;
      side.label.textContent = LEVEL_TEXT.none;
      side.fill.setAttribute("d", "");
      side.echo.setAttribute("d", "");
    }
  }

  function renderSonar(t) {
    for (const side of Object.values(sides)) setSide(side, t.ultrasonic[side.name]);
  }

  // Blink like a parking sensor: the server sends the period, faster when closer
  let lastFrame = performance.now();
  function animate(now) {
    const dt = Math.min(0.1, (now - lastFrame) / 1000);
    lastFrame = now;
    for (const side of Object.values(sides)) {
      const lamp = side.lamps[side.level];
      if (!lamp) continue;
      side.phase = (side.phase + dt / side.period) % 1;
      if (reducedMotion.matches) {
        // Keep the colour and replace the blink with a slow, shallow pulse
        const wave = 0.5 + 0.5 * Math.cos(2 * Math.PI * side.phase);
        lamp.style.opacity = (0.7 + 0.3 * wave).toFixed(3);
        lamp.classList.remove("dim");
      } else {
        lamp.style.opacity = "";
        lamp.classList.toggle("dim", side.phase >= 0.5);
      }
    }
    requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);

  // Nothing from the robot for a while: stop pretending the last readings are current
  setInterval(() => {
    if (!state.connected || performance.now() - state.lastState > STATE_STALE_MS) {
      for (const side of Object.values(sides)) setSide(side, null);
      app.dataset.video = "waiting";
    }
  }, 250);

  // Driving ---------------------------------------------------------------------------

  const joystick = $("joystick");
  const knob = $("joy-knob");
  const joy = { pointer: null, x: 0, y: 0 };
  const keys = new Set();
  const DRIVE_KEYS = {
    KeyW: [1, 0], ArrowUp: [1, 0], KeyS: [-1, 0], ArrowDown: [-1, 0],
    KeyA: [0, 1], ArrowLeft: [0, 1], KeyD: [0, -1], ArrowRight: [0, -1],
  };
  let sending = false;

  function canControl() {
    return state.connected && !state.estop && state.driver !== "other";
  }

  function lockReason() {
    if (!state.connected) return "Not connected";
    if (state.estop) return "E-stop engaged";
    if (state.driver === "other") return "Someone else is driving";
    return "";
  }

  // A small dead zone, then a gentle curve for fine control near the centre
  function shape(v) {
    const a = Math.abs(v);
    if (a < 0.08) return 0;
    return Math.sign(v) * ((a - 0.08) / 0.92) ** 1.5;
  }

  function moveJoystick(event) {
    const rect = joystick.getBoundingClientRect();
    // The knob stays inside the ring
    const travel = rect.width / 2 - knob.offsetWidth / 2;
    let dx = event.clientX - (rect.left + rect.width / 2);
    let dy = event.clientY - (rect.top + rect.height / 2);
    const length = Math.hypot(dx, dy);
    if (length > travel) {
      dx *= travel / length;
      dy *= travel / length;
    }
    knob.style.setProperty("--jx", `${dx.toFixed(1)}px`);
    knob.style.setProperty("--jy", `${dy.toFixed(1)}px`);
    joy.x = dx / travel;
    joy.y = -dy / travel;
  }

  function endJoystick() {
    joy.pointer = null;
    joy.x = 0;
    joy.y = 0;
    knob.style.setProperty("--jx", "0px");
    knob.style.setProperty("--jy", "0px");
    refreshControls();
  }

  joystick.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (joy.pointer !== null) return;
    if (!canControl()) {
      toast(lockReason());
      return;
    }
    joy.pointer = event.pointerId;
    joystick.setPointerCapture(event.pointerId);
    joystick.dataset.state = "active";
    moveJoystick(event);
  });
  joystick.addEventListener("pointermove", (event) => {
    if (event.pointerId === joy.pointer) moveJoystick(event);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    joystick.addEventListener(type, (event) => {
      if (event.pointerId === joy.pointer) endJoystick();
    });
  }

  function keyboardAxes() {
    let linear = 0;
    let angular = 0;
    for (const code of keys) {
      linear += DRIVE_KEYS[code][0];
      angular += DRIVE_KEYS[code][1];
    }
    return { linear: clamp(linear, -1, 1), angular: clamp(angular, -1, 1) };
  }

  function stopDriving() {
    keys.clear();
    if (joy.pointer !== null) endJoystick();
    if (sending) {
      send({ type: "release" });
      sending = false;
    }
  }

  // Commands stream while a control is held; the server stops the robot when they stop
  setInterval(() => {
    let axes = null;
    if (joy.pointer !== null) axes = { linear: shape(joy.y), angular: -shape(joy.x) };
    else if (keys.size) axes = keyboardAxes();
    if (axes && canControl()) {
      sending = send({ type: "drive", ...axes, speed: state.speed });
    } else if (sending) {
      send({ type: "release" });
      sending = false;
    }
  }, SEND_INTERVAL_MS);

  window.addEventListener("blur", stopDriving);

  const speedRange = $("speed-range");
  const turbo = $("turbo");
  // Turbo lets the slider go past 100 %, up to the server's limits.turbo. It always
  // starts off, and the speed saved for the next visit never exceeds 100 %
  let turboOn = false;
  function updateSpeed() {
    const limits = state.hello ? state.hello.limits : { linear: 0.4, turbo: 1 };
    const turboMax = Math.round((limits.turbo || 1) * 100);
    turbo.hidden = turboMax <= 100;
    if (turbo.hidden) turboOn = false;
    const max = turboOn ? turboMax : 100;
    speedRange.max = max;
    if (Number(speedRange.value) > max) speedRange.value = max;
    turbo.setAttribute("aria-pressed", String(turboOn));
    speedRange.parentElement.dataset.turbo = String(turboOn);
    const min = Number(speedRange.min);
    state.speed = Number(speedRange.value) / 100;
    speedRange.style.setProperty("--fill", `${((speedRange.value - min) / (max - min)) * 100}%`);
    $("speed-pct").textContent = `${speedRange.value}%`;
    $("speed-max").textContent = `up to ${(limits.linear * state.speed).toFixed(2)} m/s`;
  }
  function saveSpeed() {
    localStorage.setItem("speed", Math.min(Number(speedRange.value), 100));
  }
  speedRange.addEventListener("input", () => {
    updateSpeed();
    saveSpeed();
  });
  // Let go of the slider so the arrow keys drive again
  speedRange.addEventListener("change", () => speedRange.blur());
  turbo.addEventListener("click", () => {
    turboOn = !turboOn;
    updateSpeed();
    saveSpeed();
    turbo.blur();
  });
  const savedSpeed = Number(localStorage.getItem("speed"));
  if (savedSpeed >= 10 && savedSpeed <= 100) speedRange.value = savedSpeed;
  updateSpeed();

  // E-stop ------------------------------------------------------------------------------

  const estop = $("estop");
  let holdStart = null;

  function setEstop(on) {
    if (on === state.estop && app.dataset.estop) return;
    state.estop = on;
    app.dataset.estop = on ? "on" : "off";
    estop.setAttribute("aria-pressed", String(on));
    $("estop-title").textContent = on ? "E-stop engaged" : "Stop";
    $("estop-sub").textContent = on ? "Press and hold 1 s to release"
      : touchScreen.matches ? "Tap to stop the robot" : "Click or press Space to stop the robot";
    if (on) stopDriving();
    refreshControls();
  }

  function engage() {
    // Stays pending until telemetry confirms it; onHello sends it again after a reconnect
    state.estopPending = { value: true, until: performance.now() + ESTOP_CONFIRM_MS };
    if (!send({ type: "estop", engaged: true })) {
      toast("Not connected. The stop is sent when the robot is back. Use the robot's switch.");
      return;
    }
    setEstop(true);
  }

  function release() {
    if (!send({ type: "estop", engaged: false })) {
      toast("Not connected");
      return;
    }
    // A lost release is not repeated after a reconnect: releasing must stay deliberate
    state.estopPending = { value: false, until: performance.now() + ESTOP_CONFIRM_MS };
    setEstop(false);
    toast("E-stop released");
  }

  function holdTick(now) {
    if (holdStart === null) return;
    const progress = Math.min(1, (now - holdStart) / HOLD_TO_RELEASE_MS);
    estop.style.setProperty("--hold", progress.toFixed(3));
    if (progress >= 1) {
      cancelHold();
      release();
    } else {
      requestAnimationFrame(holdTick);
    }
  }

  function startHold() {
    if (holdStart !== null) return;
    holdStart = performance.now();
    requestAnimationFrame(holdTick);
  }

  function cancelHold() {
    holdStart = null;
    estop.style.setProperty("--hold", "0");
  }

  estop.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    event.preventDefault();
    // Stopping happens on touch-down, not on release, to save the tap's reaction time
    if (!state.estop) {
      engage();
      return;
    }
    estop.setPointerCapture(event.pointerId);
    startHold();
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    estop.addEventListener(type, cancelHold);
  }
  estop.addEventListener("contextmenu", (event) => event.preventDefault());
  estop.addEventListener("keydown", (event) => {
    if (event.key !== "Enter" && event.key !== " ") return;
    event.preventDefault();
    event.stopPropagation();
    if (event.repeat) return;
    if (!state.estop) engage();
    else startHold();
  });
  estop.addEventListener("keyup", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      cancelHold();
    }
  });

  // Keyboard ----------------------------------------------------------------------------

  window.addEventListener("keydown", (event) => {
    if (event.ctrlKey || event.metaKey || event.altKey) return;
    if (event.code === "Space" || event.code === "Escape") {
      event.preventDefault();
      if (!state.estop) engage();
      return;
    }
    const onSlider = document.activeElement === speedRange;
    if (DRIVE_KEYS[event.code] && !(onSlider && event.code.startsWith("Arrow"))) {
      event.preventDefault();
      if (!event.repeat && !canControl()) toast(lockReason());
      keys.add(event.code);
    }
  });
  window.addEventListener("keyup", (event) => {
    if (event.code === "Space") event.preventDefault();
    keys.delete(event.code);
  });

  // Modes -------------------------------------------------------------------------------

  for (const button of document.querySelectorAll(".segmented button")) {
    button.addEventListener("click", () => {
      const mode = button.dataset.mode;
      const t = state.t;
      if (!state.connected || !t) return;
      if (mode === t.mode && t.mode_status !== "failed") return;
      state.modePending = { mode, until: performance.now() + 15000 };
      send({ type: "mode", mode });
      renderModes(t);
    });
  }

  function onNotice(msg) {
    state.modePending = null;
    if (state.t) renderModes(state.t);
    toast(msg.text);
  }

  // Camera aim --------------------------------------------------------------------------

  const pad = $("gizmo-pad");
  const dot = $("gizmo-dot");
  const PAD_INSET = 14;
  let padPointer = null;
  let padAim = null; // where this page is aiming while dragging
  let padSent = 0;
  let padTimer = null;

  function placeDot(yaw, pitch) {
    const g = state.hello && state.hello.gizmo;
    if (!g) return;
    const fx = clamp(0.5 - yaw / (2 * g.yaw), 0, 1);
    const fy = clamp((pitch - g.pitch_min) / (g.pitch_max - g.pitch_min || 1), 0, 1);
    dot.style.left = `calc(${PAD_INSET}px + (100% - ${2 * PAD_INSET}px) * ${fx.toFixed(4)})`;
    dot.style.top = `calc(${PAD_INSET}px + (100% - ${2 * PAD_INSET}px) * ${fy.toFixed(4)})`;
    const pan = deg(yaw);
    const tilt = deg(-pitch);
    $("gizmo-readout").textContent =
      `Pan ${Math.abs(pan)}°${pan > 0 ? " left" : pan < 0 ? " right" : ""}, ` +
      (tilt > 0 ? `tilt ${tilt}° up` : "level");
  }

  function sendAim() {
    clearTimeout(padTimer);
    padTimer = null;
    if (!padAim) return;
    padSent = performance.now();
    send({ type: "gizmo", yaw: padAim.yaw, pitch: padAim.pitch });
  }

  function aimAt(event) {
    const g = state.hello.gizmo;
    const rect = pad.getBoundingClientRect();
    const fx = clamp((event.clientX - rect.left - PAD_INSET) / (rect.width - 2 * PAD_INSET), 0, 1);
    const fy = clamp((event.clientY - rect.top - PAD_INSET) / (rect.height - 2 * PAD_INSET), 0, 1);
    // Left on the pad looks left (positive yaw); the top looks up (negative pitch)
    padAim = { yaw: (0.5 - fx) * 2 * g.yaw, pitch: g.pitch_min + fy * (g.pitch_max - g.pitch_min) };
    placeDot(padAim.yaw, padAim.pitch);
    // At most 10 aims a second, always ending with the latest
    const wait = 100 - (performance.now() - padSent);
    if (wait <= 0) sendAim();
    else if (padTimer === null) padTimer = setTimeout(sendAim, wait);
  }

  pad.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (padPointer !== null || !state.hello) return;
    if (!canControl()) {
      toast(lockReason());
      return;
    }
    padPointer = event.pointerId;
    pad.setPointerCapture(event.pointerId);
    aimAt(event);
  });
  pad.addEventListener("pointermove", (event) => {
    if (event.pointerId === padPointer) aimAt(event);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    pad.addEventListener(type, (event) => {
      if (event.pointerId !== padPointer) return;
      padPointer = null;
      sendAim();
      // Show the robot's own value again once it has caught up
      setTimeout(() => { if (padPointer === null) padAim = null; }, 400);
    });
  }

  $("gizmo-center").addEventListener("click", () => {
    if (!canControl()) {
      toast(lockReason());
      return;
    }
    padAim = { yaw: 0, pitch: 0 };
    placeDot(0, 0);
    sendAim();
    setTimeout(() => { if (padPointer === null) padAim = null; }, 400);
  });

  function renderGizmo(t) {
    if (!padAim) placeDot(t.gizmo.yaw, t.gizmo.pitch);
  }

  // Person following --------------------------------------------------------------------

  function renderPerson(t) {
    const el = $("person-status");
    let text = t.person_status;
    let kind = "idle";
    if (!text) {
      text = t.mode_status === "starting" ? "Detector starting" : "No word from the detector yet";
    } else if (text.startsWith("Enrolling")) {
      kind = "enrolling";
    } else if (text === "Following") {
      kind = "following";
    } else {
      kind = "waiting";
    }
    el.textContent = text;
    el.dataset.state = kind;
    const ready = t.mode === "person" && t.mode_status === "running";
    $("enroll").disabled = !ready;
    $("forget").disabled = !ready;
  }

  function personAction(action, text) {
    if (!canControl()) {
      toast(lockReason());
      return;
    }
    if (send({ type: "person", action })) setResult(text, null);
  }

  function setResult(text, ok) {
    const el = $("person-result");
    el.textContent = text;
    if (ok === null) delete el.dataset.ok;
    else el.dataset.ok = String(ok);
  }

  function onResult(msg) {
    setResult(msg.message, msg.success);
  }

  $("enroll").addEventListener("click", () => personAction("enroll", "Enrolling"));
  $("forget").addEventListener("click", () => personAction("forget", "Forgetting"));

  // Shared ------------------------------------------------------------------------------

  function refreshControls() {
    if (joy.pointer === null) {
      joystick.dataset.state = canControl() ? "idle" : "locked";
      $("joy-lock").textContent = lockReason();
    }
    $("gizmo-pad").closest(".gizmo").dataset.locked = String(!canControl());
  }

  let toastTimer = null;
  function toast(text) {
    if (!text) return;
    const el = $("toast");
    el.textContent = text;
    el.dataset.show = "1";
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { el.dataset.show = ""; }, 3200);
  }

  setEstop(false);
  setConnection("connecting");
  connect();
})();
