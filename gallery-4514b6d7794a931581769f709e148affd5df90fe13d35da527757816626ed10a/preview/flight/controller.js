const codes = ["Space", "KeyF", "KeyW", "KeyA", "KeyS", "KeyD"];
const initialYaw = Math.atan2(-.71, .64);
const initialPitch = Math.atan2(.29, Math.hypot(.64, .71));
const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

export async function install({ viewer, root, read, base, output, help, mode, loadAssets }) {
  const [{ inputs }, { PythonLogic }, config] = await Promise.all([
    import(new URL("./src/vehicle-host.js", location.href)),
    import(new URL("./src/python-logic.js", location.href)),
    read("flight.json"),
    loadAssets(),
  ]);
  const canvas = viewer.canvas;
  const Vec = viewer.camera.position.constructor;
  function part(name) {
    const node = root.getObjectByName(name);
    if (!node || node.isMesh) throw Error("Missing flight part " + name);
    if (node.matrixAutoUpdate) node.updateMatrix();
    const position = node.position.clone(), rotation = node.quaternion.clone();
    const scale = node.scale.clone();
    node.matrix.decompose(position, rotation, scale);
    return { node, position, rotation, scale };
  }
  const body = part("Helicopter"), main = part("Main_rotor"), tail = part("Tail_rotor");
  const doors = config.doors.map(part);
  const inspectedDoorMatrices = new Map();
  const pitchAxis = new Vec(0, 0, 1), rollAxis = new Vec(1, 0, 0);
  const up = new Vec(0, 1, 0), tailAxis = new Vec(...config.tailAxis).normalize();
  const leanCenter = new Vec(-1.4, 2.75, 0);
  const offset = new Vec(), target = new Vec(), eye = new Vec();
  const q = body.rotation.clone(), delta = q.clone();
  const rest = { x: 0, y: 0, z: 0, vx: 0, vz: 0, pitch: 0, roll: 0, mainAngle: 0, tailAngle: 0, power: 0 };
  const abort = new AbortController();
  const { signal } = abort;
  let logic, generation = 0, state = rest, inspectionPose, failed = false;
  let yaw = initialYaw, pitch = initialPitch, zoom = 39, telemetryTime = 0;
  const previousKeys = new Set();
  const startup = new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(Error("Flight runtime startup timed out.")), 115000);
    logic = new PythonLogic({
      module: new URL("flight_runtime.py", base).href,
      dependencies: { "flight_core.py": new URL("flight_core.py", base).href },
    }, snapshot => {
      clearTimeout(timeout);
      if (snapshot.generation === generation) state = snapshot.state;
      canvas.dataset.flightPython = "ready";
      resolve();
    }, error => {
      clearTimeout(timeout);
      failed = true;
      mode.set(false);
      output.textContent = "Flight controls unavailable. " + error;
      reject(Error(error));
    });
  });
  try {
    await startup;
  } catch (error) {
    logic.dispose();
    throw error;
  }

  const svgNS = "http://www.w3.org/2000/svg";
  const overlay = document.createElementNS(svgNS, "svg");
  overlay.setAttribute("aria-hidden", "true");
  overlay.style.cssText = "position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:3";
  viewer.container.append(overlay);
  const pulses = [];
  const projected = new Vec();
  function emitPulse() {
    const origin = body.node.localToWorld(new Vec(8.2, 2.25, .17));
    const direction = body.node.localToWorld(new Vec(9.2, 2.25, .17)).sub(origin).normalize();
    const line = document.createElementNS(svgNS, "line");
    line.setAttribute("stroke", "#ffefd1");
    line.setAttribute("stroke-width", "3");
    line.setAttribute("stroke-linecap", "round");
    line.style.filter = "drop-shadow(0 0 5px #ff8a43)";
    overlay.append(line);
    pulses.push({ origin, direction, line, start: performance.now() });
    if (pulses.length > 16) pulses.shift().line.remove();
  }
  function projectPulse(pulse, distance, x, y) {
    projected.copy(pulse.origin).addScaledVector(pulse.direction, distance).project(viewer.camera);
    pulse.line.setAttribute(x, ((projected.x + 1) * 50) + "%");
    pulse.line.setAttribute(y, ((1 - projected.y) * 50) + "%");
    return projected.z >= -1 && projected.z <= 1;
  }
  function updatePulses() {
    if (!pulses.length) return;
    const now = performance.now();
    while (pulses.length && now - pulses[0].start >= 580) pulses.shift().line.remove();
    viewer.camera.updateMatrixWorld();
    for (const pulse of pulses) {
      const age = (now - pulse.start) / 1000, distance = age * 37;
      const start = projectPulse(pulse, Math.max(0, distance - 4.5), "x1", "y1");
      const end = projectPulse(pulse, distance, "x2", "y2");
      pulse.line.setAttribute("opacity", start && end ? 1 - age / .58 : 0);
    }
  }
  help.textContent = "Space: Take off / climb · F: Descend · WASD: Fly · Mouse: Orbit (right drag if unlocked) · Wheel: Zoom · Left click: Light pulse · Esc: Inspect";
  const input = inputs(mode, canvas, {
    codes,
    send() {
      for (const key of codes) {
        const held = input.keys.has(key);
        if (held === previousKeys.has(key)) continue;
        logic.event(held ? "press" : "release", key);
        if (held) previousKeys.add(key);
        else previousKeys.delete(key);
      }
    },
    release() {
      previousKeys.clear();
      logic.event("clear");
      pulses.length = 0;
      overlay.replaceChildren();
    },
    mouse(event) {
      if (document.pointerLockElement !== canvas && !(event.buttons & 2)) return;
      yaw -= event.movementX * .004;
      pitch = clamp(pitch + event.movementY * .004, -.12, 1.22);
    },
    wheel(event) {
      const pixels = event.deltaY * (event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? canvas.clientHeight : 1);
      zoom = clamp(zoom * Math.exp(pixels * .001), 12, 76);
    },
    down(event) {
      if (event.button === 0) emitPulse();
    },
  });
  canvas.addEventListener("contextmenu", event => {
    if (mode.active) event.preventDefault();
  }, { signal });

  function pose(part, rotation = part.rotation, position = part.position) {
    part.node.matrixAutoUpdate = false;
    part.node.matrix.compose(position, rotation, part.scale);
    part.node.matrixWorldNeedsUpdate = true;
  }
  function placeAircraft() {
    q.setFromAxisAngle(pitchAxis, state.pitch);
    q.multiply(delta.setFromAxisAngle(rollAxis, state.roll));
    offset.copy(leanCenter).applyQuaternion(q).negate().add(leanCenter).add(body.position);
    offset.x += state.x; offset.y += state.y; offset.z += state.z;
    pose(body, q.premultiply(body.rotation), offset);
    pose(main, q.copy(main.rotation).multiply(delta.setFromAxisAngle(up, state.mainAngle)));
    pose(tail, q.copy(tail.rotation).multiply(delta.setFromAxisAngle(tailAxis, state.tailAngle)));
  }
  function center() {
    return target.set(state.x - 1.5, state.y + 3.6, state.z);
  }
  function track(dt, snap = false) {
    const horizontal = Math.cos(pitch);
    eye.set(Math.cos(yaw) * horizontal, Math.sin(pitch), Math.sin(yaw) * horizontal);
    eye.multiplyScalar(zoom).add(center());
    if (snap) {
      viewer.setCamera({ position: eye.toArray(), target: target.toArray(), fov: 36, near: .06, far: 350 });
    } else {
      const amount = 1 - Math.exp(-7 * dt);
      viewer.controls.target.lerp(target, amount);
      viewer.camera.position.lerp(eye, amount);
      viewer.controls.changed();
    }
  }
  function reset() {
    input.release();
    logic.event("reset", ++generation);
    state = rest;
    yaw = initialYaw; pitch = initialPitch; zoom = 39;
    telemetryTime = 0;
    placeAircraft();
    if (mode.active) track(0, true);
  }
  function setMode(enabled) {
    if (enabled && failed) {
      queueMicrotask(() => mode.set(false));
      return;
    }
    if (enabled) {
      inspectionPose = viewer.savePose();
      for (const door of doors) {
        inspectedDoorMatrices.set(door, door.node.matrix.clone());
        pose(door);
      }
      reset();
    } else if (inspectionPose) {
      reset();
      viewer.setCamera(inspectionPose);
      inspectionPose = null;
      for (const [door, matrix] of inspectedDoorMatrices) {
        door.node.matrix.copy(matrix);
        door.node.matrixWorldNeedsUpdate = true;
      }
      inspectedDoorMatrices.clear();
      if (!failed) output.textContent = "";
    }
  }
  function update(dt) {
    if (!mode.active || failed) return;
    logic.update();
    placeAircraft();
    track(dt);
    updatePulses();
    telemetryTime -= dt;
    if (telemetryTime <= 0) {
      telemetryTime = .1;
      canvas.dataset.flightState = JSON.stringify(state);
      canvas.dataset.flightCamera = JSON.stringify({ yaw, pitch, zoom });
      output.textContent = (state.y > .03 ? "Airborne" : "Helipad") + " · Altitude " + state.y.toFixed(1)
        + " m · Speed " + Math.hypot(state.vx, state.vz).toFixed(1) + " m/s";
    }
  }
  return {
    input, setMode, reset, update,
    focus: () => mode.active ? center().toArray() : null,
    dispose() {
      abort.abort();
      input.dispose();
      logic.dispose();
      overlay.remove();
      delete canvas.dataset.flightPython;
      delete canvas.dataset.flightState;
      delete canvas.dataset.flightCamera;
    },
  };
}
