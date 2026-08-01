// Live skeleton view of the dual Scorpion arms, drawn in passthrough AR.
//
// The server broadcasts `robot_skeleton` messages (~10 Hz) containing, per
// arm, the world positions of the joint chain for the COMMANDED pose and the
// last MEASURED pose, plus per-joint limit proximity and command-vs-measured
// error. This component renders:
//   - measured pose: solid spheres + tubes (what the robot is really doing)
//   - commanded pose: translucent ghost (what it was told to do)
//   - joint colors: green -> orange -> red as a joint approaches its limit
//   - stalled joints (command far ahead of measured): pulsing magenta
//   - a faint sphere per shoulder showing the reachable workspace bubble
//
// Robot base frame is z-up, arms extend toward -Y. The scene is y-up with -Z
// away from the viewer, so the model is placed in front of the operator with
// the arms reaching away (embodiment view, matching mirror_lateral=false).
// Toggle visibility by clicking either thumbstick.

AFRAME.registerComponent('robot-skeleton', {
  init: function () {
    this.MODEL_SCALE = 0.5;
    // Chest height, half a metre ahead of where the operator starts.
    this.MODEL_POSITION = new THREE.Vector3(0, 1.1, -0.55);
    this.STALL_DEG = 12;      // |commanded - measured| that counts as stalled
    this.LIMIT_ORANGE = 0.75; // limit_frac where green fades to orange
    this.LIMIT_RED = 0.92;    // limit_frac where a joint shows red

    this.root = new THREE.Group();
    this.root.position.copy(this.MODEL_POSITION);
    this.root.scale.setScalar(this.MODEL_SCALE);
    this.el.sceneEl.object3D.add(this.root);

    this.arms = {};
    for (const arm of ['left', 'right']) {
      this.arms[arm] = this._buildArm();
    }
    this.workspaceSpheres = {};
    const uiConfig = window.TELEGRIP_VR_UI_CONFIG;
    this._visible = Boolean(
      uiConfig && uiConfig.shadow_enabled_by_default);
    this.root.visible = this._visible;
    this._lastMessage = null;
  },

  // Robot base frame (x, y, z; z-up, arms toward -Y) -> three.js (y-up).
  // Maps robot +Y to scene +Z and robot X to scene -X (det +1, no mirror),
  // so the arms reach away from the viewer (-Z) like your own arms do.
  _toScene: function (pt) {
    return new THREE.Vector3(-pt[0], pt[2], pt[1]);
  },

  _buildArm: function () {
    const N = 8; // joint0..joint6 + TCP
    const mk = (opacity) => {
      const group = new THREE.Group();
      const joints = [];
      const bones = [];
      for (let i = 0; i < N; i++) {
        const sphere = new THREE.Mesh(
          new THREE.SphereGeometry(0.022, 12, 12),
          new THREE.MeshBasicMaterial({
            color: 0x30d158, transparent: opacity < 1, opacity: opacity,
            depthTest: opacity >= 1,
          }));
        group.add(sphere);
        joints.push(sphere);
      }
      for (let i = 0; i < N - 1; i++) {
        const bone = new THREE.Mesh(
          new THREE.CylinderGeometry(0.009, 0.009, 1, 8),
          new THREE.MeshBasicMaterial({
            color: 0xf2f2f7, transparent: opacity < 1, opacity: opacity * 0.9,
            depthTest: opacity >= 1,
          }));
        group.add(bone);
        bones.push(bone);
      }
      this.root.add(group);
      return { group, joints, bones };
    };
    return { measured: mk(1.0), commanded: mk(0.28) };
  },

  _placeBone: function (bone, a, b) {
    const mid = a.clone().add(b).multiplyScalar(0.5);
    const dir = b.clone().sub(a);
    const len = Math.max(dir.length(), 1e-6);
    bone.position.copy(mid);
    bone.scale.set(1, len, 1);
    bone.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0), dir.normalize());
  },

  _jointColor: function (limitFrac, errorDeg, pulse) {
    if (Math.abs(errorDeg) > this.STALL_DEG) {
      // Stalled motor: pulsing magenta, unmistakable.
      return pulse ? 0xff2d9a : 0x8e1f63;
    }
    if (limitFrac >= this.LIMIT_RED) return 0xff453a;
    if (limitFrac >= this.LIMIT_ORANGE) return 0xff9f0a;
    return 0x30d158;
  },

  updateFromMessage: function (msg) {
    this._lastMessage = msg;
    if (!this._visible) return;
    const pulse = (Date.now() % 500) < 250;

    for (const arm of ['left', 'right']) {
      const data = msg[arm];
      const vis = this.arms[arm];
      if (!vis) continue;
      if (!data) {
        vis.measured.group.visible = false;
        vis.commanded.group.visible = false;
        continue;
      }
      vis.measured.group.visible = true;

      for (const kind of ['measured', 'commanded']) {
        const pts = (data[kind] || []).map((p) => this._toScene(p));
        const skel = vis[kind];
        if (pts.length !== skel.joints.length) {
          skel.group.visible = false;
          continue;
        }
        for (let i = 0; i < pts.length; i++) {
          skel.joints[i].position.copy(pts[i]);
          if (kind === 'measured' && i < data.limit_frac.length) {
            // Gray means feedback is missing/stale. Never show green
            // "healthy" joints when Python has no fresh motor measurement.
            skel.joints[i].material.color.setHex(data.measured_valid === false
              ? 0x8e8e93
              : this._jointColor(data.limit_frac[i], data.error_deg[i], pulse));
          }
        }
        for (let i = 0; i < pts.length - 1; i++) {
          this._placeBone(skel.bones[i], pts[i], pts[i + 1]);
        }
        // Hide the ghost entirely when it matches the measured pose, so a
        // healthy arm shows one clean skeleton instead of a blurry double.
        if (kind === 'commanded') {
          const maxErr = Math.max(...data.error_deg.map(Math.abs));
          skel.group.visible = data.measured_valid !== false &&
            maxErr > this.STALL_DEG * 0.5;
        }
      }

      if (data.workspace_center && !this.workspaceSpheres[arm] &&
          msg.workspace_radius) {
        const sphere = new THREE.Mesh(
          new THREE.SphereGeometry(msg.workspace_radius, 24, 24),
          new THREE.MeshBasicMaterial({
            color: 0x0a84ff, transparent: true, opacity: 0.05,
            depthWrite: false, side: THREE.DoubleSide,
          }));
        sphere.position.copy(this._toScene(data.workspace_center));
        this.root.add(sphere);
        this.workspaceSpheres[arm] = sphere;
      }
    }
  },

  toggle: function () {
    this.setEnabled(!this._visible);
  },

  setEnabled: function (enabled) {
    this._visible = Boolean(enabled);
    this.root.visible = this._visible;
    this.el.sceneEl.emit('skeleton-visibility-changed',
      { enabled: this._visible }, false);
    if (this._visible && this._lastMessage) {
      this.updateFromMessage(this._lastMessage);
    }
  },
});
