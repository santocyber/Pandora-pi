(function () {
  /* ── EMOTIONS ────────────────────────────────────── */
  var EMOTION_HEAD = {
    neutro:   { x: 0,    y: 0,   z: 0    },
    feliz:    { x: -0.04, y: 0,   z: 0.07 },
    triste:   { x: 0.12,  y: 0,   z: 0    },
    pensando: { x: -0.02, y: 0.1, z: 0.05 },
    falando:  { x: 0,     y: 0,   z: 0    },
    erro:     { x: 0,     y: 0,   z: 0    },
    assustado:{ x: -0.06, y: 0,   z: 0.03 },
  };

  var EMOTION_BODY = {
    neutro:    { armL: 0.04,  armR: -0.04, spineX: 0.01,  spineZ: 0.005, hipZ: 0.002, legL: 0.02, legR: -0.02, footL: 0,    footR: 0,    rootBob: 0.005, speed: 0.6,  headBob: 0.006, rootBobMul: 1,    jitter: 0 },
    feliz:     { armL: 0.3,   armR: -0.3,  spineX: -0.02, spineZ: 0.01,  hipZ: 0.015, legL: 0.08, legR: -0.08, footL: 0.03, footR: -0.03, rootBob: 0.018, speed: 1.2,  headBob: 0.012, rootBobMul: 1.3,  jitter: 0 },
    triste:    { armL: 0.02,  armR: -0.02, spineX: 0.12,  spineZ: 0.005, hipZ: 0.003, legL: 0.01, legR: -0.01, footL: 0,    footR: 0,    rootBob: 0.003, speed: 0.35, headBob: 0.003, rootBobMul: 0.55, jitter: 0 },
    pensando:  { armL: 0.06,  armR: -0.06, spineX: 0.02,  spineZ: 0.03,  hipZ: 0.005, legL: 0.03, legR: -0.03, footL: 0,    footR: 0,    rootBob: 0.006, speed: 0.7,  headBob: 0.004, rootBobMul: 0.8,  jitter: 0 },
    falando:   { armL: 0.1,   armR: -0.1,  spineX: 0.01,  spineZ: 0.01,  hipZ: 0.005, legL: 0.03, legR: -0.03, footL: 0,    footR: 0,    rootBob: 0.008, speed: 0.9,  headBob: 0.01,  rootBobMul: 1.1,  jitter: 0 },
    erro:      { armL: 0.02,  armR: -0.02, spineX: 0.015, spineZ: 0.04,  hipZ: 0.005, legL: 0,    legR: 0,    footL: 0,    footR: 0,    rootBob: 0.004, speed: 0.5,  headBob: 0.018, rootBobMul: 1.45, jitter: 1 },
    assustado: { armL: 0.08,  armR: -0.08, spineX: -0.045,spineZ: 0.02,  hipZ: 0.005, legL: 0.02, legR: -0.02, footL: 0,    footR: 0,    rootBob: 0.005, speed: 0.7,  headBob: 0.012, rootBobMul: 1.15, jitter: 0 },
  };

  var EMOTION_MORPHS = {
    neutro:   { "Fcl_ALL_Neutral": 1.0, "Fcl_MTH_Neutral": 1.0 },
    feliz:    { "Fcl_ALL_Joy": 1.0, "Fcl_EYE_Joy": 1.0, "Fcl_BRW_Joy": 1.0, "Fcl_MTH_Joy": 1.0 },
    triste:   { "Fcl_ALL_Sorrow": 1.0, "Fcl_EYE_Sorrow": 0.9, "Fcl_BRW_Sorrow": 0.9, "Fcl_MTH_Sorrow": 0.9 },
    pensando: { "Fcl_ALL_Fun": 0.8, "Fcl_EYE_Fun": 0.7, "Fcl_BRW_Fun": 0.7, "Fcl_EYE_Surprised": 0.4, "Fcl_MTH_Fun": 0.5 },
    falando:  { "Fcl_ALL_Neutral": 1.0, "Fcl_MTH_Neutral": 0.3 },
    erro:     { "Fcl_ALL_Surprised": 1.0, "Fcl_EYE_Spread": 1.0, "Fcl_BRW_Angry": 0.8, "Fcl_MTH_Surprised": 0.7 },
    assustado:{ "Fcl_ALL_Surprised": 1.0, "Fcl_EYE_Spread": 1.0, "Fcl_BRW_Surprised": 0.8, "Fcl_MTH_Surprised": 0.9 },
  };

  var VISEMES = ["Fcl_MTH_A", "Fcl_MTH_I", "Fcl_MTH_U", "Fcl_MTH_E", "Fcl_MTH_O"];
  var BASE_MORPHS = { "Fcl_HA_Hide": 1.0, "Fcl_HA_Short": 0.0 };
  var BLINK_MORPHS = ["Fcl_EYE_Close_L", "Fcl_EYE_Close_R"];
  var BLINK_INTERVAL = 7.0;

  /* ── ACTIONS ─────────────────────────────────────── */
  var ACTIONS = {
    parado:     { armL: 0.03,  armR: -0.03, spineX: 0.01,  spineZ: 0.005, hipZ: 0.002, legL: 0,     legR: 0,     footL: 0,    footR: 0,    rootBob: 0.005, speed: 0.6,  label: "Parado", morphs: null },
    andando:    { armL: 0.3,   armR: -0.3,  spineX: 0.04,  spineZ: 0.02,  hipZ: 0.01,  legL: 0.3,   legR: -0.3,  footL: 0.05, footR: -0.05, rootBob: 0.012, speed: 1.8,  label: "🚶 Andando", morphs: null },
    comemorando:{ armL: 0.35,  armR: -0.35, spineX: -0.02, spineZ: 0.01,  hipZ: 0.015, legL: 0.08,  legR: -0.08, footL: 0.03, footR: -0.03, rootBob: 0.018, speed: 1.2,  label: "🎉 Comemorando",
                  morphs: {"Fcl_ALL_Joy":1.0,"Fcl_EYE_Joy":1.0,"Fcl_BRW_Joy":0.8,"Fcl_MTH_Joy":1.0} },
    festa:      { armL: 0.4,   armR: -0.4,  spineX: 0.03,  spineZ: 0.04,  hipZ: 0.03,  legL: 0.15,  legR: -0.15, footL: 0.04, footR: -0.04, rootBob: 0.015, speed: 2.0,  label: "🎊 Festa",
                  morphs: {"Fcl_ALL_Fun":1.0,"Fcl_EYE_Fun":1.0,"Fcl_MTH_Large":1.0} },
    cansado:    { armL: 0.02,  armR: -0.02, spineX: 0.12,  spineZ: 0.005, hipZ: 0.003, legL: 0.01,  legR: -0.01, footL: 0,    footR: 0,    rootBob: 0.003, speed: 0.35, label: "😮‍💨 Cansado",
                  morphs: {"Fcl_ALL_Sorrow":0.8,"Fcl_EYE_Sorrow":0.7,"Fcl_BRW_Sorrow":0.6,"Fcl_MTH_Sorrow":0.7} },
    acenando:   { armL: 0,     armR: 0,     spineX: -0.01, spineZ: 0.01,  hipZ: 0.002, legL: 0,     legR: 0,     footL: 0,    footR: 0,    rootBob: 0.005, speed: 1.3,  label: "👋 Acenando",
                  morphs: {"Fcl_ALL_Joy":0.9,"Fcl_MTH_Joy":0.7} },
  };

  /* ── STATE ───────────────────────────────────────── */
  var el, scene, camera, renderer, controls, clock, frameId, ro;
  var emotion = "neutro";
  var action = "parado";
  var mdl = { root: null, hips: null, headBone: null, spineBone: null, shoulderL: null, shoulderR: null,
              upperArmL: null, upperArmR: null, lowerArmL: null, lowerArmR: null, handL: null, handR: null,
              thighL: null, thighR: null, lowerLegL: null, lowerLegR: null, footL: null, footR: null };
  var initQ = {}; // initial quaternions for bind-pose composition
  var pose = {};
  var morphMeshes = [];
  var morphDict = {};
  var visemeIdx = 0, visemeTimer = 0, blinkTimer = 2.0, blinkState = 0;
  var headLerp = { x: 0, y: 0, z: 0 };
  var _euler = new THREE.Euler();
  var _quat = new THREE.Quaternion();

  var CAL = {
    armX_L: -1.2, armY_L: 0, armZ_L: 0,
    armX_R: -1.4, armY_R: 0, armZ_R: 0,
    lowerX_L: 2.0, lowerY_L: 0, lowerZ_L: 0,
    lowerX_R: 0.0, lowerY_R: 0, lowerZ_R: 0,
  };
  window.setCal = function (k, v) { if (k in CAL) { CAL[k] = v; return true; } return false; };
  window.getCal = function (k) { return CAL[k]; };
  window.getCalAll = function () { return JSON.parse(JSON.stringify(CAL)); };

  function blendParam(name, target, dt, rate) {
    if (pose[name] == null) pose[name] = target || 0;
    pose[name] += ((target || 0) - pose[name]) * Math.min(1, dt * (rate || 6));
    return pose[name];
  }

  function phaseSin(t, speed, phase) {
    return Math.sin(t * speed + (phase || 0));
  }

  window.setAction = function (a) { action = a || "parado"; };
  window.updateFace3D = function (e) { if (e) emotion = e; };

  /* ── LOADING ─────────────────────────────────────── */
  window.switchModel3D = function () {}; // no-op, only chibi

  function setLoading(a) { var e = document.getElementById("modelLoading"); if (e) e.classList.toggle("hidden", !a); }

  function tuneMaterial(mat) {
    if (!mat) return;
    var name = (mat.name || "").toUpperCase();

    if (name.indexOf("HAIR") !== -1) {
      mat.transparent = false;
      mat.alphaTest = 0.35;
      mat.depthWrite = true;
      mat.side = THREE.DoubleSide;
      mat.needsUpdate = true;
      return;
    }

    if (name.indexOf("EYEIRIS") !== -1 || name.indexOf("EYEWHITE") !== -1) {
      mat.transparent = false;
      mat.alphaTest = 0;
      mat.depthWrite = true;
      mat.side = THREE.FrontSide;
      mat.needsUpdate = true;
      return;
    }

    if (name.indexOf("EYEHIGHLIGHT") !== -1) {
      mat.transparent = true;
      mat.alphaTest = 0.02;
      mat.depthWrite = false;
      mat.side = THREE.DoubleSide;
      mat.needsUpdate = true;
      return;
    }

    if (name.indexOf("EYELASH") !== -1 || name.indexOf("EYELINE") !== -1 || name.indexOf("BROW") !== -1) {
      mat.transparent = false;
      mat.alphaTest = 0.35;
      mat.depthWrite = true;
      mat.side = THREE.DoubleSide;
      mat.needsUpdate = true;
    }
  }

  function loadModel() {
    if (typeof THREE.GLTFLoader === "undefined") return;
    setLoading(true);
    new THREE.GLTFLoader().load("/static/models/chibi.glb", function (gltf) {
      mdl.root = gltf.scene;
      mdl.root.scale.setScalar(3.4);
      mdl.root.position.set(0, -0.9, 0);
      mdl.root.rotation.y = Math.PI;
      scene.add(mdl.root);

      gltf.scene.traverse(function (child) {
        if (!child.isBone) return;
        var n = child.name;
        if (n.indexOf("J_Bip_C_Hips") !== -1) { mdl.hips = child; initQ.hips = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_C_Head") !== -1 || n.indexOf("Head") !== -1) { mdl.headBone = child; initQ.head = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_C_Spine") !== -1) { mdl.spineBone = child; initQ.spine = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_Shoulder") !== -1) { mdl.shoulderL = child; initQ.shoulderL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_Shoulder") !== -1) { mdl.shoulderR = child; initQ.shoulderR = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_UpperArm") !== -1) { mdl.upperArmL = child; initQ.upperArmL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_UpperArm") !== -1) { mdl.upperArmR = child; initQ.upperArmR = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_LowerArm") !== -1) { mdl.lowerArmL = child; initQ.lowerArmL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_LowerArm") !== -1) { mdl.lowerArmR = child; initQ.lowerArmR = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_Hand") !== -1) { mdl.handL = child; initQ.handL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_Hand") !== -1) { mdl.handR = child; initQ.handR = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_UpperLeg") !== -1) { mdl.thighL = child; initQ.thighL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_UpperLeg") !== -1) { mdl.thighR = child; initQ.thighR = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_LowerLeg") !== -1) { mdl.lowerLegL = child; initQ.lowerLegL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_LowerLeg") !== -1) { mdl.lowerLegR = child; initQ.lowerLegR = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_L_Foot") !== -1) { mdl.footL = child; initQ.footL = child.quaternion.clone(); }
        if (n.indexOf("J_Bip_R_Foot") !== -1) { mdl.footR = child; initQ.footR = child.quaternion.clone(); }
      });

      gltf.scene.traverse(function (child) {
        if (child.isMesh && child.material) {
          if (Array.isArray(child.material)) child.material.forEach(tuneMaterial);
          else tuneMaterial(child.material);
        }

        if (child.isMesh && child.morphTargetInfluences && child.morphTargetDictionary) {
          morphMeshes.push(child);
          for (var k in child.morphTargetDictionary) morphDict[k] = child.morphTargetDictionary[k];
        }
      });
      console.debug("[face3d] Bones found:",
        "hips=" + !!mdl.hips, "head=" + !!mdl.headBone, "spine=" + !!mdl.spineBone,
        "shoulderL=" + !!mdl.shoulderL, "shoulderR=" + !!mdl.shoulderR,
        "upperArmL=" + !!mdl.upperArmL, "upperArmR=" + !!mdl.upperArmR,
        "lowerArmL=" + !!mdl.lowerArmL, "lowerArmR=" + !!mdl.lowerArmR,
        "handL=" + !!mdl.handL, "handR=" + !!mdl.handR,
        "thighL=" + !!mdl.thighL, "thighR=" + !!mdl.thighR,
        "lowerLegL=" + !!mdl.lowerLegL, "lowerLegR=" + !!mdl.lowerLegR,
        "footL=" + !!mdl.footL, "footR=" + !!mdl.footR);

      setLoading(false);
      applyMorphs();
    });
  }

  /* ── MORPHS ──────────────────────────────────────── */
  function applyMorphs() {
    var act = ACTIONS[action] || ACTIONS.parado;
    var targets = act.morphs || EMOTION_MORPHS[emotion] || EMOTION_MORPHS.neutro;
    var isFalando = emotion === "falando" && !act.morphs;

    for (var mi = 0; mi < morphMeshes.length; mi++) {
      var mesh = morphMeshes[mi];
      var inf = mesh.morphTargetInfluences;
      var dict = mesh.morphTargetDictionary;
      if (!inf || !dict) continue;

      for (var ii = 0; ii < inf.length; ii++) inf[ii] = 0;

      for (var tname in targets) {
        var idx = dict[tname];
        if (idx != null) inf[idx] = targets[tname];
      }

      if (isFalando && dict["Fcl_MTH_Neutral"]) {
        var vi = dict[VISEMES[visemeIdx]];
        if (vi != null) inf[vi] = 0.7;
        if (dict["Fcl_MTH_Neutral"]) inf[dict["Fcl_MTH_Neutral"]] = 0.3;
      }

      if (blinkState > 0) {
        for (var bi = 0; bi < BLINK_MORPHS.length; bi++) {
          var bi2 = dict[BLINK_MORPHS[bi]];
          if (bi2 != null) inf[bi2] = Math.min(1, blinkState);
        }
      }

      for (var bname in BASE_MORPHS) {
        var bi3 = dict[bname];
        if (bi3 != null && inf[bi3] === 0) inf[bi3] = BASE_MORPHS[bname];
      }
    }
  }

  /* ── ANIMATE ─────────────────────────────────────── */
  function animate() {
    if (!renderer || !scene || !camera || !clock) return;
    frameId = requestAnimationFrame(animate);
    var dt = Math.min(clock.getDelta(), 0.033);
    var t = clock.getElapsedTime();

    var act = ACTIONS[action] || ACTIONS.parado;
    var body = EMOTION_BODY[emotion] || EMOTION_BODY.neutro;
    var as = body.speed || act.speed || 0.001;
    var phase = t * as;
    var armL = blendParam("armL", body.armL || 0, dt, 7);
    var armR = blendParam("armR", body.armR || 0, dt, 7);
    var spineX = blendParam("spineX", body.spineX || 0, dt, 6);
    var spineZ = blendParam("spineZ", body.spineZ || 0, dt, 6);
    var hipZ = blendParam("hipZ", body.hipZ || 0, dt, 6);
    var legL = blendParam("legL", body.legL || 0, dt, 7);
    var legR = blendParam("legR", body.legR || 0, dt, 7);
    var footL = blendParam("footL", body.footL || 0, dt, 7);
    var footR = blendParam("footR", body.footR || 0, dt, 7);
    var rootBob = blendParam("rootBob", (body.rootBob || 0) * (body.rootBobMul || 1), dt, 6);
    var jitter = body.jitter ? Math.sin(t * 35) * 0.018 : 0;
    var stepA = phaseSin(t, as, 0);
    var stepB = phaseSin(t, as, Math.PI);
    var walkL = Math.sin(phase + Math.PI);
    var walkR = Math.sin(phase + Math.PI * 2.5);

    if (mdl.headBone) {
      var hr = EMOTION_HEAD[emotion] || EMOTION_HEAD.neutro;
      var tx = hr.x;
      var ty = hr.y;
      var tz = hr.z;
      tx += Math.sin(phase * 0.65) * (body.headBob || 0);
      tz += Math.cos(phase * 0.5) * (body.headBob || 0) * 0.65;
      if (emotion === "erro") tx += Math.sin(t * 28) * 0.02;
      headLerp.x += (tx - headLerp.x) * Math.min(1, dt * 4);
      headLerp.y += (ty - headLerp.y) * Math.min(1, dt * 4);
      headLerp.z += (tz - headLerp.z) * Math.min(1, dt * 4);
      _euler.set(headLerp.x, headLerp.y, headLerp.z);
      _quat.setFromEuler(_euler);
      mdl.headBone.quaternion.copy(initQ.head).multiply(_quat);
    }

    if (mdl.spineBone) {
      var sx = spineX * Math.sin(phase * 0.85) + jitter;
      var sz = spineZ * Math.cos(phase * 0.72) + jitter * 0.7;
      var sy = jitter * 0.3;
      if (emotion === "feliz") {
        sx = spineX * Math.sin(phase * 1.3);
        sz = spineZ * Math.cos(phase * 1.1);
        sy = Math.sin(phase * 1.4) * 0.04;
      }
      if (emotion === "triste") {
        sx = spineX + Math.sin(phase * 0.5) * 0.015;
        sz = spineZ * Math.sin(phase * 0.4);
        sy = Math.sin(phase * 0.3) * 0.01;
      }
      if (emotion === "pensando") {
        sy = Math.sin(phase * 0.6) * 0.02;
      }
      if (emotion === "falando") {
        sy = Math.sin(phase * 0.9) * 0.015;
      }
      _euler.set(sx, sy, sz);
      _quat.setFromEuler(_euler);
      mdl.spineBone.quaternion.copy(initQ.spine).multiply(_quat);
    }

    if (mdl.hips) {
      var hz = hipZ * Math.sin(phase + Math.PI * 0.5) + jitter * 0.45;
      var hy = 0;
      if (emotion === "feliz") {
        hz = hipZ * Math.sin(phase * 1.6);
        hy = Math.sin(phase * 1.3) * 0.02;
      }
      if (emotion === "triste") {
        hy = Math.sin(phase * 0.3) * 0.005;
      }
      _euler.set(0, hy, hz);
      _quat.setFromEuler(_euler);
      mdl.hips.quaternion.copy(initQ.hips).multiply(_quat);
    }

    if (mdl.shoulderL) {
      var shL_X = 0, shL_Z = 0;
      if (emotion === "feliz") {
        shL_Z = Math.sin(phase * 1.8) * 0.04;
      }
      if (emotion === "neutro") {
        shL_Z = Math.sin(phase * 0.5) * 0.02;
      }
      _euler.set(shL_X, 0, shL_Z);
      _quat.setFromEuler(_euler);
      mdl.shoulderL.quaternion.copy(initQ.shoulderL).multiply(_quat);
    }
    if (mdl.shoulderR) {
      var shR_X = 0, shR_Z = 0;
      if (emotion === "feliz") {
        shR_Z = Math.sin(phase * 1.8 + Math.PI) * 0.04;
      }
      if (emotion === "neutro") {
        shR_Z = Math.sin(phase * 0.5 + Math.PI) * 0.02;
      }
      _euler.set(shR_X, 0, shR_Z);
      _quat.setFromEuler(_euler);
      mdl.shoulderR.quaternion.copy(initQ.shoulderR).multiply(_quat);
    }
    if (mdl.upperArmL) {
      var armLX = CAL.armX_L, armLY = CAL.armY_L, armLZ = CAL.armZ_L;
      if (emotion === "feliz") {
        var pumpL = Math.sin(phase * 1.2) * 0.35;
        armLY = CAL.armY_L + Math.abs(pumpL);
      }
      var swingL = armL * stepB;
      _euler.set(armLX, armLY, armLZ + swingL);
      _quat.setFromEuler(_euler);
      mdl.upperArmL.quaternion.copy(initQ.upperArmL).multiply(_quat);
    }
    if (mdl.upperArmR) {
      var armRX = CAL.armX_R, armRY = CAL.armY_R, armRZ = CAL.armZ_R;
      if (emotion === "feliz") {
        var pumpR = Math.sin(phase * 1.2) * 0.35;
        armRY = CAL.armY_R - Math.abs(pumpR);
      }
      var swingR = -armR * stepA;
      _euler.set(armRX, armRY, armRZ + swingR);
      _quat.setFromEuler(_euler);
      mdl.upperArmR.quaternion.copy(initQ.upperArmR).multiply(_quat);
    }

    if (mdl.lowerArmL) {
      var loX_L = CAL.lowerX_L, loY_L = CAL.lowerY_L, loZ_L = CAL.lowerZ_L;
      if (emotion === "feliz") {
        loZ_L = CAL.lowerZ_L + Math.sin(phase * 1.5) * 0.2;
      }
      _euler.set(loX_L, loY_L, loZ_L);
      _quat.setFromEuler(_euler);
      mdl.lowerArmL.quaternion.copy(initQ.lowerArmL).multiply(_quat);
    }
    if (mdl.lowerArmR) {
      var loX_R = CAL.lowerX_R, loY_R = CAL.lowerY_R, loZ_R = CAL.lowerZ_R;
      if (emotion === "feliz") {
        loZ_R = CAL.lowerZ_R + Math.sin(phase * 1.5 + Math.PI) * 0.2;
      }
      _euler.set(loX_R, loY_R, loZ_R);
      _quat.setFromEuler(_euler);
      mdl.lowerArmR.quaternion.copy(initQ.lowerArmR).multiply(_quat);
    }
    if (mdl.handL) {
      mdl.handL.quaternion.copy(initQ.handL);
    }
    if (mdl.handR) {
      mdl.handR.quaternion.copy(initQ.handR);
    }

    if (mdl.thighL) {
      var tl = legL * walkL;
      if (emotion === "feliz") tl = legL * Math.sin(phase * 0.8);
      _euler.set(tl, 0, 0);
      _quat.setFromEuler(_euler);
      mdl.thighL.quaternion.copy(initQ.thighL).multiply(_quat);
    }
    if (mdl.thighR) {
      var tr = legR * walkR;
      if (emotion === "feliz") tr = legR * Math.sin(phase * 0.8 + Math.PI);
      _euler.set(tr, 0, 0);
      _quat.setFromEuler(_euler);
      mdl.thighR.quaternion.copy(initQ.thighR).multiply(_quat);
    }
    if (mdl.lowerLegL && mdl.thighL) {
      var kneeL = Math.max(0, legL * walkL) * 0.5;
      if (emotion === "feliz") kneeL *= 0.35;
      _euler.set(kneeL, 0, 0);
      _quat.setFromEuler(_euler);
      mdl.lowerLegL.quaternion.copy(initQ.lowerLegL).multiply(_quat);
    }
    if (mdl.lowerLegR && mdl.thighR) {
      var kneeR = Math.max(0, legR * walkR) * 0.5;
      if (emotion === "feliz") kneeR *= 0.35;
      _euler.set(kneeR, 0, 0);
      _quat.setFromEuler(_euler);
      mdl.lowerLegR.quaternion.copy(initQ.lowerLegR).multiply(_quat);
    }

    if (mdl.footL) {
      var toeL = Math.max(0, legL * walkL) * (Math.abs(footL) / Math.max(0.001, Math.abs(legL)));
      _euler.set(-toeL, 0, 0);
      _quat.setFromEuler(_euler);
      mdl.footL.quaternion.copy(initQ.footL).multiply(_quat);
    }
    if (mdl.footR) {
      var toeR = Math.max(0, legR * walkR) * (Math.abs(footR) / Math.max(0.001, Math.abs(legR)));
      _euler.set(-toeR, 0, 0);
      _quat.setFromEuler(_euler);
      mdl.footR.quaternion.copy(initQ.footR).multiply(_quat);
    }

    if (mdl.root) {
      var bob = rootBob * Math.sin(t * (as > 0.01 ? as * 2 : 1.1));
      if (emotion === "feliz") bob = rootBob * Math.max(0, Math.sin(phase * 2.4) * 1.3 + 0.2);
      if (emotion === "triste") bob = rootBob * Math.sin(t * 0.7);
      mdl.root.position.y = -0.9 + bob;
    }

    blinkTimer -= dt;
    if (blinkTimer <= 0) { blinkState = 0.01; blinkTimer = BLINK_INTERVAL + Math.random() * 2.0; }
    if (blinkState > 0) { blinkState += dt * 6.0; if (blinkState >= 1.0) blinkState = -0.01; }
    if (blinkState < 0) { blinkState -= dt * 4.0; if (blinkState <= -1.0) { blinkState = 0; blinkTimer = BLINK_INTERVAL + Math.random() * 2.5; } }

    if (morphMeshes.length > 0) {
      visemeTimer += dt;
      if (visemeTimer > 0.1) {
        if (emotion === "falando") visemeIdx = (visemeIdx + 1) % VISEMES.length;
        else visemeIdx = 0;
        visemeTimer = 0;
      }
      applyMorphs();
    }

    if (controls) controls.update();
    renderer.render(scene, camera);
  }

  /* ── LIFECYCLE ───────────────────────────────────── */

  function createEnvTexture() {
    var pmrem = new THREE.PMREMGenerator(renderer);
    pmrem.compileEquirectangularShader();
    var env = new THREE.Scene();
    env.background = new THREE.Color(0x0a1628);
    env.add(new THREE.HemisphereLight(0x4488ff, 0x3388cc, 1.2));
    for (var i = 0; i < 4; i++) {
      var b = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1.2, 1.2),
        new THREE.MeshBasicMaterial({ color: [0x4488ff, 0xff4466, 0x44ffaa, 0xffaa44][i] }));
      var a = (i / 4) * Math.PI * 2 + 0.5;
      b.position.set(Math.cos(a) * 4.2, Math.sin(i * 1.8) * 0.6 - 0.2, Math.sin(a) * 4.2);
      env.add(b);
    }
    var tex = pmrem.fromScene(env, 0, 0.1, 100).texture;
    pmrem.dispose(); return tex;
  }

  window.initFace3D = function (container) {
    if (typeof THREE === "undefined" || !container) return;
    window.destroyFace3D();

    el = container;
    var w = container.clientWidth || 760, h = container.clientHeight || w / 1.18;

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(42, w / h, 0.1, 30);
    camera.position.set(0, 0.1, 3.7);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(w, h);
    renderer.setClearColor(0x000000, 0);
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.06;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    if (typeof THREE.OrbitControls !== "undefined")
      controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.target.set(0, 0.2, 0);
    controls.update();

    scene.environment = createEnvTexture();
    scene.environmentIntensity = 0.55;
    scene.add(new THREE.AmbientLight(0x95acbd, 0.22));

    var key = new THREE.DirectionalLight(0xffffff, 1.35);
    key.position.set(2, 3.2, 4.2); key.castShadow = true;
    key.shadow.mapSize.set(256, 256); key.shadow.radius = 4;
    scene.add(key);

    var fill = new THREE.DirectionalLight(0x78b0ff, 0.52);
    fill.position.set(-3, 1.2, 2.6); scene.add(fill);

    var rim = new THREE.DirectionalLight(0x6688cc, 0.55);
    rim.position.set(0.1, -1.2, -3.2); scene.add(rim);

    loadModel();

    clock = new THREE.Clock();
    ro = new ResizeObserver(function () {
      if (!el || !camera || !renderer) return;
      var cw = el.clientWidth || 760, ch = el.clientHeight || cw / 1.18;
      camera.aspect = cw / ch; camera.updateProjectionMatrix();
      renderer.setSize(cw, ch);
    });
    ro.observe(container);
    animate();
  };

  window.destroyFace3D = function () {
    if (frameId) { cancelAnimationFrame(frameId); frameId = null; }
    if (ro) { ro.disconnect(); ro = null; }
    if (mdl.root) {
      scene.remove(mdl.root);
      mdl.root.traverse(function (child) {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
          if (Array.isArray(child.material)) child.material.forEach(function (m) { m.dispose(); });
          else child.material.dispose();
        }
      });
    }
    if (renderer) { renderer.dispose(); if (el && renderer.domElement && renderer.domElement.parentNode === el) el.removeChild(renderer.domElement); }
    disposeAll(scene);
    el = null; scene = null; camera = null; renderer = null; controls = null; clock = null; morphMeshes = []; morphDict = {}; initQ = {}; pose = {};
  };

  function disposeAll(obj) {
    if (!obj) return;
    obj.traverse(function (child) {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach(function (m) { m.dispose(); });
        else child.material.dispose();
      }
    });
  }
})();
