export type ArmSide = "right" | "left";

export type ArmVisualMapping = {
  sign: [number, number, number, number, number, number, number];
  offset_deg: [number, number, number, number, number, number, number];
  gripper_closed_deg: number;
  gripper_open_deg: number;
};

export type ScorpionVisualMapping = {
  right: ArmVisualMapping;
  left: ArmVisualMapping;
};

export const SCORPION_ARM_JOINTS = [
  "joint0",
  "joint1",
  "joint2",
  "joint3",
  "joint4",
  "joint5",
  "joint6",
] as const;

/** Default Full-CAD sign conventions for drive_mode == 0, with a visible gripper stroke. */
export const DEFAULT_SCORPION_VISUAL_MAPPING: ScorpionVisualMapping = {
  right: {
    sign: [-1, 1, -1, 1, 1, -1, -1],
    offset_deg: [0, 0, 0, 0, 0, 0, 0],
    gripper_closed_deg: 0,
    gripper_open_deg: -45,
  },
  left: {
    sign: [1, -1, 1, -1, -1, 1, -1],
    offset_deg: [0, 0, 0, 0, 0, 0, 0],
    gripper_closed_deg: 0,
    gripper_open_deg: -45,
  },
};

const STORAGE_PREFIX = "lelab.scorpionVisualMapping.";

function normalizeArm(raw: unknown, fallback: ArmVisualMapping): ArmVisualMapping {
  const obj = raw && typeof raw === "object" ? (raw as Record<string, unknown>) : {};
  const rawSign = Array.isArray(obj.sign) ? obj.sign : [];
  const rawOffset = Array.isArray(obj.offset_deg) ? obj.offset_deg : [];

  const sign = fallback.sign.map((def, i) => {
    const val = rawSign[i];
    return val === 1 || val === -1 ? val : def;
  }) as ArmVisualMapping["sign"];

  const offset_deg = fallback.offset_deg.map((def, i) => {
    const val = rawOffset[i];
    return typeof val === "number" && Number.isFinite(val) ? val : def;
  }) as ArmVisualMapping["offset_deg"];

  const gripper_closed_deg =
    typeof obj.gripper_closed_deg === "number" && Number.isFinite(obj.gripper_closed_deg)
      ? obj.gripper_closed_deg
      : fallback.gripper_closed_deg;
  const gripper_open_deg =
    typeof obj.gripper_open_deg === "number" && Number.isFinite(obj.gripper_open_deg)
      ? obj.gripper_open_deg
      : fallback.gripper_open_deg;

  return { sign, offset_deg, gripper_closed_deg, gripper_open_deg };
}

export function normalizeVisualMapping(raw: unknown): ScorpionVisualMapping {
  const obj = raw && typeof raw === "object" ? (raw as Record<string, unknown>) : {};
  return {
    right: normalizeArm(obj.right, DEFAULT_SCORPION_VISUAL_MAPPING.right),
    left: normalizeArm(obj.left, DEFAULT_SCORPION_VISUAL_MAPPING.left),
  };
}

export function computeScorpionUrdfJoints(
  motorPositions: Record<string, number>,
  mapping: ScorpionVisualMapping,
): Record<string, number> {
  const joints: Record<string, number> = {};
  for (const [side, suffix] of [
    ["right", "R"],
    ["left", "L"],
  ] as const) {
    const arm = mapping[side];
    for (let i = 0; i < SCORPION_ARM_JOINTS.length; i++) {
      const value = motorPositions[`${side}_joint${i}.pos`];
      if (typeof value === "number" && Number.isFinite(value)) {
        joints[`joint${i}_${suffix}`] = (((value - arm.offset_deg[i]) / arm.sign[i]) * Math.PI) / 180;
      }
    }
    const gripper = motorPositions[`${side}_gripper.pos`];
    if (typeof gripper === "number" && Number.isFinite(gripper)) {
      const clamped = Math.min(100, Math.max(0, gripper));
      const deg = arm.gripper_closed_deg + ((arm.gripper_open_deg - arm.gripper_closed_deg) * clamped) / 100;
      joints[`joint7_${suffix}`] = (deg * Math.PI) / 180;
    }
  }
  return joints;
}

export function zeroArmToPositions(
  mapping: ScorpionVisualMapping,
  side: ArmSide,
  motorPositions: Record<string, number>,
): ScorpionVisualMapping {
  const arm = mapping[side];
  const nextOffsets = arm.offset_deg.map((prev, i) => {
    const value = motorPositions[`${side}_joint${i}.pos`];
    return typeof value === "number" && Number.isFinite(value) ? Math.round(value * 10) / 10 : prev;
  }) as ArmVisualMapping["offset_deg"];
  return {
    ...mapping,
    [side]: {
      ...arm,
      offset_deg: nextOffsets,
    },
  };
}

export function loadLocalVisualMapping(robotName: string | null): ScorpionVisualMapping | null {
  try {
    const raw = localStorage.getItem(`${STORAGE_PREFIX}${robotName || "default"}`);
    return raw ? normalizeVisualMapping(JSON.parse(raw)) : null;
  } catch {
    return null;
  }
}

export function saveLocalVisualMapping(robotName: string | null, mapping: ScorpionVisualMapping | null): void {
  try {
    const key = `${STORAGE_PREFIX}${robotName || "default"}`;
    if (mapping === null) {
      localStorage.removeItem(key);
    } else {
      localStorage.setItem(key, JSON.stringify(mapping));
    }
  } catch {
    // Browser storage is optional.
  }
}
