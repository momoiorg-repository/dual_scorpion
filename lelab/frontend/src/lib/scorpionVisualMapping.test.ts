import { describe, expect, test } from "vitest";
import {
  DEFAULT_SCORPION_VISUAL_MAPPING,
  computeScorpionUrdfJoints,
  normalizeVisualMapping,
  zeroArmToPositions,
} from "./scorpionVisualMapping";

describe("scorpionVisualMapping", () => {
  test("computes all 16 URDF joints including grippers from motor positions", () => {
    const positions: Record<string, number> = {
      "right_joint0.pos": 30,
      "left_joint0.pos": 15,
      "right_gripper.pos": 50,
      "left_gripper.pos": 100,
    };
    const joints = computeScorpionUrdfJoints(positions, DEFAULT_SCORPION_VISUAL_MAPPING);
    expect(joints.joint0_R).toBeCloseTo((-30 * Math.PI) / 180, 6);
    expect(joints.joint0_L).toBeCloseTo((15 * Math.PI) / 180, 6);
    expect(joints.joint7_R).toBeCloseTo((-22.5 * Math.PI) / 180, 6);
    expect(joints.joint7_L).toBeCloseTo((-45 * Math.PI) / 180, 6);
  });

  test("zeroArmToPositions sets offsets to current frame angles so output becomes 0 rad", () => {
    const positions = {
      "right_joint0.pos": 24.34,
      "right_joint1.pos": -12.86,
      "right_joint2.pos": 5.0,
      "right_joint3.pos": 35.0,
      "right_joint4.pos": -6.0,
      "right_joint5.pos": -80.5,
      "right_joint6.pos": -5.8,
    };
    const zeroed = zeroArmToPositions(DEFAULT_SCORPION_VISUAL_MAPPING, "right", positions);
    const joints = computeScorpionUrdfJoints(positions, zeroed);
    expect(Math.abs(joints.joint0_R)).toBeLessThan(0.002);
    expect(Math.abs(joints.joint1_R)).toBeLessThan(0.002);
    expect(Math.abs(joints.joint5_R)).toBeLessThan(0.002);
  });

  test("normalizeVisualMapping falls back safely on malformed fields", () => {
    const normalized = normalizeVisualMapping({
      right: { sign: [99, 1], offset_deg: [12, "bad"] },
    });
    expect(normalized.right.sign[0]).toBe(DEFAULT_SCORPION_VISUAL_MAPPING.right.sign[0]);
    expect(normalized.right.sign[1]).toBe(1);
    expect(normalized.right.offset_deg[0]).toBe(12);
    expect(normalized.right.offset_deg[1]).toBe(0);
  });
});
