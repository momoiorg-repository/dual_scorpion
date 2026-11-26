import type { RobotRecord } from "@/hooks/useRobots";

export interface UrdfFileModel {
  path: string;
  blobUrl: string;
  name?: string;
}

export interface JointAnimationConfig {
  name: string;
  type: "sine" | "linear" | "constant";
  min: number;
  max: number;
  speed: number;
  offset: number;
  isDegrees?: boolean;
  customEasing?: (time: number) => number;
}

export interface RobotAnimationConfig {
  joints: JointAnimationConfig[];
  speedMultiplier?: number;
}

export function robotHardwareFields(robot: RobotRecord) {
  if (robot.robot_model !== "dual_scorpion") return {};
  return {
    robot_model: robot.robot_model,
    leader_left_port: robot.leader_left_port,
    leader_right_port: robot.leader_right_port,
    follower_left_port: robot.follower_left_port,
    follower_right_port: robot.follower_right_port,
    view_mapping: robot.view_mapping,
  };
}
