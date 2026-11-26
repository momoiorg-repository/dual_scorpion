// @vitest-environment jsdom
import React, { act } from "react";
import { createRoot } from "react-dom/client";
import { afterEach, expect, test, vi } from "vitest";

import { ScorpionMappingWizardModal } from "./ScorpionMappingWizardModal";
import type { RobotRecord } from "@/hooks/useRobots";

const toastMock = vi.fn();
const stableToast = { toast: toastMock };

vi.mock("@/hooks/use-toast", () => ({ useToast: () => stableToast }));
vi.mock("@/components/UrdfViewer", () => ({
  default: ({ recordedFrame }: { recordedFrame?: { joints: Record<string, number> } }) => (
    <div
      data-testid="wizard-urdf-viewer"
      data-joint0-r={String(recordedFrame?.joints?.joint0_R ?? 0)}
    />
  ),
}));

let mockPositions: Record<string, number> = {
  "right_joint0.pos": 5.0,
  "right_joint1.pos": 2.0,
  "right_joint2.pos": -3.0,
  "right_joint3.pos": 4.0,
  "right_joint4.pos": 1.0,
  "right_joint5.pos": -2.0,
  "right_joint6.pos": 0.0,
  "right_gripper.pos": 0,
};
const savedBodies: Array<{
  joints?: Record<string, { expected_model_direction: number; motor_reference_deg: number }>;
  view_gripper_deg?: Record<string, { closed: number; open: number }>;
}> = [];

const fetchMock = vi.fn(async (input: RequestInfo | URL, init?: RequestInit) => {
  const url = String(input);
  const json = (body: unknown, status = 200) =>
    new Response(JSON.stringify(body), {
      status,
      headers: { "Content-Type": "application/json" },
    });

  if (url.endsWith("/scorpion-mapping/start")) {
    return json({ success: true, message: "ok" });
  }
  if (url.endsWith("/scorpion-mapping/positions")) {
    return json({
      success: true,
      active: true,
      positions: mockPositions,
    });
  }
  if (url.endsWith("/scorpion-mapping/stop")) {
    return json({ success: true });
  }
  if (url.endsWith("/scorpion-mapping/save")) {
    savedBodies.push(JSON.parse(String(init?.body || "{}")));
    return json({
      success: true,
      filename: "follower-mapping.yaml",
      verified: false,
      verification_status: "partial",
    });
  }
  return json({ status: "error" }, 404);
});

const stableApi = {
  baseUrl: "http://api",
  fetchWithHeaders: (...args: Parameters<typeof fetch>) => fetchMock(...args),
};

vi.mock("@/contexts/ApiContext", () => ({ useApi: () => stableApi }));

afterEach(() => {
  document.body.innerHTML = "";
  toastMock.mockReset();
  fetchMock.mockClear();
  savedBodies.length = 0;
  mockPositions = {
    "right_joint0.pos": 5.0,
    "right_joint1.pos": 2.0,
    "right_joint2.pos": -3.0,
    "right_joint3.pos": 4.0,
    "right_joint4.pos": 1.0,
    "right_joint5.pos": -2.0,
    "right_joint6.pos": 0.0,
    "right_gripper.pos": 0,
  };
});

test("ScorpionMappingWizardModal records whole-arm Pose 1 + Pose 2, allows Live Mirror adjustments, and saves YAML", async () => {
  Object.assign(globalThis, { IS_REACT_ACT_ENVIRONMENT: true });
  const robot: RobotRecord = {
    name: "test-scorp",
    robot_model: "dual_scorpion",
    leader_port: "",
    follower_port: "",
    leader_left_port: "",
    leader_right_port: "",
    follower_left_port: "/dev/ttyACM0",
    follower_right_port: "/dev/ttyACM1",
    leader_config: "",
    follower_config: "follower.json",
    cameras: [],
    is_clean: false,
  };
  const onSaved = vi.fn();
  const onOpenChange = vi.fn();
  const host = document.createElement("div");
  document.body.append(host);
  const root = createRoot(host);

  await act(async () => {
    root.render(
      <ScorpionMappingWizardModal
        open={true}
        onOpenChange={onOpenChange}
        robot={robot}
        onSaved={onSaved}
      />,
    );
  });

  const connectBtn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("Connect Follower"),
  );
  await act(async () => {
    connectBtn?.click();
    await Promise.resolve();
    await Promise.resolve();
  });

  const pose1Btn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("ポーズ 1 (L字) を記録"),
  );
  await act(async () => {
    pose1Btn?.click();
    await Promise.resolve();
  });

  // Move all 7 right-arm joints by >= 10 deg to match Pose 2
  mockPositions = {
    "right_joint0.pos": -25.0,
    "right_joint1.pos": 35.0,
    "right_joint2.pos": 25.0,
    "right_joint3.pos": -30.0,
    "right_joint4.pos": 38.0,
    "right_joint5.pos": 28.0,
    "right_joint6.pos": 33.0,
    "right_gripper.pos": 50,
  };
  await act(async () => {
    await new Promise((r) => setTimeout(r, 250));
  });

  const pose2Btn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("ポーズ 2 をまとめて記録"),
  );
  await act(async () => {
    pose2Btn?.click();
    await Promise.resolve();
  });

  // Should transition straight to Live 確認 with all 7 right-arm joints verified
  expect(document.body.textContent).toContain("完了関節: 7 / 14");

  // Flip joint0 direction and swap gripper open/closed in Live Mirror
  const flipBtn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("向き反転"),
  );
  const swapGripperBtn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("開閉入れ替え"),
  );
  await act(async () => {
    flipBtn?.click();
    swapGripperBtn?.click();
    await Promise.resolve();
  });

  const saveBtn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("Save Mapping YAML & Apply"),
  );
  await act(async () => {
    saveBtn?.click();
    await Promise.resolve();
    await Promise.resolve();
  });

  expect(savedBodies).toHaveLength(1);
  expect(Object.keys(savedBodies[0].joints ?? {})).toHaveLength(7);
  expect(savedBodies[0].joints?.right_joint0?.expected_model_direction).toBe(-1);
  expect(savedBodies[0].view_gripper_deg?.right).toEqual({ closed: -45, open: 0 });
  expect(onSaved).toHaveBeenCalledWith("follower-mapping.yaml");

  act(() => root.unmount());
});
