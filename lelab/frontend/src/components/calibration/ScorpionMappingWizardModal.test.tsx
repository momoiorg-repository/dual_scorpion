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
  default: () => <div data-testid="wizard-urdf-viewer" />,
}));

let mockMotorValue = 10.0;
const savedBodies: unknown[] = [];

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
      positions: {
        "right_joint0.pos": mockMotorValue,
        "right_gripper.pos": 0,
      },
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
  mockMotorValue = 10.0;
});

test("ScorpionMappingWizardModal connects follower, captures 2 poses, and saves YAML without CAD input", async () => {
  Object.assign(globalThis, { IS_REACT_ACT_ENVIRONMENT: true });
  const robot: RobotRecord = {
    name: "test-scorp",
    robot_model: "dual_scorpion",
    leader_port: "",
    follower_port: "",
    leader_left_port: "",
    leader_right_port: "",
    follower_left_port: "/dev/ttyACM1",
    follower_right_port: "/dev/ttyACM0",
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
    b.textContent?.includes("画面と同じ姿勢で記録"),
  );
  await act(async () => {
    pose1Btn?.click();
    await Promise.resolve();
  });

  mockMotorValue = 38.0;
  await act(async () => {
    await new Promise((r) => setTimeout(r, 250));
  });

  const pose2Btn = [...document.body.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("動かした先を記録"),
  );
  await act(async () => {
    pose2Btn?.click();
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
  expect(onSaved).toHaveBeenCalledWith("follower-mapping.yaml");

  act(() => root.unmount());
});
