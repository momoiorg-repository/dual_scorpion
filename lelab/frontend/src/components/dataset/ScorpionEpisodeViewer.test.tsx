// @vitest-environment jsdom
import React, { act } from "react";
import { createRoot } from "react-dom/client";
import { afterEach, expect, test, vi } from "vitest";

import ScorpionEpisodeViewer from "./ScorpionEpisodeViewer";

const toastMock = vi.fn();
const stableToast = { toast: toastMock };
let lastPassedMapping: unknown = null;

vi.mock("@/hooks/use-toast", () => ({ useToast: () => stableToast }));
vi.mock("@/components/UrdfViewer", () => ({
  default: (props: { visualMapping?: unknown }) => {
    lastPassedMapping = props.visualMapping;
    return <div data-testid="mock-urdf-viewer" />;
  },
}));

const robotState: Record<string, unknown> = {
  name: "test-bot",
  robot_model: "dual_scorpion",
  visual_mapping: null,
};

const fetchMock = vi.fn(async (input: RequestInfo | URL, init?: RequestInit) => {
  const url = String(input);
  const json = (body: unknown, status = 200) =>
    new Response(JSON.stringify(body), {
      status,
      headers: { "Content-Type": "application/json" },
    });

  if (url.includes("/dataset-scorpion-joints")) {
    return json({
      frames: [
        {
          robot_model: "dual_scorpion",
          joints: {},
          motor_positions: {
            "right_joint0.pos": 20,
            "left_joint0.pos": -10,
            "right_gripper.pos": 50,
          },
        },
      ],
    });
  }
  if (url.endsWith("/robots/test-bot") && init?.method === "POST") {
    const patch = JSON.parse(String(init.body || "{}"));
    Object.assign(robotState, patch);
    return json({ status: "success", robot: { ...robotState } });
  }
  if (url.endsWith("/robots/test-bot")) {
    return json({ status: "success", robot: { ...robotState } });
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
  localStorage.clear();
  toastMock.mockReset();
  fetchMock.mockClear();
  lastPassedMapping = null;
});

test("ScorpionEpisodeViewer allows flipping/zeroing and saving 3D alignment without CAD", async () => {
  Object.assign(globalThis, { IS_REACT_ACT_ENVIRONMENT: true });
  localStorage.setItem("lelab.selectedRobot", "test-bot");
  const host = document.createElement("div");
  document.body.append(host);
  const root = createRoot(host);

  await act(async () => {
    root.render(<ScorpionEpisodeViewer repoId="local/demo" episode={0} frameIndex={0} />);
  });

  const showBtn = [...host.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("Show joint viewer"),
  );
  expect(showBtn?.className).toContain("bg-gray-950");

  await act(async () => {
    showBtn?.click();
    await Promise.resolve();
    await Promise.resolve();
  });

  const alignBtn = [...host.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("Adjust 3D alignment"),
  );
  expect(alignBtn).toBeDefined();

  await act(async () => {
    alignBtn?.click();
    await Promise.resolve();
  });

  const zeroAllBtn = [...host.querySelectorAll("button")].find((b) =>
    b.textContent?.includes("Set 0° from frame"),
  );
  await act(async () => {
    zeroAllBtn?.click();
    await Promise.resolve();
  });

  const saveBtn = [...host.querySelectorAll("button")].find((b) => b.textContent?.trim() === "Save");
  await act(async () => {
    saveBtn?.click();
    await Promise.resolve();
    await Promise.resolve();
  });

  expect(robotState.visual_mapping).toBeTruthy();
  const saved = robotState.visual_mapping as { right: { offset_deg: number[] } };
  expect(saved.right.offset_deg[0]).toBe(20);
  expect(lastPassedMapping).toBeTruthy();

  act(() => root.unmount());
});
