// @vitest-environment jsdom
import React, { act } from "react";
import { createRoot } from "react-dom/client";
import { MemoryRouter, Route, Routes } from "react-router-dom";
import { afterEach, expect, test, vi } from "vitest";

import Calibration from "./Calibration";

const toastMock = vi.fn();
const stableToast = { toast: toastMock };

vi.mock("@/hooks/use-toast", () => ({ useToast: () => stableToast }));
vi.mock("@/components/recording/CameraConfiguration", () => ({
  default: () => null,
}));

const makeFetchMock = (robotState: Record<string, unknown>) =>
  vi.fn(async (input: RequestInfo | URL, init?: RequestInit) => {
    const url = String(input);
    const json = (body: unknown, status = 200) =>
      new Response(JSON.stringify(body), {
        status,
        headers: { "Content-Type": "application/json" },
      });

    if (url.endsWith("/calibration-status")) {
      return json({
        calibration_active: false,
        status: "idle",
        device_type: null,
        error: null,
        message: "",
        step: 0,
        total_steps: 1,
        current_positions: null,
        recorded_ranges: null,
      });
    }
    if (url.includes("/calibration-configs/")) {
      return json({ success: true, configs: [] });
    }
    if (url.endsWith("/available-ports")) {
      return json({
        status: "success",
        ports: ["/dev/tty0", "/dev/ttyS0", "/dev/ttyACM0", "/dev/ttyACM1"],
      });
    }
    if (url.endsWith("/start-port-detection")) {
      return json({
        status: "success",
        data: { ports_before: ["/dev/ttyACM0", "/dev/ttyACM1"] },
      });
    }
    if (url.endsWith("/detect-port-after-disconnect")) {
      return json({ status: "success", port: "/dev/ttyACM1" });
    }
    if (url.endsWith("/robots/scorpion-bot") && init?.method === "POST") {
      const patch = JSON.parse(String(init.body || "{}"));
      Object.assign(robotState, patch);
      return json({ status: "success", robot: { ...robotState } });
    }
    if (url.endsWith("/robots/scorpion-bot")) {
      return json({ status: "success", robot: { ...robotState } });
    }
    return json({ status: "error" }, 404);
  });

let activeFetch: ReturnType<typeof makeFetchMock> | null = null;
const stableFetch = (...args: Parameters<typeof fetch>) => {
  if (!activeFetch) {
    return Promise.resolve(
      new Response(JSON.stringify({ status: "error" }), {
        status: 404,
        headers: { "Content-Type": "application/json" },
      }),
    );
  }
  return activeFetch(...args);
};
const stableApi = {
  baseUrl: "http://api",
  fetchWithHeaders: stableFetch,
};

vi.mock("@/contexts/ApiContext", () => ({
  useApi: () => stableApi,
}));

afterEach(() => {
  vi.useRealTimers();
  document.body.innerHTML = "";
  toastMock.mockReset();
  activeFetch = null;
});

const renderCalibration = async (robotState: Record<string, unknown>) => {
  Object.assign(globalThis, { IS_REACT_ACT_ENVIRONMENT: true });
  activeFetch = makeFetchMock(robotState);
  const host = document.createElement("div");
  document.body.append(host);
  const root = createRoot(host);
  await act(async () => {
    root.render(
      <MemoryRouter
        initialEntries={[{ pathname: "/calibration", state: { robot_name: "scorpion-bot" } }]}
      >
        <Routes>
          <Route path="/calibration" element={<Calibration />} />
        </Routes>
      </MemoryRouter>,
    );
  });
  await act(async () => {
    await Promise.resolve();
    await Promise.resolve();
  });
  return { host, root, fetchMock: activeFetch };
};

test("Dual Scorpion renders dropdown selectors and Find buttons for all 4 ports", async () => {
  const { host, root, fetchMock } = await renderCalibration({
    name: "scorpion-bot",
    robot_model: "dual_scorpion",
    leader_left_port: "",
    leader_right_port: "",
    follower_left_port: "",
    follower_right_port: "",
    leader_port: "",
    follower_port: "",
    leader_config: "",
    follower_config: "",
    cameras: [],
    is_clean: true,
  });

  for (const label of [
    "Leader left port",
    "Leader right port",
    "Follower left port",
    "Follower right port",
  ]) {
    expect(host.querySelector(`[aria-label="Select ${label}"]`)).not.toBeNull();
    expect(host.querySelector(`[aria-label="Find ${label}"]`)).not.toBeNull();
  }
  expect(
    fetchMock.mock.calls.some(([url]) => String(url).endsWith("/available-ports")),
  ).toBe(true);

  act(() => root.unmount());
});

test("Dual Scorpion Find button detects USB disconnect port and saves to robot record without legacy save", async () => {
  vi.useFakeTimers();
  const robotState = {
    name: "scorpion-bot",
    robot_model: "dual_scorpion",
    leader_left_port: "",
    leader_right_port: "",
    follower_left_port: "",
    follower_right_port: "",
    leader_port: "",
    follower_port: "",
    leader_config: "",
    follower_config: "",
    cameras: [],
    is_clean: true,
  };
  const { host, root, fetchMock } = await renderCalibration(robotState);

  const findBtn = host.querySelector(
    '[aria-label="Find Follower right port"]',
  ) as HTMLButtonElement | null;
  expect(findBtn).not.toBeNull();

  await act(async () => {
    findBtn?.click();
    await Promise.resolve();
    await Promise.resolve();
  });

  await act(async () => {
    vi.advanceTimersByTime(2100);
    await Promise.resolve();
    await Promise.resolve();
  });

  expect(robotState.follower_right_port).toBe("/dev/ttyACM1");
  expect(
    fetchMock.mock.calls.some(([url]) => String(url).endsWith("/save-robot-port")),
  ).toBe(false);

  act(() => root.unmount());
});
