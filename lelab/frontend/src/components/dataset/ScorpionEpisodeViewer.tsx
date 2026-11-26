import { lazy, Suspense, useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { useApi } from "@/contexts/ApiContext";
import { useToast } from "@/hooks/use-toast";
import {
  DEFAULT_SCORPION_VISUAL_MAPPING,
  SCORPION_ARM_JOINTS,
  loadLocalVisualMapping,
  normalizeVisualMapping,
  saveLocalVisualMapping,
  zeroArmToPositions,
  type ArmSide,
  type ScorpionVisualMapping,
} from "@/lib/scorpionVisualMapping";

const UrdfViewer = lazy(() => import("@/components/UrdfViewer"));
type Frame = { joints: Record<string, number>; motor_positions: Record<string, number> };

function readSelectedRobotName(): string | null {
  try {
    return localStorage.getItem("lelab.selectedRobot");
  } catch {
    return null;
  }
}

export default function ScorpionEpisodeViewer({
  repoId, episode, frameIndex,
}: {
  repoId: string;
  episode: number;
  frameIndex: number;
}) {
  const { baseUrl, fetchWithHeaders } = useApi();
  const { toast } = useToast();
  const [open, setOpen] = useState(false);
  const [showAlign, setShowAlign] = useState(false);
  const [activeSide, setActiveSide] = useState<ArmSide>("right");
  const [frames, setFrames] = useState<Frame[]>([]);
  const [error, setError] = useState("");
  const [saving, setSaving] = useState(false);
  const [robotName, setRobotName] = useState<string | null>(() => readSelectedRobotName());
  const [visualMapping, setVisualMapping] = useState<ScorpionVisualMapping>(
    () => loadLocalVisualMapping(readSelectedRobotName()) ?? DEFAULT_SCORPION_VISUAL_MAPPING,
  );

  useEffect(() => {
    if (!open) return;
    const selected = readSelectedRobotName();
    setRobotName(selected);
    const local = loadLocalVisualMapping(selected);
    if (local) setVisualMapping(local);
    if (!selected) return;
    let cancelled = false;
    fetchWithHeaders(`${baseUrl}/robots/${encodeURIComponent(selected)}`)
      .then((res) => (res.ok ? res.json() : null))
      .then((data) => {
        if (cancelled || !data?.robot?.visual_mapping) return;
        const normalized = normalizeVisualMapping(data.robot.visual_mapping);
        setVisualMapping(normalized);
        saveLocalVisualMapping(selected, normalized);
      })
      .catch(() => {});
    return () => {
      cancelled = true;
    };
  }, [open, baseUrl, fetchWithHeaders]);

  useEffect(() => {
    if (!open) return;
    const controller = new AbortController();
    const params = new URLSearchParams({ repo_id: repoId, episode_index: String(episode) });
    const robot = readSelectedRobotName();
    if (robot) params.set("robot_name", robot);
    setFrames([]);
    setError("");
    (async () => {
      try {
        const response = await fetchWithHeaders(`${baseUrl}/dataset-scorpion-joints?${params}`, {
          signal: controller.signal,
        });
        const data = await response.json();
        if (!response.ok) throw new Error(data.detail || "Could not read joint positions.");
        if (!controller.signal.aborted) setFrames(data.frames);
      } catch (cause) {
        if (!controller.signal.aborted) setError(String(cause));
      }
    })();
    return () => controller.abort();
  }, [open, repoId, episode, baseUrl, fetchWithHeaders]);

  const frame = frames[frameIndex];
  const hasBackendMapping = Boolean(frame && Object.keys(frame.joints).length > 0);
  const arm = visualMapping[activeSide];

  const updateArm = (side: ArmSide, patch: Partial<ScorpionVisualMapping[ArmSide]>) => {
    setVisualMapping((prev) => ({
      ...prev,
      [side]: { ...prev[side], ...patch },
    }));
  };

  const handleFlipJoint = (index: number) => {
    const nextSigns = [...arm.sign] as ScorpionVisualMapping[ArmSide]["sign"];
    nextSigns[index] = nextSigns[index] === 1 ? -1 : 1;
    updateArm(activeSide, { sign: nextSigns });
  };

  const handleOffsetChange = (index: number, nextOffset: number) => {
    const nextOffsets = [...arm.offset_deg] as ScorpionVisualMapping[ArmSide]["offset_deg"];
    nextOffsets[index] = nextOffset;
    updateArm(activeSide, { offset_deg: nextOffsets });
  };

  const handleZeroJoint = (index: number) => {
    const val = frame?.motor_positions?.[`${activeSide}_joint${index}.pos`];
    if (typeof val !== "number" || !Number.isFinite(val)) return;
    handleOffsetChange(index, Math.round(val * 10) / 10);
  };

  const handleZeroWholeArm = () => {
    if (!frame?.motor_positions) return;
    setVisualMapping((prev) => zeroArmToPositions(prev, activeSide, frame.motor_positions));
  };

  const handleSave = async () => {
    saveLocalVisualMapping(robotName, visualMapping);
    if (!robotName) {
      toast({ title: "3D alignment saved in browser" });
      return;
    }
    setSaving(true);
    try {
      const response = await fetchWithHeaders(`${baseUrl}/robots/${encodeURIComponent(robotName)}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ visual_mapping: visualMapping }),
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      toast({ title: "3D alignment saved", description: `Saved for "${robotName}".` });
    } catch (cause) {
      toast({
        title: "Saved locally only",
        description: `Could not save to robot record: ${String(cause)}`,
        variant: "destructive",
      });
    } finally {
      setSaving(false);
    }
  };

  const handleReset = async () => {
    setVisualMapping(DEFAULT_SCORPION_VISUAL_MAPPING);
    saveLocalVisualMapping(robotName, null);
    if (robotName) {
      try {
        await fetchWithHeaders(`${baseUrl}/robots/${encodeURIComponent(robotName)}`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ visual_mapping: null }),
        });
      } catch {
        // Best-effort clear on server.
      }
    }
    toast({ title: "3D alignment reset to default" });
  };

  return (
    <div className="space-y-2">
      <div className="flex flex-wrap items-center gap-2">
        <Button
          size="sm"
          variant="outline"
          onClick={() => setOpen((value) => !value)}
          className="border-gray-800 bg-gray-950 text-gray-300 hover:bg-gray-900 hover:text-white"
        >
          {open ? "Hide joint viewer" : "Show joint viewer"}
        </Button>
        {open && frame && !hasBackendMapping && (
          <Button
            size="sm"
            variant="outline"
            onClick={() => setShowAlign((value) => !value)}
            className="border-gray-800 bg-gray-950 text-sky-300 hover:bg-gray-900 hover:text-sky-200"
          >
            {showAlign ? "Hide 3D alignment" : "Adjust 3D alignment"}
          </Button>
        )}
      </div>

      {open && (
        error ? <p role="alert" className="text-sm text-red-400">{error}</p>
          : frame ? (
            <div className="space-y-3">
              <div className="h-80 overflow-hidden rounded-md border border-gray-800">
                <Suspense fallback={<p className="p-4 text-gray-400">Loading robot model…</p>}>
                  <UrdfViewer
                    robotModel="dual_scorpion"
                    recordedFrame={frame}
                    visualMapping={visualMapping}
                  />
                </Suspense>
              </div>

              {showAlign && !hasBackendMapping && (
                <div className="rounded-md border border-gray-800 bg-gray-950 p-3 text-xs text-gray-200 space-y-3">
                  <div className="flex flex-wrap items-center justify-between gap-2">
                    <div>
                      <p className="font-medium text-white">
                        Adjust 3D alignment {robotName ? `(${robotName})` : ""}
                      </p>
                      <p className="text-[11px] text-gray-400">
                        Match the 3D robot visually to your camera video (for display only, no CAD required).
                      </p>
                    </div>
                    <div className="flex flex-wrap items-center gap-1.5">
                      <Button
                        type="button"
                        size="sm"
                        variant="outline"
                        onClick={handleZeroWholeArm}
                        className="h-7 border-gray-700 bg-gray-900 px-2 text-xs text-gray-200 hover:bg-gray-800"
                      >
                        Set 0° from frame
                      </Button>
                      <Button
                        type="button"
                        size="sm"
                        variant="outline"
                        onClick={() => void handleReset()}
                        className="h-7 border-gray-700 bg-gray-900 px-2 text-xs text-gray-300 hover:bg-gray-800"
                      >
                        Reset
                      </Button>
                      <Button
                        type="button"
                        size="sm"
                        disabled={saving}
                        onClick={() => void handleSave()}
                        className="h-7 bg-sky-600 px-3 text-xs text-white hover:bg-sky-500"
                      >
                        {saving ? "Saving..." : "Save"}
                      </Button>
                    </div>
                  </div>

                  <div className="flex gap-1 border-b border-gray-800 pb-2">
                    {(["right", "left"] as const).map((side) => (
                      <button
                        key={side}
                        type="button"
                        onClick={() => setActiveSide(side)}
                        className={`rounded px-3 py-1 text-xs font-medium capitalize transition ${
                          activeSide === side
                            ? "bg-gray-800 text-white"
                            : "text-gray-400 hover:text-gray-200"
                        }`}
                      >
                        {side} arm
                      </button>
                    ))}
                  </div>

                  <div className="grid grid-cols-1 gap-2 sm:grid-cols-2">
                    {SCORPION_ARM_JOINTS.map((joint, idx) => {
                      const rawDeg = frame.motor_positions[`${activeSide}_${joint}.pos`];
                      const sign = arm.sign[idx];
                      const offset = arm.offset_deg[idx];
                      return (
                        <div
                          key={joint}
                          className="flex flex-col gap-1 rounded border border-gray-800 bg-gray-900/60 p-2"
                        >
                          <div className="flex items-center justify-between gap-2">
                            <span className="font-mono font-medium text-gray-200">
                              {activeSide}_{joint}
                              {typeof rawDeg === "number" ? ` (${rawDeg.toFixed(1)}°)` : ""}
                            </span>
                            <div className="flex items-center gap-1">
                              <button
                                type="button"
                                onClick={() => handleFlipJoint(idx)}
                                className="rounded border border-gray-700 bg-gray-800 px-1.5 py-0.5 font-mono text-[11px] text-gray-200 hover:bg-gray-700"
                                title="Flip rotation direction"
                              >
                                {sign === 1 ? "+dir" : "-dir"} Flip
                              </button>
                              <button
                                type="button"
                                onClick={() => handleZeroJoint(idx)}
                                className="rounded border border-gray-700 bg-gray-800 px-1.5 py-0.5 text-[11px] text-gray-200 hover:bg-gray-700"
                                title="Set current frame value as 0°"
                              >
                                Zero
                              </button>
                            </div>
                          </div>
                          <div className="flex items-center gap-2">
                            <input
                              type="range"
                              min={-180}
                              max={180}
                              step={1}
                              value={offset}
                              aria-label={`${activeSide} ${joint} offset`}
                              onChange={(e) => handleOffsetChange(idx, Number(e.target.value))}
                              className="h-1 flex-1 cursor-pointer accent-sky-500"
                            />
                            <span className="w-12 text-right font-mono text-[11px] text-gray-300">
                              {offset.toFixed(0)}°
                            </span>
                          </div>
                        </div>
                      );
                    })}

                    <div className="flex flex-col gap-1 rounded border border-gray-800 bg-gray-900/60 p-2">
                      <div className="flex items-center justify-between gap-2">
                        <span className="font-mono font-medium text-gray-200">
                          {activeSide}_gripper
                          {typeof frame.motor_positions[`${activeSide}_gripper.pos`] === "number"
                            ? ` (${frame.motor_positions[`${activeSide}_gripper.pos`].toFixed(0)}%)`
                            : ""}
                        </span>
                        <button
                          type="button"
                          onClick={() =>
                            updateArm(activeSide, {
                              gripper_closed_deg: arm.gripper_open_deg,
                              gripper_open_deg: arm.gripper_closed_deg,
                            })
                          }
                          className="rounded border border-gray-700 bg-gray-800 px-1.5 py-0.5 text-[11px] text-gray-200 hover:bg-gray-700"
                        >
                          Swap open/closed
                        </button>
                      </div>
                      <div className="flex items-center gap-2">
                        <span className="w-11 text-[11px] text-gray-400">Closed</span>
                        <input
                          type="range"
                          min={-90}
                          max={90}
                          step={1}
                          value={arm.gripper_closed_deg}
                          aria-label={`${activeSide} gripper closed`}
                          onChange={(e) =>
                            updateArm(activeSide, { gripper_closed_deg: Number(e.target.value) })
                          }
                          className="h-1 flex-1 cursor-pointer accent-sky-500"
                        />
                        <span className="w-10 text-right font-mono text-[11px] text-gray-300">
                          {arm.gripper_closed_deg.toFixed(0)}°
                        </span>
                      </div>
                      <div className="flex items-center gap-2">
                        <span className="w-11 text-[11px] text-gray-400">Open</span>
                        <input
                          type="range"
                          min={-90}
                          max={90}
                          step={1}
                          value={arm.gripper_open_deg}
                          aria-label={`${activeSide} gripper open`}
                          onChange={(e) =>
                            updateArm(activeSide, { gripper_open_deg: Number(e.target.value) })
                          }
                          className="h-1 flex-1 cursor-pointer accent-sky-500"
                        />
                        <span className="w-10 text-right font-mono text-[11px] text-gray-300">
                          {arm.gripper_open_deg.toFixed(0)}°
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              )}
            </div>
          ) : <p className="text-sm text-gray-400">Loading joint positions…</p>
      )}
    </div>
  );
}
