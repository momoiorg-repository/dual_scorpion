import React, { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { useApi } from "@/contexts/ApiContext";
import { useToast } from "@/hooks/use-toast";
import type { RobotRecord } from "@/hooks/useRobots";
import { SCORPION_ARM_JOINTS, type ArmSide } from "@/lib/scorpionVisualMapping";

const UrdfViewer = lazy(() => import("@/components/UrdfViewer"));

interface JointCapture {
  model_reference_deg: number;
  motor_reference_deg: number;
  start_motor_deg: number;
  moved_motor_deg?: number;
  expected_model_direction: 1 | -1;
}

interface Props {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  robot: RobotRecord;
  onSaved: (filename: string) => void;
}

const REFERENCE_PRESETS = [-45, 0, 45] as const;

export const ScorpionMappingWizardModal: React.FC<Props> = ({
  open,
  onOpenChange,
  robot,
  onSaved,
}) => {
  const { baseUrl, fetchWithHeaders } = useApi();
  const { toast } = useToast();

  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState("");
  const [positions, setPositions] = useState<Record<string, number>>({});

  const [side, setSide] = useState<ArmSide>("right");
  const [selectedSlot, setSelectedSlot] = useState<number | "gripper">(0);
  const [previewPhase, setPreviewPhase] = useState<"pose1" | "pose2">("pose1");
  const [referenceDeg, setReferenceDeg] = useState<number>(0);
  const [moveDir, setMoveDir] = useState<1 | -1>(1);

  const [captures, setCaptures] = useState<Record<string, JointCapture>>({});
  const [gripperLimits, setGripperLimits] = useState<
    Record<ArmSide, { closed?: number; open?: number }>
  >({ right: {}, left: {} });

  const connectedRef = useRef(false);
  connectedRef.current = connected;

  const stopSession = useCallback(async () => {
    if (!connectedRef.current) return;
    connectedRef.current = false;
    setConnected(false);
    try {
      await fetchWithHeaders(`${baseUrl}/scorpion-mapping/stop`, { method: "POST" });
    } catch {
      /* best-effort disconnect */
    }
  }, [baseUrl, fetchWithHeaders]);

  useEffect(() => {
    if (!open) {
      void stopSession();
      setError("");
      return;
    }
    return () => {
      void stopSession();
    };
  }, [open, stopSession]);

  useEffect(() => {
    if (!open || !connected) return;
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetchWithHeaders(`${baseUrl}/scorpion-mapping/positions`);
        const data = await res.json();
        if (!cancelled && data.success && data.positions) {
          setPositions(data.positions);
        }
      } catch {
        /* retry next tick */
      }
    };
    void poll();
    const timer = window.setInterval(() => void poll(), 200);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [open, connected, baseUrl, fetchWithHeaders]);

  const handleConnect = async () => {
    setConnecting(true);
    setError("");
    try {
      const res = await fetchWithHeaders(`${baseUrl}/scorpion-mapping/start`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          follower_left_port: robot.follower_left_port ?? "",
          follower_right_port: robot.follower_right_port ?? "",
          follower_config: robot.follower_config ?? "",
        }),
      });
      const data = await res.json();
      if (!res.ok || !data.success) {
        throw new Error(data.message || "Could not connect follower.");
      }
      setConnected(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setConnecting(false);
    }
  };

  const activeJointName =
    selectedSlot === "gripper" ? `${side}_gripper` : `${side}_joint${selectedSlot}`;
  const activeCapture = captures[activeJointName];
  const liveValue = positions[`${activeJointName}.pos`];

  const targetUrdfJoints = useMemo(() => {
    const joints: Record<string, number> = {};
    for (const s of ["right", "left"] as const) {
      const suffix = s === "right" ? "R" : "L";
      for (let i = 0; i < 7; i++) {
        let deg = 0;
        if (s === side && selectedSlot === i) {
          deg = previewPhase === "pose1" ? referenceDeg : referenceDeg + moveDir * 30;
        }
        joints[`joint${i}_${suffix}`] = (deg * Math.PI) / 180;
      }
      const gripDeg =
        s === side && selectedSlot === "gripper"
          ? previewPhase === "pose1"
            ? (gripperLimits[s].closed ?? 0)
            : (gripperLimits[s].open ?? 45)
          : 0;
      joints[`joint7_${suffix}`] = (gripDeg * Math.PI) / 180;
    }
    return joints;
  }, [side, selectedSlot, previewPhase, referenceDeg, moveDir, gripperLimits]);

  const recordedFrame = useMemo(
    () => ({ joints: targetUrdfJoints, motor_positions: positions }),
    [targetUrdfJoints, positions],
  );

  const handleCaptureAllNeutral = () => {
    setCaptures((prev) => {
      const next = { ...prev };
      for (let i = 0; i < 7; i++) {
        const key = `${side}_joint${i}`;
        const val = positions[`${key}.pos`];
        if (typeof val !== "number" || !Number.isFinite(val)) continue;
        next[key] = {
          model_reference_deg: 0,
          motor_reference_deg: val,
          start_motor_deg: val,
          moved_motor_deg: prev[key]?.moved_motor_deg,
          expected_model_direction: prev[key]?.expected_model_direction ?? 1,
        };
      }
      return next;
    });
    setPreviewPhase("pose2");
    toast({
      title: `Captured ${side} neutral pose`,
      description: "Now wiggle each joint one at a time in the 3D direction.",
    });
  };

  const handleCapturePose1 = () => {
    if (typeof liveValue !== "number" || !Number.isFinite(liveValue)) return;
    setCaptures((prev) => ({
      ...prev,
      [activeJointName]: {
        model_reference_deg: referenceDeg,
        motor_reference_deg: liveValue,
        start_motor_deg: liveValue,
        moved_motor_deg: prev[activeJointName]?.moved_motor_deg,
        expected_model_direction: moveDir,
      },
    }));
    setPreviewPhase("pose2");
  };

  const handleCapturePose2 = () => {
    if (typeof liveValue !== "number" || !Number.isFinite(liveValue) || !activeCapture) return;
    const delta = Math.abs(liveValue - activeCapture.start_motor_deg);
    if (delta < 10) {
      setError(`${activeJointName}: move the joint at least 10° (currently ${delta.toFixed(1)}°).`);
      return;
    }
    setError("");
    setCaptures((prev) => ({
      ...prev,
      [activeJointName]: {
        ...activeCapture,
        moved_motor_deg: liveValue,
        expected_model_direction: moveDir,
      },
    }));
    if (typeof selectedSlot === "number" && selectedSlot < 6) {
      setSelectedSlot(selectedSlot + 1);
      setPreviewPhase("pose1");
      setReferenceDeg(0);
    } else if (selectedSlot === 6) {
      setSelectedSlot("gripper");
      setPreviewPhase("pose1");
    }
  };

  const readyJoints = Object.entries(captures).filter(
    ([, c]) => typeof c.moved_motor_deg === "number" && Math.abs(c.moved_motor_deg - c.start_motor_deg) >= 10,
  );

  const handleSave = async () => {
    if (readyJoints.length === 0) {
      setError("Complete at least one joint (Pose 1 + Pose 2) before saving.");
      return;
    }
    setSaving(true);
    setError("");
    try {
      const jointsPayload = Object.fromEntries(
        readyJoints.map(([key, c]) => [
          key,
          {
            model_reference_deg: c.model_reference_deg,
            motor_reference_deg: c.motor_reference_deg,
            start_motor_deg: c.start_motor_deg,
            moved_motor_deg: c.moved_motor_deg,
            expected_model_direction: c.expected_model_direction,
          },
        ]),
      );
      const gripperPayload: Record<string, { closed: number; open: number }> = {};
      for (const s of ["right", "left"] as const) {
        gripperPayload[s] = {
          closed: typeof gripperLimits[s].closed === "number" ? gripperLimits[s].closed : 0,
          open: typeof gripperLimits[s].open === "number" ? gripperLimits[s].open : 45,
        };
      }
      const res = await fetchWithHeaders(`${baseUrl}/scorpion-mapping/save`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          robot_name: robot.name,
          follower_left_port: robot.follower_left_port ?? "",
          follower_right_port: robot.follower_right_port ?? "",
          follower_config: robot.follower_config ?? "",
          joints: jointsPayload,
          view_gripper_deg: Object.keys(gripperPayload).length > 0 ? gripperPayload : undefined,
          allow_partial: true,
        }),
      });
      const data = await res.json();
      if (!res.ok || !data.success) {
        throw new Error(data.message || "Could not save mapping YAML.");
      }
      toast({
        title: data.verified ? "Verified mapping saved" : "Partial mapping saved",
        description: `${data.filename} (${data.verification_status})`,
      });
      await stopSession();
      onSaved(data.filename);
      onOpenChange(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setSaving(false);
    }
  };

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-5xl border-slate-700 bg-slate-900 text-white">
        <DialogHeader>
          <DialogTitle>3D Mapping Wizard (CAD知識不要)</DialogTitle>
          <DialogDescription className="text-slate-400">
            画面の 3D ロボットと同じ形・同じ向きに実機の Follower を手で動かすだけで、実測マッピング YAML を自動作成します。
          </DialogDescription>
        </DialogHeader>

        <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
          <div className="flex flex-col gap-2">
            <div className="h-80 overflow-hidden rounded-md border border-slate-700 bg-slate-950">
              <Suspense fallback={<p className="p-4 text-sm text-slate-400">Loading 3D model…</p>}>
                <UrdfViewer robotModel="dual_scorpion" recordedFrame={recordedFrame} />
              </Suspense>
            </div>
            <p className="text-xs text-slate-400">
              {selectedSlot === "gripper"
                ? `3D Preview: ${side}_gripper (${previewPhase === "pose1" ? "Closed" : "Open"})`
                : `3D Preview: ${activeJointName} = ${
                    previewPhase === "pose1" ? referenceDeg : referenceDeg + moveDir * 30
                  }°`}
            </p>
          </div>

          <div className="flex flex-col gap-3 text-sm">
            {!connected ? (
              <div className="space-y-3 rounded-md border border-amber-700/60 bg-amber-950/30 p-3">
                <p className="text-xs text-amber-200">
                  安全のため、接続時に Follower のトルクを解除します。両腕を手で支えられる状態で接続してください。
                </p>
                <Button
                  type="button"
                  onClick={() => void handleConnect()}
                  disabled={connecting}
                  className="w-full bg-sky-600 text-white hover:bg-sky-500"
                >
                  {connecting ? "Connecting Follower..." : "Connect Follower (Torque OFF)"}
                </Button>
              </div>
            ) : (
              <div className="flex items-center justify-between rounded-md border border-emerald-700/60 bg-emerald-950/30 px-3 py-2 text-xs">
                <span className="text-emerald-300">Follower connected (Torque OFF)</span>
                <Button
                  type="button"
                  size="sm"
                  variant="outline"
                  onClick={() => void stopSession()}
                  className="h-7 border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
                >
                  Disconnect
                </Button>
              </div>
            )}

            <div className="flex items-center justify-between gap-2">
              <div className="flex gap-1">
                {(["right", "left"] as const).map((s) => (
                  <button
                    key={s}
                    type="button"
                    onClick={() => {
                      setSide(s);
                      setPreviewPhase("pose1");
                    }}
                    className={`rounded px-3 py-1 text-xs font-medium capitalize ${
                      side === s
                        ? "bg-sky-600 text-white"
                        : "bg-slate-800 text-slate-300 hover:bg-slate-700"
                    }`}
                  >
                    {s} arm
                  </button>
                ))}
              </div>
              <Button
                type="button"
                size="sm"
                variant="outline"
                disabled={!connected}
                onClick={handleCaptureAllNeutral}
                className="h-7 border-slate-600 bg-slate-800 text-xs text-slate-200 hover:bg-slate-700"
              >
                全7軸 Pose 1 (0°) をまとめて記録
              </Button>
            </div>

            <div className="flex flex-wrap gap-1.5">
              {SCORPION_ARM_JOINTS.map((j, idx) => {
                const cap = captures[`${side}_${j}`];
                const done =
                  cap &&
                  typeof cap.moved_motor_deg === "number" &&
                  Math.abs(cap.moved_motor_deg - cap.start_motor_deg) >= 10;
                return (
                  <button
                    key={j}
                    type="button"
                    onClick={() => {
                      setSelectedSlot(idx);
                      setPreviewPhase("pose1");
                    }}
                    className={`rounded border px-2 py-1 font-mono text-xs ${
                      selectedSlot === idx
                        ? "border-sky-400 bg-sky-900/50 text-white"
                        : done
                        ? "border-emerald-700 bg-emerald-950/40 text-emerald-300"
                        : "border-slate-700 bg-slate-800 text-slate-300"
                    }`}
                  >
                    {j} {done ? "✓" : ""}
                  </button>
                );
              })}
              <button
                type="button"
                onClick={() => {
                  setSelectedSlot("gripper");
                  setPreviewPhase("pose1");
                }}
                className={`rounded border px-2 py-1 font-mono text-xs ${
                  selectedSlot === "gripper"
                    ? "border-sky-400 bg-sky-900/50 text-white"
                    : "border-slate-700 bg-slate-800 text-slate-300"
                }`}
              >
                gripper
              </button>
            </div>

            {selectedSlot === "gripper" ? (
              <div className="space-y-3 rounded-md border border-slate-700 bg-slate-800/60 p-3 text-xs">
                <p className="text-slate-300">
                  {side}_gripper 現在値:{" "}
                  <span className="font-mono text-white">
                    {typeof liveValue === "number" ? `${liveValue.toFixed(1)}%` : "—"}
                  </span>
                </p>
                <div className="flex flex-wrap gap-2">
                  <Button
                    type="button"
                    size="sm"
                    onClick={() => {
                      setGripperLimits((prev) => ({
                        ...prev,
                        [side]: { closed: 0, open: (prev[side].open ?? 45) > 0 ? 45 : -45 },
                      }));
                      setPreviewPhase("pose1");
                    }}
                    className="bg-slate-700 text-white hover:bg-slate-600"
                  >
                    Closed プレビュー (0°)
                  </Button>
                  <Button
                    type="button"
                    size="sm"
                    onClick={() => {
                      setPreviewPhase("pose2");
                    }}
                    className="bg-slate-700 text-white hover:bg-slate-600"
                  >
                    Open プレビュー ({(gripperLimits[side].open ?? 45) > 0 ? "+45°" : "-45°"})
                  </Button>
                  <Button
                    type="button"
                    size="sm"
                    variant="outline"
                    onClick={() => {
                      const nextOpen = (gripperLimits[side].open ?? 45) > 0 ? -45 : 45;
                      setGripperLimits((prev) => ({
                        ...prev,
                        [side]: { closed: 0, open: nextOpen },
                      }));
                      setPreviewPhase("pose2");
                    }}
                    className="border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
                  >
                    開閉向き反転
                  </Button>
                </div>
              </div>
            ) : (
              <div className="space-y-3 rounded-md border border-slate-700 bg-slate-800/60 p-3 text-xs">
                <div className="flex items-center justify-between">
                  <span className="font-mono font-medium text-white">
                    {activeJointName} 現在:{" "}
                    {typeof liveValue === "number" ? `${liveValue.toFixed(1)}°` : "—"}
                  </span>
                  <div className="flex items-center gap-1">
                    {REFERENCE_PRESETS.map((deg) => (
                      <button
                        key={deg}
                        type="button"
                        onClick={() => {
                          setReferenceDeg(deg);
                          setPreviewPhase("pose1");
                        }}
                        className={`rounded px-1.5 py-0.5 font-mono text-[11px] ${
                          referenceDeg === deg
                            ? "bg-sky-600 text-white"
                            : "bg-slate-700 text-slate-300"
                        }`}
                      >
                        {deg}°
                      </button>
                    ))}
                  </div>
                </div>

                <div className="space-y-1.5">
                  <p className="text-slate-300">
                    1. 3D モデルの姿勢（{referenceDeg}°）に合わせて記録します
                  </p>
                  <div className="flex items-center gap-2">
                    <Button
                      type="button"
                      size="sm"
                      disabled={!connected}
                      onClick={handleCapturePose1}
                      className="bg-slate-700 text-white hover:bg-slate-600"
                    >
                      1. 画面と同じ姿勢で記録 (Pose 1)
                    </Button>
                    {activeCapture && (
                      <span className="font-mono text-emerald-300">
                        記録済: {activeCapture.motor_reference_deg.toFixed(1)}°
                      </span>
                    )}
                  </div>
                </div>

                <div className="space-y-1.5">
                  <div className="flex items-center justify-between">
                    <p className="text-slate-300">
                      2. 3D が示す方向へ実機を 10° 以上動かして記録します
                    </p>
                    <button
                      type="button"
                      onClick={() => {
                        setMoveDir((d) => (d === 1 ? -1 : 1));
                        setPreviewPhase("pose2");
                      }}
                      className="rounded border border-slate-600 bg-slate-700 px-2 py-0.5 text-[11px] text-slate-200 hover:bg-slate-600"
                    >
                      向き切替 ({moveDir === 1 ? "+30°" : "-30°"})
                    </button>
                  </div>
                  <div className="flex items-center gap-2">
                    <Button
                      type="button"
                      size="sm"
                      disabled={!connected || !activeCapture}
                      onClick={handleCapturePose2}
                      className="bg-sky-600 text-white hover:bg-sky-500"
                    >
                      2. 動かした先を記録 (Pose 2)
                    </Button>
                    {activeCapture && typeof liveValue === "number" && (
                      <span className="font-mono text-slate-300">
                        Δ {(liveValue - activeCapture.start_motor_deg).toFixed(1)}°
                      </span>
                    )}
                  </div>
                </div>
              </div>
            )}

            {error && (
              <p role="alert" className="rounded border border-red-800 bg-red-950/40 p-2 text-xs text-red-300">
                {error}
              </p>
            )}

            <div className="mt-auto flex items-center justify-between border-t border-slate-800 pt-3">
              <span className="text-xs text-slate-400">
                完了関節: {readyJoints.length} / 14
              </span>
              <div className="flex gap-2">
                <Button
                  type="button"
                  variant="outline"
                  onClick={() => onOpenChange(false)}
                  className="border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
                >
                  Close
                </Button>
                <Button
                  type="button"
                  disabled={saving || readyJoints.length === 0}
                  onClick={() => void handleSave()}
                  className="bg-emerald-600 text-white hover:bg-emerald-500"
                >
                  {saving ? "Saving..." : "Save Mapping YAML & Apply"}
                </Button>
              </div>
            </div>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
};
