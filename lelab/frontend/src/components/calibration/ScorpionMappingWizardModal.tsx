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
import {
  DEFAULT_SCORPION_VISUAL_MAPPING,
  SCORPION_ARM_JOINTS,
  computeScorpionUrdfJoints,
  type ArmSide,
  type ScorpionVisualMapping,
} from "@/lib/scorpionVisualMapping";

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

type WizardStage = "pose1" | "pose2" | "live";

// Non-colliding, high-visibility target pose where all 7 joints differ clearly from 0°.
const POSE2_TARGET_DEG = [35, 35, 30, -35, 45, 35, 45] as const;
const MIN_MOTION_DEG = 10;

function isJointReady(cap: JointCapture | undefined): cap is JointCapture & { moved_motor_deg: number } {
  return Boolean(
    cap &&
      typeof cap.moved_motor_deg === "number" &&
      Math.abs(cap.moved_motor_deg - cap.start_motor_deg) >= MIN_MOTION_DEG,
  );
}

function signFromCapture(cap: JointCapture): 1 | -1 {
  const delta = (cap.moved_motor_deg ?? cap.start_motor_deg) - cap.start_motor_deg;
  return delta * cap.expected_model_direction > 0 ? 1 : -1;
}

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
  const [stage, setStage] = useState<WizardStage>("pose1");
  const [focusJoint, setFocusJoint] = useState<number | null>(null);
  const [focusDir, setFocusDir] = useState<1 | -1>(1);
  const [animatePose, setAnimatePose] = useState(false);
  const [animPhase, setAnimPhase] = useState<"pose1" | "pose2">("pose2");
  const [countdown, setCountdown] = useState<{ target: "pose1" | "pose2"; left: number } | null>(null);
  const [gripperPreview, setGripperPreview] = useState<"live" | "closed" | "open">("live");

  const [captures, setCaptures] = useState<Record<string, JointCapture>>({});
  const [gripperLimits, setGripperLimits] = useState<
    Record<ArmSide, { closed: number; open: number }>
  >({ right: { closed: 0, open: -45 }, left: { closed: 0, open: -45 } });

  const connectedRef = useRef(false);
  connectedRef.current = connected;
  const positionsRef = useRef<Record<string, number>>(positions);
  positionsRef.current = positions;
  const capturesRef = useRef<Record<string, JointCapture>>(captures);
  capturesRef.current = captures;

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
      setCountdown(null);
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

  useEffect(() => {
    if (!open || stage !== "pose2" || !animatePose) {
      setAnimPhase("pose2");
      return;
    }
    const timer = window.setInterval(() => {
      setAnimPhase((prev) => (prev === "pose2" ? "pose1" : "pose2"));
    }, 1200);
    return () => window.clearInterval(timer);
  }, [open, stage, animatePose]);

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

  const capturePose1Now = useCallback(
    (sourcePositions: Record<string, number>, targetSide: ArmSide, advance = true) => {
      const prev = capturesRef.current;
      const next = { ...prev };
      let capturedCount = 0;
      for (let i = 0; i < 7; i++) {
        const key = `${targetSide}_joint${i}`;
        const val = sourcePositions[`${key}.pos`];
        if (typeof val !== "number" || !Number.isFinite(val)) continue;
        capturedCount++;
        next[key] = {
          model_reference_deg: 0,
          motor_reference_deg: val,
          start_motor_deg: val,
          moved_motor_deg: prev[key]?.moved_motor_deg,
          expected_model_direction: prev[key]?.expected_model_direction ?? (POSE2_TARGET_DEG[i] >= 0 ? 1 : -1),
        };
      }
      if (capturedCount === 0) {
        setError(`Could not read ${targetSide} motor positions.`);
        return;
      }
      setCaptures(next);
      setError("");
      setFocusJoint(null);
      if (advance) {
        setStage("pose2");
        setAnimPhase("pose2");
      }
      toast({
        title: `${targetSide === "right" ? "右腕" : "左腕"} ポーズ 1 (L字) を記録しました`,
        description: advance
          ? "つづけて左の 3D ロボットと同じポーズ 2 の形に実機を動かしてください。"
          : "0° の基準角度を更新しました。",
      });
    },
    [toast],
  );

  const capturePose2BatchNow = useCallback(
    (sourcePositions: Record<string, number>, targetSide: ArmSide) => {
      const prev = capturesRef.current;
      const next = { ...prev };
      const missing: number[] = [];
      let updatedCount = 0;
      for (let i = 0; i < 7; i++) {
        const key = `${targetSide}_joint${i}`;
        const base = prev[key];
        const val = sourcePositions[`${key}.pos`];
        if (!base || typeof val !== "number" || !Number.isFinite(val)) {
          missing.push(i);
          continue;
        }
        const delta = Math.abs(val - base.start_motor_deg);
        if (delta >= MIN_MOTION_DEG) {
          updatedCount++;
          next[key] = {
            ...base,
            moved_motor_deg: val,
            expected_model_direction: POSE2_TARGET_DEG[i] >= 0 ? 1 : -1,
          };
        } else if (!isJointReady(base)) {
          missing.push(i);
        }
      }
      setCaptures(next);
      setError("");
      if (missing.length === 0) {
        setFocusJoint(null);
        setStage("live");
        toast({
          title: `${targetSide === "right" ? "右腕" : "左腕"} 7軸すべて記録完了`,
          description: "実機を動かして、画面の 3D ロボットが同じ向き・形で動くか確認してください。",
        });
      } else {
        setFocusJoint(missing[0]);
        setFocusDir(POSE2_TARGET_DEG[missing[0]] >= 0 ? 1 : -1);
        toast({
          title: `${updatedCount}軸を記録（あと ${missing.length} 軸）`,
          description: `動きが小さかった関節 (${missing.map((j) => `joint${j}`).join(", ")}) だけ少し動かして追加記録してください。`,
        });
      }
    },
    [toast],
  );

  const captureSingleJoint = () => {
    if (focusJoint === null) return;
    const key = `${side}_joint${focusJoint}`;
    const prev = capturesRef.current;
    const base = prev[key];
    const val = positions[`${key}.pos`];
    if (!base || typeof val !== "number" || !Number.isFinite(val)) return;
    const delta = Math.abs(val - base.start_motor_deg);
    if (delta < MIN_MOTION_DEG) {
      setError(`${key}: 元の L字から ${MIN_MOTION_DEG}° 以上動かしてください（現在 ${delta.toFixed(1)}°）`);
      return;
    }
    setError("");
    const next = {
      ...prev,
      [key]: {
        ...base,
        moved_motor_deg: val,
        expected_model_direction: focusDir,
      },
    };
    setCaptures(next);
    const nextMissing: number[] = [];
    for (let i = 0; i < 7; i++) {
      if (!isJointReady(next[`${side}_joint${i}`])) nextMissing.push(i);
    }
    if (nextMissing.length === 0) {
      setFocusJoint(null);
      setStage("live");
      toast({ title: `${side === "right" ? "右腕" : "左腕"} 7軸すべて記録完了` });
    } else {
      setFocusJoint(nextMissing[0]);
      setFocusDir(POSE2_TARGET_DEG[nextMissing[0]] >= 0 ? 1 : -1);
    }
  };

  useEffect(() => {
    if (!countdown) return;
    if (countdown.left <= 0) {
      const target = countdown.target;
      setCountdown(null);
      if (target === "pose1") {
        capturePose1Now(positionsRef.current, side);
      } else {
        capturePose2BatchNow(positionsRef.current, side);
      }
      return;
    }
    const timer = window.setTimeout(() => {
      setCountdown((prev) => (prev ? { ...prev, left: prev.left - 1 } : null));
    }, 1000);
    return () => window.clearTimeout(timer);
  }, [countdown, side, capturePose1Now, capturePose2BatchNow]);

  const liveVisualMapping = useMemo<ScorpionVisualMapping>(() => {
    const out: ScorpionVisualMapping = {
      right: {
        sign: [...DEFAULT_SCORPION_VISUAL_MAPPING.right.sign],
        offset_deg: [...DEFAULT_SCORPION_VISUAL_MAPPING.right.offset_deg],
        gripper_closed_deg: gripperLimits.right.closed ?? 0,
        gripper_open_deg: gripperLimits.right.open ?? -45,
      },
      left: {
        sign: [...DEFAULT_SCORPION_VISUAL_MAPPING.left.sign],
        offset_deg: [...DEFAULT_SCORPION_VISUAL_MAPPING.left.offset_deg],
        gripper_closed_deg: gripperLimits.left.closed ?? 0,
        gripper_open_deg: gripperLimits.left.open ?? -45,
      },
    };
    for (const s of ["right", "left"] as const) {
      for (let i = 0; i < 7; i++) {
        const cap = captures[`${s}_joint${i}`];
        if (!cap) continue;
        if (isJointReady(cap)) {
          const sgn = signFromCapture(cap);
          out[s].sign[i] = sgn;
          out[s].offset_deg[i] = cap.motor_reference_deg - sgn * cap.model_reference_deg;
        } else {
          out[s].offset_deg[i] = cap.motor_reference_deg - out[s].sign[i] * cap.model_reference_deg;
        }
      }
    }
    return out;
  }, [captures, gripperLimits]);

  const targetUrdfJoints = useMemo(() => {
    if (stage === "live") {
      const live = computeScorpionUrdfJoints(positions, liveVisualMapping);
      if (gripperPreview !== "live") {
        const suffix = side === "right" ? "R" : "L";
        live[`joint7_${suffix}`] = ((gripperPreview === "open" ? -45 : 0) * Math.PI) / 180;
      }
      return live;
    }
    const joints: Record<string, number> = {};
    for (const s of ["right", "left"] as const) {
      const suffix = s === "right" ? "R" : "L";
      for (let i = 0; i < 7; i++) {
        let deg = 0;
        if (s === side && stage === "pose2") {
          if (focusJoint !== null) {
            deg = i === focusJoint && animPhase === "pose2" ? focusDir * 40 : 0;
          } else {
            deg = animPhase === "pose2" ? POSE2_TARGET_DEG[i] : 0;
          }
        }
        joints[`joint${i}_${suffix}`] = (deg * Math.PI) / 180;
      }
      const gripDeg =
        s === side && stage === "pose2" && focusJoint === null && animPhase === "pose2"
          ? -45
          : 0;
      joints[`joint7_${suffix}`] = (gripDeg * Math.PI) / 180;
    }
    return joints;
  }, [stage, side, focusJoint, focusDir, animPhase, positions, liveVisualMapping, gripperPreview]);

  const recordedFrame = useMemo(
    () => ({ joints: targetUrdfJoints, motor_positions: positions }),
    [targetUrdfJoints, positions],
  );

  const sideHasPose1 = SCORPION_ARM_JOINTS.every((_, i) => Boolean(captures[`${side}_joint${i}`]));
  const sideReadyCount = SCORPION_ARM_JOINTS.filter((_, i) => isJointReady(captures[`${side}_joint${i}`])).length;
  const readyJoints = Object.entries(captures).filter(([, c]) => isJointReady(c));
  const otherSide: ArmSide = side === "right" ? "left" : "right";
  const otherSideReadyCount = SCORPION_ARM_JOINTS.filter((_, i) =>
    isJointReady(captures[`${otherSide}_joint${i}`]),
  ).length;

  const flipJointSign = (jointIdx: number) => {
    const key = `${side}_joint${jointIdx}`;
    setCaptures((prev) => {
      const cap = prev[key];
      if (!cap) return prev;
      return {
        ...prev,
        [key]: {
          ...cap,
          expected_model_direction: cap.expected_model_direction === 1 ? -1 : 1,
        },
      };
    });
  };

  const rezeroSingleJoint = (jointIdx: number) => {
    const key = `${side}_joint${jointIdx}`;
    const liveVal = positions[`${key}.pos`];
    if (typeof liveVal !== "number" || !Number.isFinite(liveVal)) return;
    setCaptures((prev) => {
      const cap = prev[key];
      if (!cap) return prev;
      return {
        ...prev,
        [key]: {
          ...cap,
          model_reference_deg: 0,
          motor_reference_deg: liveVal,
        },
      };
    });
  };

  const handleSave = async () => {
    if (readyJoints.length === 0) {
      setError("最低 1 つ以上の関節（ポーズ 1 + ポーズ 2）を記録してから保存してください。");
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
          open: typeof gripperLimits[s].open === "number" ? gripperLimits[s].open : -45,
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
          view_gripper_deg: gripperPayload,
          allow_partial: true,
        }),
      });
      const data = await res.json();
      if (!res.ok || !data.success) {
        throw new Error(data.message || "Could not save mapping YAML.");
      }
      toast({
        title: data.verified ? "検証済みマッピングを保存しました" : "一部マッピングを保存しました",
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
      <DialogContent className="w-[95vw] max-w-6xl border-slate-700 bg-slate-900 text-white">
        <DialogHeader>
          <DialogTitle>3D Mapping Wizard (CAD知識不要)</DialogTitle>
          <DialogDescription className="text-slate-400">
            左の 3D モデルが示す「ポーズ 1 (L字)」と「ポーズ 2」に実機を手で合わせて記録するだけで全軸を自動計算します。
          </DialogDescription>
        </DialogHeader>

        <div className="grid grid-cols-1 gap-4 lg:grid-cols-12">
          <div className="flex flex-col gap-2 lg:col-span-7">
            <div className="h-[420px] overflow-hidden rounded-md border border-slate-700 bg-slate-950 lg:h-[520px]">
              <Suspense fallback={<p className="p-4 text-sm text-slate-400">Loading 3D model…</p>}>
                <UrdfViewer robotModel="dual_scorpion" recordedFrame={recordedFrame} />
              </Suspense>
            </div>
            <p className="text-xs text-slate-400">
              {stage === "live"
                ? "Live Mirror: 実機を手で動かすと 3D ロボットがリアルタイムに追従します"
                : stage === "pose1"
                ? `3D お手本: ${side === "right" ? "右腕" : "左腕"} ポーズ 1（L字の基準姿勢）`
                : focusJoint !== null
                ? `3D お手本: ${side}_joint${focusJoint} を動かした姿勢`
                : `3D お手本: ${side === "right" ? "右腕" : "左腕"} ポーズ 2（目標姿勢）`}
            </p>
          </div>

          <div className="flex flex-col gap-3 text-sm lg:col-span-5">
            {!connected ? (
              <div className="space-y-3 rounded-md border border-amber-700/60 bg-amber-950/30 p-3">
                <p className="text-xs text-amber-200">
                  安全のため、接続時に Follower のトルクを解除します。腕を手で支えられる状態で接続してください。
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
                {(["right", "left"] as const).map((s) => {
                  const cnt = SCORPION_ARM_JOINTS.filter((_, i) => isJointReady(captures[`${s}_joint${i}`])).length;
                  return (
                    <button
                      key={s}
                      type="button"
                      onClick={() => {
                        setSide(s);
                        setFocusJoint(null);
                        setStage(cnt === 7 ? "live" : SCORPION_ARM_JOINTS.every((_, i) => Boolean(captures[`${s}_joint${i}`])) ? "pose2" : "pose1");
                      }}
                      className={`rounded px-3 py-1 text-xs font-medium ${
                        side === s
                          ? "bg-sky-600 text-white"
                          : "bg-slate-800 text-slate-300 hover:bg-slate-700"
                      }`}
                    >
                      {s === "right" ? "Right arm" : "Left arm"} ({cnt}/7)
                    </button>
                  );
                })}
              </div>
              <div className="flex gap-1 text-[11px]">
                {([
                  ["pose1", "1. L字"],
                  ["pose2", "2. 目標姿勢"],
                  ["live", "3. Live確認"],
                ] as const).map(([key, label]) => (
                  <button
                    key={key}
                    type="button"
                    onClick={() => {
                      setFocusJoint(null);
                      setStage(key);
                    }}
                    className={`rounded px-2 py-1 ${
                      stage === key ? "bg-slate-700 text-white" : "bg-slate-800/70 text-slate-400 hover:text-slate-200"
                    }`}
                  >
                    {label}
                  </button>
                ))}
              </div>
            </div>

            {stage === "pose1" && (
              <div className="space-y-3 rounded-md border border-slate-700 bg-slate-800/60 p-3 text-xs">
                <p className="text-slate-200">
                  <strong>ステップ 1：</strong>
                  実機の {side === "right" ? "右腕" : "左腕"} を、左の 3D ロボットと同じ <strong>L字姿勢</strong>{" "}
                  に合わせて記録してください。
                </p>
                <div className="flex flex-wrap gap-2">
                  <Button
                    type="button"
                    size="sm"
                    disabled={!connected}
                    onClick={() => capturePose1Now(positions, side, true)}
                    className="bg-sky-600 text-white hover:bg-sky-500"
                  >
                    ポーズ 1 (L字) を記録
                  </Button>
                  <Button
                    type="button"
                    size="sm"
                    variant="outline"
                    disabled={!connected}
                    onClick={() =>
                      setCountdown((prev) => (prev ? null : { target: "pose1", left: 3 }))
                    }
                    className="border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
                  >
                    {countdown?.target === "pose1" ? `あと ${countdown.left} 秒... (中止)` : "3秒後に記録 (両手用)"}
                  </Button>
                </div>
              </div>
            )}

            {stage === "pose2" && (
              <div className="space-y-3 rounded-md border border-slate-700 bg-slate-800/60 p-3 text-xs">
                {!sideHasPose1 ? (
                  <p className="text-amber-300">先にステップ 1（ポーズ 1: L字）を記録してください。</p>
                ) : (
                  <>
                    <div className="flex items-center justify-between gap-2">
                      <p className="text-slate-200">
                        <strong>ステップ 2：</strong>
                        {focusJoint === null
                          ? "左の 3D と同じポーズに実機を動かして記録します（±の意識は不要です）。"
                          : `動きが小さかった ${side}_joint${focusJoint} だけを 3D に合わせて追加記録してください。`}
                      </p>
                      <button
                        type="button"
                        onClick={() => setAnimatePose((v) => !v)}
                        className={`shrink-0 rounded border px-2 py-0.5 text-[11px] ${
                          animatePose
                            ? "border-sky-500 bg-sky-900/60 text-sky-200"
                            : "border-slate-600 bg-slate-700 text-slate-200"
                        }`}
                      >
                        {animatePose ? "アニメ停止" : "L字↔ポーズ2 アニメ"}
                      </button>
                    </div>

                    <div className="flex flex-wrap gap-1.5">
                      {SCORPION_ARM_JOINTS.map((j, idx) => {
                        const cap = captures[`${side}_joint${idx}`];
                        const liveVal = positions[`${side}_joint${idx}.pos`];
                        const liveDelta =
                          cap && typeof liveVal === "number" ? Math.abs(liveVal - cap.start_motor_deg) : 0;
                        const ready = isJointReady(cap);
                        const willPass = liveDelta >= MIN_MOTION_DEG;
                        return (
                          <button
                            key={j}
                            type="button"
                            onClick={() => {
                              setFocusJoint((prev) => (prev === idx ? null : idx));
                              setFocusDir(POSE2_TARGET_DEG[idx] >= 0 ? 1 : -1);
                              setAnimPhase("pose2");
                            }}
                            className={`rounded border px-2 py-1 font-mono text-[11px] ${
                              focusJoint === idx
                                ? "border-sky-400 bg-sky-900/60 text-white"
                                : ready || willPass
                                ? "border-emerald-700 bg-emerald-950/40 text-emerald-300"
                                : "border-slate-700 bg-slate-800 text-slate-300"
                            }`}
                          >
                            {j} {ready ? "✓" : `(${liveDelta.toFixed(0)}°)`}
                          </button>
                        );
                      })}
                    </div>

                    {focusJoint === null ? (
                      <div className="flex flex-wrap gap-2">
                        <Button
                          type="button"
                          size="sm"
                          disabled={!connected}
                          onClick={() => capturePose2BatchNow(positions, side)}
                          className="bg-sky-600 text-white hover:bg-sky-500"
                        >
                          ポーズ 2 をまとめて記録
                        </Button>
                        <Button
                          type="button"
                          size="sm"
                          variant="outline"
                          disabled={!connected}
                          onClick={() =>
                            setCountdown((prev) => (prev ? null : { target: "pose2", left: 3 }))
                          }
                          className="border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
                        >
                          {countdown?.target === "pose2"
                            ? `あと ${countdown.left} 秒... (中止)`
                            : "3秒後に記録 (両手用)"}
                        </Button>
                      </div>
                    ) : (
                      <div className="flex flex-wrap items-center gap-2">
                        <Button
                          type="button"
                          size="sm"
                          disabled={!connected}
                          onClick={captureSingleJoint}
                          className="bg-sky-600 text-white hover:bg-sky-500"
                        >
                          {`joint${focusJoint} を追加記録`}
                        </Button>
                        <Button
                          type="button"
                          size="sm"
                          variant="outline"
                          onClick={() => setFocusDir((d) => (d === 1 ? -1 : 1))}
                          className="border-slate-600 bg-slate-800 text-slate-200 hover:bg-slate-700"
                        >
                          反対向きを試す
                        </Button>
                        <Button
                          type="button"
                          size="sm"
                          variant="ghost"
                          onClick={() => setFocusJoint(null)}
                          className="text-slate-400 hover:text-white"
                        >
                          全体ポーズに戻る
                        </Button>
                      </div>
                    )}
                  </>
                )}
              </div>
            )}

            {stage === "live" && (
              <div className="space-y-2 rounded-md border border-slate-700 bg-slate-800/60 p-3 text-xs">
                <div className="flex flex-wrap items-center justify-between gap-2">
                  <span className="text-emerald-300">
                    実機を手で動かして、左の 3D が同じ姿勢になるか確認できます
                  </span>
                  <div className="flex gap-1.5">
                    <Button
                      type="button"
                      size="sm"
                      variant="outline"
                      disabled={!connected}
                      onClick={() => capturePose1Now(positions, side, false)}
                      className="h-7 border-slate-600 bg-slate-800 px-2 text-[11px] text-slate-200 hover:bg-slate-700"
                    >
                      いまの L字 で 0° 更新
                    </Button>
                    {otherSideReadyCount < 7 && (
                      <Button
                        type="button"
                        size="sm"
                        onClick={() => {
                          setSide(otherSide);
                          setFocusJoint(null);
                          setStage("pose1");
                        }}
                        className="h-7 bg-sky-600 px-2 text-[11px] text-white hover:bg-sky-500"
                      >
                        次は {otherSide === "left" ? "Left arm" : "Right arm"} →
                      </Button>
                    )}
                  </div>
                </div>

                <div className="max-h-44 space-y-1 overflow-y-auto pr-1">
                  {SCORPION_ARM_JOINTS.map((j, idx) => {
                    const cap = captures[`${side}_joint${idx}`];
                    const ready = isJointReady(cap);
                    const suffix = side === "right" ? "R" : "L";
                    const liveUrdfRad = targetUrdfJoints[`joint${idx}_${suffix}`];
                    const liveUrdfDeg =
                      typeof liveUrdfRad === "number" ? `${((liveUrdfRad * 180) / Math.PI).toFixed(0)}°` : "—";
                    return (
                      <div
                        key={j}
                        className="flex items-center justify-between rounded bg-slate-900/70 px-2 py-1"
                      >
                        <span className="font-mono text-slate-200">
                          {j} {ready ? "✓" : "(未完了)"}{" "}
                          <span className="text-slate-400">3D:{liveUrdfDeg}</span>
                        </span>
                        <div className="flex gap-1">
                          <button
                            type="button"
                            disabled={!connected || !cap}
                            onClick={() => rezeroSingleJoint(idx)}
                            className="rounded border border-slate-700 bg-slate-800 px-1.5 py-0.5 text-[11px] text-slate-200 hover:bg-slate-700 disabled:opacity-40"
                          >
                            0°に合わせる
                          </button>
                          <button
                            type="button"
                            disabled={!cap}
                            onClick={() => flipJointSign(idx)}
                            className="rounded border border-slate-700 bg-slate-800 px-1.5 py-0.5 text-[11px] text-sky-300 hover:bg-slate-700 disabled:opacity-40"
                          >
                            向き反転
                          </button>
                        </div>
                      </div>
                    );
                  })}
                </div>

                <div className="flex flex-wrap items-center justify-between gap-2 border-t border-slate-700 pt-2">
                  <span className="font-mono text-slate-300">
                    {side}_gripper:{" "}
                    {typeof positions[`${side}_gripper.pos`] === "number"
                      ? `${positions[`${side}_gripper.pos`].toFixed(0)}%`
                      : "—"}
                    {" "}
                    <span className="text-slate-400">
                      ({gripperLimits[side].closed === 0 ? "0%=閉 / 100%=開" : "0%=開 / 100%=閉"})
                    </span>
                  </span>
                  <div className="flex flex-wrap gap-1.5">
                    <Button
                      type="button"
                      size="sm"
                      variant="outline"
                      disabled={!connected || typeof positions[`${side}_gripper.pos`] !== "number"}
                      onClick={() => {
                        const pct = positions[`${side}_gripper.pos`];
                        if (typeof pct !== "number") return;
                        setGripperLimits((prev) => ({
                          ...prev,
                          [side]: pct < 50 ? { closed: 0, open: -45 } : { closed: -45, open: 0 },
                        }));
                        setGripperPreview("live");
                      }}
                      className="h-7 border-slate-600 bg-slate-800 px-2 text-[11px] text-slate-200 hover:bg-slate-700"
                    >
                      実機は今「閉」
                    </Button>
                    <Button
                      type="button"
                      size="sm"
                      variant="outline"
                      disabled={!connected || typeof positions[`${side}_gripper.pos`] !== "number"}
                      onClick={() => {
                        const pct = positions[`${side}_gripper.pos`];
                        if (typeof pct !== "number") return;
                        setGripperLimits((prev) => ({
                          ...prev,
                          [side]: pct >= 50 ? { closed: 0, open: -45 } : { closed: -45, open: 0 },
                        }));
                        setGripperPreview("live");
                      }}
                      className="h-7 border-slate-600 bg-slate-800 px-2 text-[11px] text-slate-200 hover:bg-slate-700"
                    >
                      実機は今「開」
                    </Button>
                    <Button
                      type="button"
                      size="sm"
                      variant="outline"
                      onClick={() => {
                        setGripperLimits((prev) => ({
                          ...prev,
                          [side]: {
                            closed: prev[side].open,
                            open: prev[side].closed,
                          },
                        }));
                        setGripperPreview("live");
                      }}
                      className="h-7 border-slate-600 bg-slate-800 px-2 text-[11px] text-sky-300 hover:bg-slate-700"
                    >
                      開閉入れ替え
                    </Button>
                    <Button
                      type="button"
                      size="sm"
                      variant="outline"
                      onClick={() =>
                        setGripperPreview((p) =>
                          p === "live" ? "open" : p === "open" ? "closed" : "live",
                        )
                      }
                      className="h-7 border-slate-600 bg-slate-800 px-2 text-[11px] text-slate-300 hover:bg-slate-700"
                    >
                      {gripperPreview === "live"
                        ? "開/閉プレビュー"
                        : `プレビュー中:${gripperPreview === "open" ? "開" : "閉"}→Live`}
                    </Button>
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
                完了関節: {readyJoints.length} / 14（{side === "right" ? "右" : "左"}: {sideReadyCount}/7）
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
