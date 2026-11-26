import { lazy, Suspense, useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { useApi } from "@/contexts/ApiContext";

const UrdfViewer = lazy(() => import("@/components/UrdfViewer"));
type Frame = { joints: Record<string, number>; motor_positions: Record<string, number> };

export default function ScorpionEpisodeViewer({
  repoId, episode, frameIndex,
}: {
  repoId: string;
  episode: number;
  frameIndex: number;
}) {
  const { baseUrl, fetchWithHeaders } = useApi();
  const [open, setOpen] = useState(false);
  const [frames, setFrames] = useState<Frame[]>([]);
  const [error, setError] = useState("");

  useEffect(() => {
    if (!open) return;
    const controller = new AbortController();
    const params = new URLSearchParams({ repo_id: repoId, episode_index: String(episode) });
    try {
      const robot = localStorage.getItem("lelab.selectedRobot");
      if (robot) params.set("robot_name", robot);
    } catch {
      // Browser storage is optional.
    }
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
  return (
    <div className="space-y-2">
      <Button size="sm" variant="outline" onClick={() => setOpen((value) => !value)}>
        {open ? "Hide joint viewer" : "Show joint viewer"}
      </Button>
      {open && (
        error ? <p role="alert" className="text-sm text-red-400">{error}</p>
          : frame ? (
            <div className="h-80 overflow-hidden rounded-md border border-gray-800">
              <Suspense fallback={<p className="p-4 text-gray-400">Loading robot model…</p>}>
                <UrdfViewer robotModel="dual_scorpion" recordedFrame={frame} />
              </Suspense>
            </div>
          ) : <p className="text-sm text-gray-400">Loading joint positions…</p>
      )}
    </div>
  );
}
