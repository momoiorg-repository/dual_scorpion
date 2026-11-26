import { useEffect, useRef, useState, useCallback } from "react";
import { URDFViewerElement } from "@/lib/urdfViewerHelpers";
import { useApi } from "@/contexts/ApiContext";
import type { RobotModel } from "@/hooks/useRobots";
import { computeScorpionUrdfJoints, type ScorpionVisualMapping } from "@/lib/scorpionVisualMapping";

interface JointData {
  type: "joint_update";
  joints: Record<string, number>;
  timestamp: number;
  robot_model?: RobotModel;
  motor_positions?: Record<string, number>;
}

interface UseRealTimeJointsProps {
  viewerRef: React.RefObject<URDFViewerElement>;
  enabled?: boolean;
  websocketUrl?: string;
  robotModel?: RobotModel;
  visualMapping?: ScorpionVisualMapping;
}

const INITIAL_RECONNECT_DELAY_MS = 1000;
const MAX_RECONNECT_DELAY_MS = 30000;

export const useRealTimeJoints = ({
  viewerRef,
  enabled = true,
  websocketUrl,
  robotModel = "so101",
  visualMapping,
}: UseRealTimeJointsProps) => {
  const { wsBaseUrl } = useApi();
  const finalWebSocketUrl = websocketUrl || `${wsBaseUrl}/ws/joint-data`;

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectDelayRef = useRef(INITIAL_RECONNECT_DELAY_MS);
  const intentionallyClosedRef = useRef(false);
  const visualMappingRef = useRef<ScorpionVisualMapping | undefined>(visualMapping);
  visualMappingRef.current = visualMapping;
  const [isConnected, setIsConnected] = useState(false);
  const [motorPositions, setMotorPositions] = useState<Record<string, number>>({});
  const [mappedJoints, setMappedJoints] = useState(0);

  const updateJointValues = useCallback(
    (joints: Record<string, number>) => {
      const viewer = viewerRef.current;
      if (!viewer || typeof viewer.setJointValue !== "function") return;
      Object.entries(joints).forEach(([jointName, value]) => {
        try {
          viewer.setJointValue(jointName, value);
        } catch (error) {
          console.warn(`Failed to set joint ${jointName}:`, error);
        }
      });
    },
    [viewerRef]
  );

  useEffect(() => {
    if (!enabled) return;

    intentionallyClosedRef.current = false;

    const connect = () => {
      if (intentionallyClosedRef.current) return;

      let ws: WebSocket;
      try {
        ws = new WebSocket(finalWebSocketUrl);
      } catch (error) {
        console.error("Failed to create WebSocket:", error);
        scheduleReconnect();
        return;
      }
      wsRef.current = ws;

      ws.onopen = () => {
        setIsConnected(true);
        reconnectDelayRef.current = INITIAL_RECONNECT_DELAY_MS;
        if (reconnectTimeoutRef.current) {
          clearTimeout(reconnectTimeoutRef.current);
          reconnectTimeoutRef.current = null;
        }
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data) as JointData;
          if (data.type === "joint_update" && data.joints && (data.robot_model ?? "so101") === robotModel) {
            const backendJoints = data.joints;
            const rawMotors = data.motor_positions ?? {};
            const effectiveJoints =
              robotModel === "dual_scorpion" &&
              Object.keys(backendJoints).length === 0 &&
              Object.keys(rawMotors).length > 0 &&
              visualMappingRef.current
                ? computeScorpionUrdfJoints(rawMotors, visualMappingRef.current)
                : backendJoints;
            updateJointValues(effectiveJoints);
            if (robotModel === "dual_scorpion") {
              setMotorPositions(rawMotors);
              setMappedJoints(Object.keys(effectiveJoints).length);
            }
          }
        } catch (error) {
          console.error("Error parsing WebSocket message:", error);
        }
      };

      ws.onclose = (event) => {
        setIsConnected(false);
        wsRef.current = null;
        if (intentionallyClosedRef.current) return;
        if (event.code === 1000) return; // clean close
        scheduleReconnect();
      };

      ws.onerror = () => {
        setIsConnected(false);
      };
    };

    const scheduleReconnect = () => {
      if (reconnectTimeoutRef.current) return;
      const delay = reconnectDelayRef.current;
      reconnectDelayRef.current = Math.min(
        delay * 2,
        MAX_RECONNECT_DELAY_MS
      );
      reconnectTimeoutRef.current = setTimeout(() => {
        reconnectTimeoutRef.current = null;
        connect();
      }, delay);
    };

    connect();

    return () => {
      intentionallyClosedRef.current = true;
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      if (wsRef.current) {
        wsRef.current.close(1000);
        wsRef.current = null;
      }
      setIsConnected(false);
    };
  }, [enabled, finalWebSocketUrl, updateJointValues, robotModel]);

  return { isConnected, motorPositions, mappedJoints };
};
