import React, {
  useEffect,
  useRef,
  useState,
  useMemo,
  useCallback,
  memo,
} from "react";
import { cn } from "@/lib/utils";

import URDFManipulator from "urdf-loader/src/urdf-manipulator-element.js";
import { useUrdf } from "@/hooks/useUrdf";
import type { RobotModel } from "@/hooks/useRobots";
import { useApi } from "@/contexts/ApiContext";
import { useRealTimeJoints } from "@/hooks/useRealTimeJoints";
import {
  createUrdfViewer,
  fitGridToRobot,
  setupMeshLoader,
  setupJointHighlighting,
  setupModelLoading,
  URDFViewerElement,
} from "@/lib/urdfViewerHelpers";
import {
  DEFAULT_SCORPION_VISUAL_MAPPING,
  computeScorpionUrdfJoints,
  loadLocalVisualMapping,
  normalizeVisualMapping,
  type ScorpionVisualMapping,
} from "@/lib/scorpionVisualMapping";

// Register the URDFManipulator as a custom element if it hasn't been already
if (typeof window !== "undefined" && !customElements.get("urdf-viewer")) {
  customElements.define("urdf-viewer", URDFManipulator);
}
import * as THREE from "three";

const SCORPION_LINK_COLORS: Record<string, number> = {
  base_link: 0x475569,
  r_link_1: 0x64748b,
  r_link_2: 0x94a3b8,
  r_link_3: 0x64748b,
  r_link_4: 0x94a3b8,
  r_link_5: 0x64748b,
  r_link_6: 0x94a3b8,
  r_link_7: 0x64748b,
  r_link_8: 0xf59e0b,
  l_link_1: 0x64748b,
  l_link_2: 0x94a3b8,
  l_link_3: 0x64748b,
  l_link_4: 0x94a3b8,
  l_link_5: 0x64748b,
  l_link_6: 0x94a3b8,
  l_link_7: 0x64748b,
  l_link_8: 0x38bdf8,
};

function tintScorpionRobot(robot: THREE.Object3D): void {
  robot.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) return;
    let cursor: THREE.Object3D | null = mesh;
    let linkName: string | null = null;
    while (cursor) {
      if (cursor.name && cursor.name in SCORPION_LINK_COLORS) {
        linkName = cursor.name;
        break;
      }
      cursor = cursor.parent;
    }
    if (!linkName) return;
    const tint = SCORPION_LINK_COLORS[linkName];
    const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    mats.forEach((mat) => {
      if (mat && "color" in mat && mat.color instanceof THREE.Color) {
        mat.color.setHex(tint);
      }
    });
  });
}

// Extend the interface for the URDF viewer element to include background property
interface UrdfViewerElement extends HTMLElement {
  background?: string;
  setJointValue?: (jointName: string, value: number) => void;
}

interface Props {
  robotModel?: RobotModel;
  recordedFrame?: { joints: Record<string, number>; motor_positions: Record<string, number> };
  visualMapping?: ScorpionVisualMapping;
}

const UrdfViewer: React.FC<Props> = ({ robotModel = "so101", recordedFrame, visualMapping }) => {
  const { baseUrl, fetchWithHeaders } = useApi();
  const scorpion = robotModel === "dual_scorpion";
  const containerRef = useRef<HTMLDivElement>(null);
  const [highlightedJoint, setHighlightedJoint] = useState<string | null>(null);
  const [showJointHud, setShowJointHud] = useState(false);
  const [loadedRobotMapping, setLoadedRobotMapping] = useState<ScorpionVisualMapping | null>(null);
  const { registerUrdfProcessor, alternativeUrdfModels, isDefaultModel } =
    useUrdf();

  const cleanupAnimationRef = useRef<(() => void) | null>(null);
  const viewerRef = useRef<URDFViewerElement | null>(null);
  const hasInitializedRef = useRef<boolean>(false);

  useEffect(() => {
    if (!scorpion || visualMapping) return;
    let selectedName: string | null = null;
    try {
      selectedName = localStorage.getItem("lelab.selectedRobot");
    } catch {
      selectedName = null;
    }
    const local = loadLocalVisualMapping(selectedName);
    if (local) setLoadedRobotMapping(local);
    if (!selectedName) return;
    let cancelled = false;
    fetchWithHeaders(`${baseUrl}/robots/${encodeURIComponent(selectedName)}`)
      .then((res) => (res.ok ? res.json() : null))
      .then((data) => {
        if (cancelled || !data?.robot?.visual_mapping) return;
        setLoadedRobotMapping(normalizeVisualMapping(data.robot.visual_mapping));
      })
      .catch(() => {});
    return () => {
      cancelled = true;
    };
  }, [scorpion, visualMapping, baseUrl, fetchWithHeaders]);

  const activeVisualMapping =
    visualMapping ?? loadedRobotMapping ?? DEFAULT_SCORPION_VISUAL_MAPPING;

  // Real-time joint updates via WebSocket
  const { isConnected, motorPositions, mappedJoints } = useRealTimeJoints({
    viewerRef,
    enabled: isDefaultModel && !recordedFrame,
    robotModel,
    visualMapping: scorpion ? activeVisualMapping : undefined,
  });
  const isWebSocketConnected = isConnected && (!scorpion || Object.keys(motorPositions).length > 0);
  const positions = recordedFrame?.motor_positions ?? motorPositions;
  const backendProvidedJoints = Boolean(recordedFrame && Object.keys(recordedFrame.joints).length > 0);
  const effectiveRecordedJoints = useMemo(() => {
    if (!recordedFrame) return null;
    if (Object.keys(recordedFrame.joints).length > 0) return recordedFrame.joints;
    if (scorpion && Object.keys(recordedFrame.motor_positions).length > 0) {
      return computeScorpionUrdfJoints(recordedFrame.motor_positions, activeVisualMapping);
    }
    return recordedFrame.joints;
  }, [recordedFrame, scorpion, activeVisualMapping]);
  const jointsMapped = effectiveRecordedJoints ? Object.keys(effectiveRecordedJoints).length : mappedJoints;

  useEffect(() => {
    if (!effectiveRecordedJoints) return;
    const viewer = viewerRef.current;
    const showFrame = () => {
      for (const [joint, value] of Object.entries(effectiveRecordedJoints)) {
        viewer?.setJointValue?.(joint, value);
      }
    };
    showFrame();
    viewer?.addEventListener("urdf-processed", showFrame);
    return () => viewer?.removeEventListener("urdf-processed", showFrame);
  }, [effectiveRecordedJoints]);

  // Add state for custom URDF path
  const [customUrdfPath, setCustomUrdfPath] = useState<string | null>(null);
  const [urlModifierFunc, setUrlModifierFunc] = useState<
    ((url: string) => string) | null
  >(null);

  const packageRef = useRef<string>("");

  // Implement UrdfProcessor interface for drag and drop
  const urdfProcessor = useMemo(
    () => ({
      loadUrdf: (urdfPath: string) => {
        setCustomUrdfPath(urdfPath);
      },
      setUrlModifierFunc: (func: (url: string) => string) => {
        setUrlModifierFunc(() => func);
      },
      getPackage: () => {
        return packageRef.current;
      },
    }),
    []
  );

  // Register the URDF processor with the global drag and drop context
  useEffect(() => {
    registerUrdfProcessor(urdfProcessor);
  }, [registerUrdfProcessor, urdfProcessor]);

  // Create URL modifier function for default model
  const defaultUrlModifier = useCallback((url: string) => {
    console.log(`🔗 defaultUrlModifier called with: ${url}`);

    // Handle various package:// URL formats for the default SO-101 model
    if (url.startsWith("package://so_arm_description/meshes/")) {
      const modifiedUrl = url.replace(
        "package://so_arm_description/meshes/",
        "/so-101-urdf/meshes/"
      );
      console.log(`🔗 Modified URL (package): ${modifiedUrl}`);
      return modifiedUrl;
    }

    // Handle case where package path might be partially resolved
    if (url.includes("so_arm_description/meshes/")) {
      const modifiedUrl = url.replace(
        /.*so_arm_description\/meshes\//,
        "/so-101-urdf/meshes/"
      );
      console.log(`🔗 Modified URL (partial): ${modifiedUrl}`);
      return modifiedUrl;
    }

    // Handle the specific problematic path pattern we're seeing in logs
    if (url.includes("/so-101-urdf/so_arm_description/meshes/")) {
      const modifiedUrl = url.replace(
        "/so-101-urdf/so_arm_description/meshes/",
        "/so-101-urdf/meshes/"
      );
      console.log(`🔗 Modified URL (problematic path): ${modifiedUrl}`);
      return modifiedUrl;
    }

    // Handle relative paths that might need mesh folder prefix
    if (
      url.endsWith(".stl") &&
      !url.startsWith("/") &&
      !url.startsWith("http")
    ) {
      const modifiedUrl = `/so-101-urdf/meshes/${url}`;
      console.log(`🔗 Modified URL (relative): ${modifiedUrl}`);
      return modifiedUrl;
    }

    console.log(`🔗 Unmodified URL: ${url}`);
    return url;
  }, []);

  // Main effect to create and setup the viewer only once
  useEffect(() => {
    if (!containerRef.current) return;

    // Create and configure the URDF viewer element
    const viewer = createUrdfViewer(containerRef.current, true);
    viewerRef.current = viewer; // Store reference to the viewer

    // Setup mesh loading function with appropriate URL modifier
    const activeUrlModifier = isDefaultModel
      ? scorpion ? null : defaultUrlModifier
      : urlModifierFunc;
    setupMeshLoader(viewer, activeUrlModifier);

    // Determine which URDF to load - fixed path to match the actual available file
    const urdfPath = isDefaultModel
      ? scorpion ? `${baseUrl}/scorpion-urdf/scorpion.urdf` : "/so-101-urdf/urdf/so101_new_calib.urdf"
      : customUrdfPath || "";

    // Set the package path for the default model
    if (isDefaultModel) {
      packageRef.current = "/"; // Set to root so we can handle full path resolution in URL modifier
    }

    // Setup model loading if a path is available
    let cleanupModelLoading = () => {};
    if (urdfPath) {
      cleanupModelLoading = setupModelLoading(
        viewer,
        urdfPath,
        packageRef.current,
        setCustomUrdfPath,
        alternativeUrdfModels
      );
    }

    // Setup joint highlighting
    const cleanupJointHighlighting = setupJointHighlighting(
      viewer,
      setHighlightedJoint
    );

    // Function to fit the robot to the camera view
    const fitRobotToView = (viewer: URDFViewerElement) => {
      if (!viewer || !viewer.robot) {
        console.log(
          "[RobotViewer] Cannot fit to view: No viewer or robot available"
        );
        return;
      }

      try {
        // Create a bounding box for the robot
        const boundingBox = new THREE.Box3().setFromObject(viewer.robot);

        // Calculate the center of the bounding box
        const center = new THREE.Vector3();
        boundingBox.getCenter(center);

        // Calculate the size of the bounding box
        const size = new THREE.Vector3();
        boundingBox.getSize(size);

        // Get the maximum dimension to ensure the entire robot is visible
        const maxDim = Math.max(size.x, size.y, size.z);

        // Rescale the ground grid to this robot's actual unit scale — it
        // starts as a 1x1 placeholder since it's created before any model
        // is loaded (see createUrdfViewer).
        fitGridToRobot(viewer, center, boundingBox.min.y, maxDim);

        // Position camera to see the center of the model
        viewer.camera.position.copy(center);

        // Move the camera back to see the entire robot
        // Use the model's up direction to determine which axis to move along
        const upVector = new THREE.Vector3();
        if (viewer.up === "+Z" || viewer.up === "Z") {
          upVector.set(1, 1, 1); // Move back in a diagonal
        } else if (viewer.up === "+Y" || viewer.up === "Y") {
          upVector.set(1, 1, 1); // Move back in a diagonal
        } else {
          upVector.set(1, 1, 1); // Default direction
        }

        // Normalize the vector and multiply by the size
        upVector.normalize().multiplyScalar(maxDim * 1.3);
        viewer.camera.position.add(upVector);

        // Make the camera look at the center of the model
        viewer.controls.target.copy(center);

        // Update controls and mark for redraw
        viewer.controls.update();
        viewer.redraw();

        console.log("[RobotViewer] Robot auto-fitted to view");
      } catch (error) {
        console.error("[RobotViewer] Error fitting robot to view:", error);
      }
    };

    // Add event listener for when the robot is loaded to auto-fit to view
    const onRobotLoad = () => {
      fitRobotToView(viewer);
    };

    // Setup animation event handler for the default model or when hasAnimation is true
    const onModelProcessed = () => {
      hasInitializedRef.current = true;
      if (scorpion && viewer.robot) {
        tintScorpionRobot(viewer.robot);
      }
      if ("setJointValue" in viewer) {
        // Clear any existing animation
        if (cleanupAnimationRef.current) {
          cleanupAnimationRef.current();
          cleanupAnimationRef.current = null;
        }
      }
      // Auto-fit the robot to view when the model is processed
      onRobotLoad();
    };

    viewer.addEventListener("urdf-processed", onModelProcessed);

    // Return cleanup function
    return () => {
      if (cleanupAnimationRef.current) {
        cleanupAnimationRef.current();
        cleanupAnimationRef.current = null;
      }
      hasInitializedRef.current = false;
      cleanupJointHighlighting();
      cleanupModelLoading();
      viewer.removeEventListener("urdf-processed", onModelProcessed);
    };
  }, [
    isDefaultModel,
    scorpion,
    baseUrl,
    customUrdfPath,
    urlModifierFunc,
    defaultUrlModifier,
    alternativeUrdfModels,
  ]);

  return (
    <div
      className={cn(
        "w-full h-full transition-all duration-300 ease-in-out relative",
        "bg-gradient-to-br from-gray-900 to-gray-800"
      )}
    >
      <div ref={containerRef} className="w-full h-full" />

      {/* Joint highlight indicator */}
      {highlightedJoint && (
        <div className="absolute bottom-4 right-4 bg-black/70 text-white px-3 py-2 rounded-md text-sm font-mono z-10">
          Joint: {highlightedJoint}
        </div>
      )}

      {scorpion && Object.keys(positions).length > 0 && (
        showJointHud ? (
          <div className="absolute bottom-2 left-2 z-10 max-h-[45%] overflow-auto rounded-md bg-black/80 p-2 text-xs text-gray-200">
            <div className="mb-1 flex items-center justify-between gap-3 border-b border-white/10 pb-1">
              <span className="font-medium text-[11px] text-gray-300">Joint INFO</span>
              <button
                type="button"
                onClick={() => setShowJointHud(false)}
                className="rounded px-1.5 py-0.5 text-[10px] text-gray-400 hover:bg-white/10 hover:text-white"
              >
                Hide
              </button>
            </div>
            <div className="grid grid-cols-2 gap-x-4">
              {Object.entries(positions).sort(([a], [b]) => a.localeCompare(b)).map(([joint, value]) => (
                <div key={joint} className="font-mono">
                  {joint.replace(".pos", "").replace(/_/g, " ")} {value.toFixed(1)}{joint.endsWith("gripper.pos") ? "%" : "°"}
                </div>
              ))}
            </div>
            {recordedFrame && !backendProvidedJoints && jointsMapped > 0 && (
              <p className="mt-1 text-[11px] text-sky-300">
                Visual preview mapping active (no CAD verification required).
              </p>
            )}
            {jointsMapped < Object.keys(positions).length && (
              <p className="mt-1 text-amber-300">A verified mapping is needed to animate the remaining joints.</p>
            )}
          </div>
        ) : (
          <button
            type="button"
            onClick={() => setShowJointHud(true)}
            className="absolute bottom-2 left-2 z-10 rounded bg-black/60 px-2 py-1 text-[11px] font-medium text-gray-300 hover:bg-black/80 hover:text-white"
          >
            Joint INFO ({Object.keys(positions).length})
          </button>
        )
      )}

      {/* WebSocket connection status */}
      {isDefaultModel && !recordedFrame && (
        <div className="absolute top-4 right-4 z-10">
          <div
            className={`flex items-center gap-2 px-3 py-2 rounded-md text-sm font-mono ${
              isWebSocketConnected
                ? "bg-green-900/70 text-green-300"
                : "bg-red-900/70 text-red-300"
            }`}
          >
            <div
              className={`w-2 h-2 rounded-full ${
                isWebSocketConnected ? "bg-green-400" : "bg-red-400"
              }`}
            />
            {isWebSocketConnected ? "Live Robot Data" : "Disconnected"}
          </div>
        </div>
      )}
    </div>
  );
};

export default memo(UrdfViewer);
