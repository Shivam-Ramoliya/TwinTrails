import React, { useState, useEffect, useCallback, useRef } from "react";
import SensorCard from "./components/SensorCard";
import PathVisualizer from "./components/PathVisualizer";
import { formatSensorData, format2DData } from "./utils/formatters";

// --- Main App Component ---

export default function App() {
  // Raw sensor data
  const [accelerometerData, setAccelerometerData] = useState(null); // (With gravity)
  const [gyroscopeData, setGyroscopeData] = useState(null);

  // Dead Reckoning (DR) state
  const [velocity, setVelocity] = useState({ x: 0, y: 0 });
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [path, setPath] = useState([{ x: 0, y: 0 }]); // Store trajectory
  const [heading, setHeading] = useState(0); // Compass heading

  // System state
  const [isMonitoring, setIsMonitoring] = useState(false);
  const [error, setError] = useState(null);

  // Refs to store DR state without triggering re-renders in the motion handler
  const velRef = useRef(velocity);
  const posRef = useRef(position);
  const pathRef = useRef(path);
  const headingRef = useRef(heading);
  const lastTimestampRef = useRef(null);
  // Stationary detection and bias estimation to combat integration drift
  const stationaryCountRef = useRef(0);
  const accelBiasRef = useRef({ x: 0, y: 0 });

  // Update refs whenever state changes
  useEffect(() => {
    velRef.current = velocity;
  }, [velocity]);
  useEffect(() => {
    posRef.current = position;
  }, [position]);
  useEffect(() => {
    pathRef.current = path;
  }, [path]);
  useEffect(() => {
    headingRef.current = heading;
  }, [heading]);

  // --- Event Handlers for Web Sensor APIs ---

  // Compass calibration and filtering state
  const [isCalibrating, setIsCalibrating] = useState(false);
  const compassFilterRef = useRef({ lastValue: null, alpha: 0.2 }); // Low-pass filter

  // Handles orientation (compass heading)
  const handleDeviceOrientation = useCallback((event) => {
    // Get device orientation data
    const alpha = event.alpha ?? event.webkitCompassHeading;
    const beta = event.beta;
    const gamma = event.gamma;

    // Only process if we have valid numeric values
    if (
      typeof alpha === "number" &&
      !Number.isNaN(alpha) &&
      typeof beta === "number" &&
      !Number.isNaN(beta) &&
      typeof gamma === "number" &&
      !Number.isNaN(gamma)
    ) {
      // 1. Compensate for device tilt
      let compensatedHeading = alpha;

      // Apply tilt compensation when device is not flat
      if (Math.abs(beta) > 5 || Math.abs(gamma) > 5) {
        // Convert degrees to radians
        const betaRad = beta * (Math.PI / 180);
        const gammaRad = gamma * (Math.PI / 180);

        // Calculate tilt-compensated heading
        const cosB = Math.cos(betaRad);
        const sinB = Math.sin(betaRad);
        const cosG = Math.cos(gammaRad);
        const sinG = Math.sin(gammaRad);

        // Apply magnetic tilt compensation formula
        compensatedHeading =
          Math.atan2(
            sinG * cosB * Math.cos((alpha * Math.PI) / 180) +
              sinB * Math.sin((alpha * Math.PI) / 180),
            cosG * Math.cos((alpha * Math.PI) / 180)
          ) *
          (180 / Math.PI);

        // Normalize to 0-360 range
        compensatedHeading = (compensatedHeading + 360) % 360;
      }

      // 2. Apply low-pass filter to reduce jitter
      if (compassFilterRef.current.lastValue === null) {
        compassFilterRef.current.lastValue = compensatedHeading;
      } else {
        const alpha = compassFilterRef.current.alpha;
        const filtered =
          alpha * compensatedHeading +
          (1 - alpha) * compassFilterRef.current.lastValue;
        compassFilterRef.current.lastValue = filtered;
        compensatedHeading = filtered;
      }

      // 3. Handle iOS vs Android differences
      if (event.webkitCompassHeading !== undefined) {
        // iOS: webkitCompassHeading is relative to magnetic north
        compensatedHeading = 360 - compensatedHeading;
      } else if (window.screen && window.screen.orientation) {
        // Android: need to account for screen orientation
        const screenOrientation = window.screen.orientation.angle || 0;
        compensatedHeading = (compensatedHeading + screenOrientation) % 360;
      }

      // 4. Detect potential interference
      const variationThreshold = 20; // degrees
      const previousHeading = headingRef.current;
      if (Math.abs(compensatedHeading - previousHeading) > variationThreshold) {
        // Sudden large change might indicate interference
        setIsCalibrating(true);
        setTimeout(() => setIsCalibrating(false), 2000); // Show calibration notice for 2 seconds
      }

      // 5. Update the heading
      setHeading(compensatedHeading);
    }
  }, []);

  // Handles DeviceMotionEvent (acceleration and rotation)
  const handleDeviceMotion = useCallback((event) => {
    // 1. Get raw acceleration (with gravity) for display
    if (event.accelerationIncludingGravity) {
      const { x, y, z } = event.accelerationIncludingGravity;
      setAccelerometerData({ x, y, z });
    }

    // 2. Get rotation rate for display
    if (event.rotationRate) {
      const { alpha, beta, gamma } = event.rotationRate;
      setGyroscopeData({
        alpha: (alpha || 0) * (180 / Math.PI),
        beta: (beta || 0) * (180 / Math.PI),
        gamma: (gamma || 0) * (180 / Math.PI),
      });
    }

    // 3. Get linear acceleration (without gravity) for DR
    const acc = event.acceleration;
    if (!acc) return; // No linear acceleration data

    // Use event.timeStamp (high-res) when available to compute accurate dt
    const now =
      typeof event.timeStamp === "number" ? event.timeStamp : performance.now();
    if (lastTimestampRef.current === null) {
      lastTimestampRef.current = now;
      return; // Wait for next event to get deltaTime
    }

    const deltaTime = (now - lastTimestampRef.current) / 1000.0; // seconds
    if (deltaTime <= 0 || deltaTime < 0.001) return; // Avoid tiny updates
    lastTimestampRef.current = now;

    // Raw 2D acceleration on device axes
    const axRaw = acc.x || 0;
    const ayRaw = acc.y || 0;

    // Stationary detection: if acceleration magnitude is very small for a few samples,
    // consider device stationary and apply zero-velocity update (ZUPT).
    const accMag2D = Math.hypot(axRaw, ayRaw);
    const stationaryThreshold = 0.12; // m/s², tuned empirically
    const stationarySamplesRequired = 6; // number of consecutive small samples

    if (accMag2D < stationaryThreshold) {
      stationaryCountRef.current += 1;
    } else {
      stationaryCountRef.current = 0;
    }

    if (stationaryCountRef.current >= stationarySamplesRequired) {
      // Update a simple bias estimate (low-pass) while stationary
      accelBiasRef.current.x = accelBiasRef.current.x * 0.8 + axRaw * 0.2;
      accelBiasRef.current.y = accelBiasRef.current.y * 0.8 + ayRaw * 0.2;

      // Zero the velocity to prevent drift
      velRef.current = { x: 0, y: 0 };
      setVelocity(velRef.current);
      // Do not integrate position while stationary — keep position stable
      return;
    }

    // Subtract bias estimated during stationary periods
    const axUnbiased = axRaw - accelBiasRef.current.x;
    const ayUnbiased = ayRaw - accelBiasRef.current.y;

    // Apply a small deadzone to suppress tiny noise
    const deadzone = 0.05; // m/s²
    const ax = Math.abs(axUnbiased) > deadzone ? axUnbiased : 0;
    const ay = Math.abs(ayUnbiased) > deadzone ? ayUnbiased : 0;

    // Rotate device acceleration (ax, ay) into world frame (worldAx, worldAy)
    const headingRad = (headingRef.current || 0) * (Math.PI / 180);
    const worldAx = ax * Math.cos(headingRad) + ay * Math.sin(headingRad);
    const worldAy = -ax * Math.sin(headingRad) + ay * Math.cos(headingRad);

    // Integrate acceleration to get velocity with a small damping term to reduce long-term drift
    const damping = 1.0; // per second, exponential decay
    const newVelX =
      (velRef.current.x + worldAx * deltaTime) * Math.exp(-damping * deltaTime);
    const newVelY =
      (velRef.current.y + worldAy * deltaTime) * Math.exp(-damping * deltaTime);

    // Integrate velocity to get position (Symplectic Euler)
    const newPosX = posRef.current.x + newVelX * deltaTime;
    const newPosY = posRef.current.y + newVelY * deltaTime;

    // Update the state-backing refs
    velRef.current = { x: newVelX, y: newVelY };
    posRef.current = { x: newPosX, y: newPosY };

    // Update the actual state for React to render
    setVelocity(velRef.current);
    setPosition(posRef.current);

    // Add new point to path
    const newPath = [...pathRef.current, { x: newPosX, y: newPosY }];
    pathRef.current = newPath;
    setPath(newPath);
  }, []); // Empty deps, relies on refs

  // --- Sensor Monitoring Control ---

  const startMonitoring = async () => {
    setError(null);
    lastTimestampRef.current = null; // Reset timestamp
    try {
      let motionGranted = false;
      let orientationGranted = false;

      // 1. Request Motion Permission
      if (typeof DeviceMotionEvent.requestPermission === "function") {
        const motionPermission = await DeviceMotionEvent.requestPermission();
        if (motionPermission === "granted") {
          motionGranted = true;
        }
      } else {
        // Android / non-iOS 13+
        motionGranted = true;
      }

      // 2. Request Orientation Permission
      if (typeof DeviceOrientationEvent.requestPermission === "function") {
        const orientationPermission =
          await DeviceOrientationEvent.requestPermission();
        if (orientationPermission === "granted") {
          orientationGranted = true;
        }
      } else {
        // Android / non-iOS 13+
        orientationGranted = true;
      }

      // 3. Add listeners if permissions are granted
      if (motionGranted && orientationGranted) {
        window.addEventListener("devicemotion", handleDeviceMotion);
        window.addEventListener("deviceorientation", handleDeviceOrientation);
        setIsMonitoring(true);
      } else {
        setError("Permission to access one or more sensors was denied.");
      }
    } catch (err) {
      setError(`Error starting sensor monitoring: ${err.message}`);
      console.error(err);
    }
  };

  const stopMonitoring = () => {
    window.removeEventListener("devicemotion", handleDeviceMotion);
    window.removeEventListener("deviceorientation", handleDeviceOrientation);
    setIsMonitoring(false);
    lastTimestampRef.current = null;
    // Don't reset DR data, so user can inspect it
  };

  const clearPath = () => {
    // Reset DR state
    setVelocity({ x: 0, y: 0 });
    setPosition({ x: 0, y: 0 });
    setPath([{ x: 0, y: 0 }]); // Reset to origin
    lastTimestampRef.current = null;
  };

  // Cleanup: Remove listeners when component unmounts
  useEffect(() => {
    return () => {
      window.removeEventListener("devicemotion", handleDeviceMotion);
      window.removeEventListener("deviceorientation", handleDeviceOrientation);
    };
  }, [handleDeviceMotion, handleDeviceOrientation]);

  return (
    <div className="flex-1 bg-gray-900 p-6 pt-16 min-h-screen font-sans">
      {/* --- Header --- */}
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-white">TwinTrails</h1>
        <p className="text-lg text-blue-400">
          DR vs. SLAM Localization (Web Preview)
        </p>
      </div>

      {/* --- Controls --- */}
      <div className="mb-6 grid grid-cols-2 gap-4">
        <button
          onClick={isMonitoring ? stopMonitoring : startMonitoring}
          className={`col-span-2 px-4 py-3 rounded-lg font-bold text-lg ${
            isMonitoring
              ? "bg-red-500 hover:bg-red-600 text-white"
              : "bg-blue-500 hover:bg-blue-600 text-white"
          } transition-colors`}
        >
          {isMonitoring ? "Stop Sensors" : "Start Sensors"}
        </button>
        <button
          onClick={clearPath}
          className="col-span-2 px-4 py-2 rounded-lg font-bold text-md bg-gray-600 hover:bg-gray-700 text-white transition-colors"
        >
          Clear Path & Reset DR
        </button>
        {error && (
          <p className="text-red-400 text-sm mt-2 col-span-2">{error}</p>
        )}
        {isCalibrating && (
          <p className="text-yellow-400 text-sm mt-2 col-span-2">
            ⚠️ High magnetic interference detected. Try moving your device in a
            figure-8 pattern away from electronics.
          </p>
        )}
        {!isMonitoring && !error && (
          <p className="text-gray-400 text-sm mt-2 text-center col-span-2">
            Click "Start Sensors" to begin. You may need to grant permissions.
          </p>
        )}
      </div>

      <div className="max-w-2xl mx-auto">
        {/* --- Path Visualizer --- */}
        <PathVisualizer path={path} />

        {/* --- Dead Reckoning (DR) Section --- */}
        <div className="mb-6">
          <h2 className="text-xl font-semibold text-white mb-3">
            Dead Reckoning (Calculated)
          </h2>
          <SensorCard
            title="Position (2D)"
            dataString={format2DData(position)}
            unit="meters"
          />
          <SensorCard
            title="Velocity (2D)"
            dataString={format2DData(velocity)}
            unit="m/s"
          />
        </div>

        {/* --- Raw Sensor Data Section --- */}
        <div className="mb-6">
          <h2 className="text-xl font-semibold text-white mb-3">
            Raw Sensor Data
          </h2>
          <SensorCard
            title="Accelerometer (with Gravity)"
            dataString={formatSensorData(accelerometerData)}
            unit="m/s²"
          />
          <SensorCard
            title="Gyroscope (Rotation Rate)"
            dataString={formatSensorData(gyroscopeData)}
            unit="°/s"
          />
          <SensorCard
            title="Orientation (Compass)"
            dataString={`alpha: ${(heading ?? 0).toFixed(2)}°`}
            unit="0° = North"
          />
        </div>

        {/* --- SLAM Section (Placeholder) --- */}
        <div className="mb-6">
          <h2 className="text-xl font-semibold text-white mb-3">
            SLAM (Vision-Based)
          </h2>
          <div className="bg-white/10 p-4 rounded-lg items-center justify-center min-h-[100px]">
            <p className="text-gray-400 text-center">
              SLAM (WebXR / ARCore) data will be displayed here.
            </p>
            <p className="text-gray-500 text-center text-xs mt-2">
              (Weeks 11-12 Milestone)
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}
