import React, { useState, useEffect, useCallback, useRef } from "react";
import SensorCard from "./SensorCard";
import PathVisualizer from "./PathVisualizer";
import { formatSensorData, format2DData } from "../utils/formatters";

/**
 * DRModule - Dead Reckoning Module
 * Uses IMU sensors (accelerometer, gyroscope, compass) for position tracking
 */
export default function DRModule() {
    // Raw sensor data
    const [accelerometerData, setAccelerometerData] = useState(null);
    const [gyroscopeData, setGyroscopeData] = useState(null);

    // Dead Reckoning state
    const [velocity, setVelocity] = useState({ x: 0, y: 0 });
    const [position, setPosition] = useState({ x: 0, y: 0 });
    const [path, setPath] = useState([{ x: 0, y: 0 }]);
    const [heading, setHeading] = useState(0);

    // GPS state
    const [gpsLocation, setGpsLocation] = useState(null);
    const [gpsError, setGpsError] = useState(null);
    const watchIdRef = useRef(null);

    // System state
    const [isMonitoring, setIsMonitoring] = useState(false);
    const [error, setError] = useState(null);
    const [isCalibrating, setIsCalibrating] = useState(false);

    // Refs for DR state
    const velRef = useRef(velocity);
    const posRef = useRef(position);
    const pathRef = useRef(path);
    const headingRef = useRef(heading);
    const lastTimestampRef = useRef(null);
    const stationaryCountRef = useRef(0);
    const accelBiasRef = useRef({ x: 0, y: 0 });
    const compassFilterRef = useRef({ lastValue: null, alpha: 0.2 });

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

    // Handle device orientation (compass)
    const handleDeviceOrientation = useCallback((event) => {
        const alpha = event.alpha ?? event.webkitCompassHeading;
        const beta = event.beta;
        const gamma = event.gamma;

        if (
            typeof alpha === "number" &&
            !Number.isNaN(alpha) &&
            typeof beta === "number" &&
            !Number.isNaN(beta) &&
            typeof gamma === "number" &&
            !Number.isNaN(gamma)
        ) {
            let compensatedHeading = alpha;

            // Tilt compensation
            if (Math.abs(beta) > 5 || Math.abs(gamma) > 5) {
                const betaRad = beta * (Math.PI / 180);
                const gammaRad = gamma * (Math.PI / 180);
                const cosB = Math.cos(betaRad);
                const sinB = Math.sin(betaRad);
                const cosG = Math.cos(gammaRad);
                const sinG = Math.sin(gammaRad);

                compensatedHeading =
                    Math.atan2(
                        sinG * cosB * Math.cos((alpha * Math.PI) / 180) +
                        sinB * Math.sin((alpha * Math.PI) / 180),
                        cosG * Math.cos((alpha * Math.PI) / 180)
                    ) *
                    (180 / Math.PI);

                compensatedHeading = (compensatedHeading + 360) % 360;
            }

            // Low-pass filter
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

            // Handle iOS vs Android
            if (event.webkitCompassHeading !== undefined) {
                compensatedHeading = 360 - compensatedHeading;
            } else if (window.screen && window.screen.orientation) {
                const screenOrientation = window.screen.orientation.angle || 0;
                compensatedHeading = (compensatedHeading + screenOrientation) % 360;
            }

            // Detect interference
            const variationThreshold = 20;
            const previousHeading = headingRef.current;
            if (Math.abs(compensatedHeading - previousHeading) > variationThreshold) {
                setIsCalibrating(true);
                setTimeout(() => setIsCalibrating(false), 2000);
            }

            setHeading(compensatedHeading);
        }
    }, []);

    // Handle device motion (acceleration & rotation)
    const handleDeviceMotion = useCallback((event) => {
        // Raw acceleration for display
        if (event.accelerationIncludingGravity) {
            const { x, y, z } = event.accelerationIncludingGravity;
            setAccelerometerData({ x, y, z });
        }

        // Rotation rate for display
        if (event.rotationRate) {
            const { alpha, beta, gamma } = event.rotationRate;
            setGyroscopeData({
                alpha: (alpha || 0) * (180 / Math.PI),
                beta: (beta || 0) * (180 / Math.PI),
                gamma: (gamma || 0) * (180 / Math.PI),
            });
        }

        // Linear acceleration for DR
        const acc = event.acceleration;
        if (!acc) return;

        const now =
            typeof event.timeStamp === "number" ? event.timeStamp : performance.now();
        if (lastTimestampRef.current === null) {
            lastTimestampRef.current = now;
            return;
        }

        const deltaTime = (now - lastTimestampRef.current) / 1000.0;
        if (deltaTime <= 0 || deltaTime < 0.001) return;
        lastTimestampRef.current = now;

        const axRaw = acc.x || 0;
        const ayRaw = acc.y || 0;

        // Stationary detection
        const accMag2D = Math.hypot(axRaw, ayRaw);
        const stationaryThreshold = 0.12;
        const stationarySamplesRequired = 6;

        if (accMag2D < stationaryThreshold) {
            stationaryCountRef.current += 1;
        } else {
            stationaryCountRef.current = 0;
        }

        if (stationaryCountRef.current >= stationarySamplesRequired) {
            accelBiasRef.current.x = accelBiasRef.current.x * 0.8 + axRaw * 0.2;
            accelBiasRef.current.y = accelBiasRef.current.y * 0.8 + ayRaw * 0.2;
            velRef.current = { x: 0, y: 0 };
            setVelocity(velRef.current);
            return;
        }

        // Bias correction
        const axUnbiased = axRaw - accelBiasRef.current.x;
        const ayUnbiased = ayRaw - accelBiasRef.current.y;

        // Deadzone
        const deadzone = 0.05;
        const ax = Math.abs(axUnbiased) > deadzone ? axUnbiased : 0;
        const ay = Math.abs(ayUnbiased) > deadzone ? ayUnbiased : 0;

        // Rotate to world frame
        const headingRad = (headingRef.current || 0) * (Math.PI / 180);
        const worldAx = ax * Math.cos(headingRad) + ay * Math.sin(headingRad);
        const worldAy = -ax * Math.sin(headingRad) + ay * Math.cos(headingRad);

        // Integrate velocity with damping
        const damping = 1.0;
        const newVelX =
            (velRef.current.x + worldAx * deltaTime) * Math.exp(-damping * deltaTime);
        const newVelY =
            (velRef.current.y + worldAy * deltaTime) * Math.exp(-damping * deltaTime);

        // Integrate position
        const newPosX = posRef.current.x + newVelX * deltaTime;
        const newPosY = posRef.current.y + newVelY * deltaTime;

        velRef.current = { x: newVelX, y: newVelY };
        posRef.current = { x: newPosX, y: newPosY };

        setVelocity(velRef.current);
        setPosition(posRef.current);

        const newPath = [...pathRef.current, { x: newPosX, y: newPosY }];
        pathRef.current = newPath;
        setPath(newPath);
    }, []);

    // Start monitoring
    const startMonitoring = async () => {
        setError(null);
        lastTimestampRef.current = null;
        try {
            let motionGranted = false;
            let orientationGranted = false;

            if (typeof DeviceMotionEvent.requestPermission === "function") {
                const motionPermission = await DeviceMotionEvent.requestPermission();
                if (motionPermission === "granted") {
                    motionGranted = true;
                }
            } else {
                motionGranted = true;
            }

            if (typeof DeviceOrientationEvent.requestPermission === "function") {
                const orientationPermission =
                    await DeviceOrientationEvent.requestPermission();
                if (orientationPermission === "granted") {
                    orientationGranted = true;
                }
            } else {
                orientationGranted = true;
            }

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

    // Stop monitoring
    const stopMonitoring = () => {
        window.removeEventListener("devicemotion", handleDeviceMotion);
        window.removeEventListener("deviceorientation", handleDeviceOrientation);
        setIsMonitoring(false);
        lastTimestampRef.current = null;
    };

    // Clear path
    const clearPath = () => {
        setVelocity({ x: 0, y: 0 });
        setPosition({ x: 0, y: 0 });
        setPath([{ x: 0, y: 0 }]);
        lastTimestampRef.current = null;
    };

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            window.removeEventListener("devicemotion", handleDeviceMotion);
            window.removeEventListener("deviceorientation", handleDeviceOrientation);
            if (watchIdRef.current !== null) {
                navigator.geolocation.clearWatch(watchIdRef.current);
            }
        };
    }, [handleDeviceMotion, handleDeviceOrientation]);

    // GPS tracking
    useEffect(() => {
        if (isMonitoring && "geolocation" in navigator) {
            const options = {
                enableHighAccuracy: true,
                timeout: 5000,
                maximumAge: 0,
            };

            const successCallback = (position) => {
                setGpsLocation({
                    latitude: position.coords.latitude,
                    longitude: position.coords.longitude,
                    accuracy: position.coords.accuracy,
                    timestamp: position.timestamp,
                });
                setGpsError(null);
            };

            const errorCallback = (error) => {
                let errorMessage = "GPS error: ";
                switch (error.code) {
                    case error.PERMISSION_DENIED:
                        errorMessage += "User denied GPS access";
                        break;
                    case error.POSITION_UNAVAILABLE:
                        errorMessage += "Location unavailable";
                        break;
                    case error.TIMEOUT:
                        errorMessage += "Request timeout";
                        break;
                    default:
                        errorMessage += "Unknown error";
                }
                setGpsError(errorMessage);
            };

            watchIdRef.current = navigator.geolocation.watchPosition(
                successCallback,
                errorCallback,
                options
            );
        } else if (!isMonitoring && watchIdRef.current !== null) {
            navigator.geolocation.clearWatch(watchIdRef.current);
            watchIdRef.current = null;
        }

        return () => {
            if (watchIdRef.current !== null) {
                navigator.geolocation.clearWatch(watchIdRef.current);
                watchIdRef.current = null;
            }
        };
    }, [isMonitoring]);

    return (
        <div className="bg-gray-900 border-r border-gray-700 p-6 overflow-y-auto">
            {/* Header */}
            <div className="mb-4 text-center">
                <h2 className="text-2xl font-bold text-green-400 mb-2">
                    DR (Dead Reckoning)
                </h2>
                <p className="text-xs text-gray-400">
                    Sensor-based positioning using accelerometer and gyroscope
                </p>
            </div>

            {/* Controls */}
            <div className="mb-4">
                <div className="flex gap-2 flex-wrap justify-center">
                    <button
                        onClick={isMonitoring ? stopMonitoring : startMonitoring}
                        className={`px-4 py-2 rounded-lg font-bold text-sm ${isMonitoring
                                ? "bg-red-500 hover:bg-red-600 text-white"
                                : "bg-blue-500 hover:bg-blue-600 text-white"
                            } transition-colors`}
                    >
                        {isMonitoring ? "Stop Sensors" : "Start Sensors"}
                    </button>
                    <button
                        onClick={clearPath}
                        className="px-4 py-2 rounded-lg font-bold text-sm bg-gray-600 hover:bg-gray-700 text-white transition-colors"
                    >
                        Reset Path
                    </button>
                </div>
                {error && (
                    <p className="text-red-400 text-xs mt-2 text-center">{error}</p>
                )}
                {isCalibrating && (
                    <p className="text-yellow-400 text-xs mt-2 text-center">
                        ⚠️ High magnetic interference detected. Move device in figure-8
                        pattern.
                    </p>
                )}
                {!isMonitoring && !error && (
                    <p className="text-gray-400 text-xs mt-2 text-center">
                        Click "Start Sensors" to begin tracking.
                    </p>
                )}
            </div>

            {/* Path Visualizer */}
            <div className="mb-6">
                <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-2">
                    DR Path Trajectory
                </h3>
                <PathVisualizer path={path} />
            </div>

            {/* GPS Actual Position */}
            <div className="mb-6">
                <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-3">
                    Actual Position (GPS)
                </h3>
                {gpsError ? (
                    <div className="bg-white/10 p-3 rounded-lg mb-2">
                        <p className="text-xs text-red-400">{gpsError}</p>
                    </div>
                ) : gpsLocation ? (
                    <div className="space-y-2">
                        <SensorCard
                            title="GPS Coordinates"
                            dataString={`Lat: ${gpsLocation.latitude.toFixed(
                                6
                            )}°, Lon: ${gpsLocation.longitude.toFixed(6)}°`}
                            unit={`±${gpsLocation.accuracy.toFixed(1)}m`}
                        />
                    </div>
                ) : (
                    <div className="bg-white/10 p-3 rounded-lg mb-2">
                        <p className="text-xs text-yellow-400">
                            {isMonitoring
                                ? "Acquiring GPS signal..."
                                : "Start sensors to enable GPS"}
                        </p>
                    </div>
                )}
            </div>

            {/* Measured Position & Velocity (DR) */}
            <div className="mb-6">
                <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-3">
                    Measured Position (Dead Reckoning)
                </h3>
                <SensorCard
                    title="DR Position (2D)"
                    dataString={format2DData(position)}
                    unit="meters"
                />
                <SensorCard
                    title="Estimated Velocity"
                    dataString={format2DData(velocity)}
                    unit="m/s"
                />
            </div>

            {/* Raw Sensor Data */}
            <div className="mb-6">
                <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-3">
                    Raw Sensor Data
                </h3>
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

            {/* Info */}
            <div className="p-4 text-xs text-gray-400 text-center">
                <p>Inertial Navigation System | IMU-based Dead Reckoning</p>
                <p className="mt-1">
                    <span className="text-green-400">●</span> Sensor Fusion Active
                </p>
            </div>
        </div>
    );
}
