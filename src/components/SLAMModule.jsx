import React, { useState, useEffect, useRef, useCallback } from "react";
import PathVisualizer from "./PathVisualizer";
import SensorCard from "./SensorCard";
import { format2DData, formatSensorData } from "../utils/formatters";

/**
 * SLAMModule - Real-time Visual-Inertial SLAM for Mobile Web
 * Combines camera-based visual odometry with IMU sensor fusion
 */
export default function SLAMModule() {
    // Camera & Video state
    const videoRef = useRef(null);
    const canvasRef = useRef(null);
    const processCanvasRef = useRef(null);
    const trajectoryCanvasRef = useRef(null);
    const streamRef = useRef(null);

    // SLAM state
    const [isRunning, setIsRunning] = useState(false);
    const [torchEnabled, setTorchEnabled] = useState(false);
    const [pose, setPose] = useState({ x: 0, y: 0, heading: 0 });
    const [trajectory, setTrajectory] = useState([{ x: 0, y: 0 }]);
    const [landmarkCount, setLandmarkCount] = useState(0);
    const [features, setFeatures] = useState([]);
    const [fps, setFps] = useState(0);
    const [error, setError] = useState(null);

    // GPS state
    const [gpsLocation, setGpsLocation] = useState(null);
    const [gpsError, setGpsError] = useState(null);
    const watchIdRef = useRef(null);

    // Sensor display state
    const [accelerometerData, setAccelerometerData] = useState(null);
    const [gyroscopeData, setGyroscopeData] = useState(null);
    const [orientationData, setOrientationData] = useState(0);

    // Refs for state that doesn't trigger re-renders
    const poseRef = useRef({ x: 0, y: 0, heading: 0 });
    const trajectoryRef = useRef([{ x: 0, y: 0 }]);
    const landmarksRef = useRef([]); // Map of 3D landmarks
    const prevFeaturesRef = useRef(null);
    const prevFrameRef = useRef(null);
    const animationFrameRef = useRef(null);
    const lastFrameTimeRef = useRef(0);

    // IMU state
    const imuDataRef = useRef({
        accel: { x: 0, y: 0, z: 0 },
        gyro: { x: 0, y: 0, z: 0 },
        orientation: 0,
    });
    const imuVelocityRef = useRef({ x: 0, y: 0 });

    // ======================
    // FEATURE EXTRACTION
    // ======================

    /**
     * Extract corner features from grayscale frame using FAST-like detector
     * @param {ImageData} frameData - Grayscale image data
     * @returns {Array} Array of feature points {x, y, score}
     */
    const extractFeatures = useCallback((frameData) => {
        const { width, height, data } = frameData;
        const features = [];
        const threshold = 20; // Lower threshold for more features
        const gridSize = 6; // Even smaller grid for denser detection
        const grid = {};

        // Simple FAST-like corner detector
        // Check pixels in a circle around each point
        const circle = [
            [-3, 0],
            [-3, 1],
            [-2, 2],
            [-1, 3],
            [0, 3],
            [1, 3],
            [2, 2],
            [3, 1],
            [3, 0],
            [3, -1],
            [2, -2],
            [1, -3],
            [0, -3],
            [-1, -3],
            [-2, -2],
            [-3, -1],
        ];

        // Optimize: larger step size for faster processing
        for (let y = 10; y < height - 10; y += 3) {
            for (let x = 10; x < width - 10; x += 3) {
                const centerIdx = y * width + x;
                const centerIntensity = data[centerIdx * 4];

                let brighterCount = 0;
                let darkerCount = 0;

                // Check circle pixels
                for (let i = 0; i < circle.length; i++) {
                    const [dx, dy] = circle[i];
                    const checkIdx = ((y + dy) * width + (x + dx)) * 4;
                    const intensity = data[checkIdx];

                    if (intensity > centerIntensity + threshold) brighterCount++;
                    if (intensity < centerIntensity - threshold) darkerCount++;
                }

                // If enough consecutive pixels are brighter or darker, it's a corner
                if (brighterCount >= 12 || darkerCount >= 12) {
                    const score = Math.max(brighterCount, darkerCount);
                    const gridKey = `${Math.floor(x / gridSize)}_${Math.floor(
                        y / gridSize
                    )}`;

                    // Non-maximum suppression within grid
                    if (!grid[gridKey] || grid[gridKey].score < score) {
                        grid[gridKey] = { x, y, score };
                    }
                }
            }
        }

        // Extract features from grid
        for (let key in grid) {
            features.push(grid[key]);
        }

        return features.slice(0, 2000); // Increase to 2000 features max
    }, []);

    /**
     * Compute descriptor for a feature point (simple patch-based)
     * @param {ImageData} frameData - Image data
     * @param {Object} feature - Feature point {x, y}
     * @returns {Array} Simple descriptor vector
     */
    const computeDescriptor = useCallback((frameData, feature) => {
        const { width, data } = frameData;
        const { x, y } = feature;
        const patchSize = 3; // Reduced from 5 for faster computation
        const descriptor = [];

        for (let dy = -patchSize; dy <= patchSize; dy++) {
            for (let dx = -patchSize; dx <= patchSize; dx++) {
                const idx = ((y + dy) * width + (x + dx)) * 4;
                descriptor.push(data[idx] || 0);
            }
        }

        return descriptor;
    }, []);

    /**
     * Track features between consecutive frames using patch matching
     * @param {ImageData} prevFrame - Previous frame
     * @param {ImageData} currFrame - Current frame
     * @param {Array} prevFeatures - Features from previous frame
     * @returns {Array} Matched feature pairs [{prev, curr}, ...]
     */
    const trackFeatures = useCallback(
        (prevFrame, currFrame, prevFeatures) => {
            const matches = [];
            const searchRadius = 20; // Reduced from 30 for faster search
            const { width, height } = currFrame;

            // Sample only a subset of features for tracking to improve FPS
            const sampleStep = Math.max(1, Math.floor(prevFeatures.length / 300));

            for (let i = 0; i < prevFeatures.length; i += sampleStep) {
                const prevFeat = prevFeatures[i];
                const prevDesc = computeDescriptor(prevFrame, prevFeat);

                let bestMatch = null;
                let bestScore = Infinity;

                // Search for best match in current frame with larger step
                for (let dy = -searchRadius; dy <= searchRadius; dy += 3) {
                    for (let dx = -searchRadius; dx <= searchRadius; dx += 3) {
                        const currX = prevFeat.x + dx;
                        const currY = prevFeat.y + dy;

                        if (
                            currX < 10 ||
                            currX >= width - 10 ||
                            currY < 10 ||
                            currY >= height - 10
                        ) {
                            continue;
                        }

                        const currDesc = computeDescriptor(currFrame, {
                            x: currX,
                            y: currY,
                        });

                        // Compute Sum of Squared Differences (SSD)
                        let ssd = 0;
                        for (let j = 0; j < prevDesc.length; j++) {
                            const diff = prevDesc[j] - currDesc[j];
                            ssd += diff * diff;
                        }

                        if (ssd < bestScore) {
                            bestScore = ssd;
                            bestMatch = { x: currX, y: currY };
                        }
                    }
                }

                // Only accept good matches
                if (bestMatch && bestScore < 3000) {
                    matches.push({
                        prev: prevFeat,
                        curr: bestMatch,
                        score: bestScore,
                    });
                }
            }

            return matches;
        },
        [computeDescriptor]
    );

    /**
     * Estimate pose change from matched features
     * Uses 2D motion estimation (translation + rotation)
     * @param {Array} matches - Matched feature pairs
     * @returns {Object} Pose delta {dx, dy, dHeading}
     */
    const estimatePose = useCallback((matches) => {
        if (matches.length < 5) {
            return { dx: 0, dy: 0, dHeading: 0 };
        }

        // Compute median displacement (robust to outliers)
        const displacements = matches.map((m) => ({
            dx: m.curr.x - m.prev.x,
            dy: m.curr.y - m.prev.y,
        }));

        displacements.sort((a, b) => a.dx - b.dx);
        const medianDx = displacements[Math.floor(displacements.length / 2)].dx;

        displacements.sort((a, b) => a.dy - b.dy);
        const medianDy = displacements[Math.floor(displacements.length / 2)].dy;

        // Estimate rotation from feature flow
        let rotationSum = 0;
        let rotationCount = 0;

        for (let i = 0; i < matches.length; i++) {
            const m = matches[i];
            const angle1 = Math.atan2(m.prev.y - 240, m.prev.x - 320); // Center at 320x240
            const angle2 = Math.atan2(m.curr.y - 240, m.curr.x - 320);
            let dAngle = angle2 - angle1;

            // Normalize angle
            if (dAngle > Math.PI) dAngle -= 2 * Math.PI;
            if (dAngle < -Math.PI) dAngle += 2 * Math.PI;

            if (Math.abs(dAngle) < 0.5) {
                rotationSum += dAngle;
                rotationCount++;
            }
        }

        const dHeading = rotationCount > 0 ? rotationSum / rotationCount : 0;

        // Scale from pixels to meters (rough calibration)
        const pixelToMeter = 0.001; // Adjust based on camera FOV and distance

        return {
            dx: -medianDx * pixelToMeter, // Negative because camera moves opposite
            dy: medianDy * pixelToMeter,
            dHeading: dHeading,
        };
    }, []);

    /**
     * Fuse visual odometry with IMU data using complementary filter
     * @param {Object} visualPose - Pose from visual odometry {dx, dy, dHeading}
     * @param {Object} imuData - IMU sensor data
     * @param {Number} dt - Time delta
     * @returns {Object} Fused pose delta
     */
    const fuseIMU = useCallback((visualPose, imuData, dt) => {
        // Complementary filter weights
        const visualWeight = 0.7;
        const imuWeight = 0.3;

        // Get IMU-based displacement
        const heading = poseRef.current.heading + visualPose.dHeading;
        const cosH = Math.cos(heading);
        const sinH = Math.sin(heading);

        // Integrate IMU velocity
        const imuVel = imuVelocityRef.current;
        const imuDx = imuVel.x * dt * cosH - imuVel.y * dt * sinH;
        const imuDy = imuVel.x * dt * sinH + imuVel.y * dt * cosH;

        // Fuse visual and IMU estimates
        const fusedDx = visualPose.dx * visualWeight + imuDx * imuWeight;
        const fusedDy = visualPose.dy * visualWeight + imuDy * imuWeight;

        // Use gyroscope for heading (more reliable than visual rotation)
        const gyroDHeading = imuData.gyro.z * dt * (Math.PI / 180);
        const fusedDHeading = visualPose.dHeading * 0.3 + gyroDHeading * 0.7;

        return {
            dx: fusedDx,
            dy: fusedDy,
            dHeading: fusedDHeading,
        };
    }, []);

    /**
     * Update map landmarks with new features
     * @param {Array} landmarks - Current map landmarks
     * @param {Array} features - New detected features
     * @param {Object} currentPose - Current robot pose
     * @returns {Array} Updated landmarks
     */
    const updateMap = useCallback((landmarks, features, currentPose) => {
        const updatedLandmarks = [...landmarks];
        const maxLandmarks = 500;
        const matchThreshold = 10; // pixels

        features.forEach((feat) => {
            // Transform feature to world coordinates
            const worldX =
                currentPose.x + feat.x * 0.001 * Math.cos(currentPose.heading);
            const worldY =
                currentPose.y + feat.x * 0.001 * Math.sin(currentPose.heading);

            // Check if landmark already exists
            let matched = false;
            for (let i = 0; i < updatedLandmarks.length; i++) {
                const lm = updatedLandmarks[i];
                const dist = Math.hypot(lm.x - worldX, lm.y - worldY);

                if (dist < matchThreshold * 0.001) {
                    // Update existing landmark
                    lm.x = (lm.x * lm.quality + worldX) / (lm.quality + 1);
                    lm.y = (lm.y * lm.quality + worldY) / (lm.quality + 1);
                    lm.quality = Math.min(lm.quality + 1, 10);
                    matched = true;
                    break;
                }
            }

            // Add new landmark
            if (!matched && updatedLandmarks.length < maxLandmarks) {
                updatedLandmarks.push({
                    id: Date.now() + Math.random(),
                    x: worldX,
                    y: worldY,
                    quality: 1,
                });
            }
        });

        return updatedLandmarks;
    }, []);

    // ======================
    // IMU HANDLERS
    // ======================

    const handleDeviceMotion = useCallback((event) => {
        if (!event.acceleration || !event.rotationRate) return;

        const acc = event.acceleration;
        const gyro = event.rotationRate;

        imuDataRef.current.accel = {
            x: acc.x || 0,
            y: acc.y || 0,
            z: acc.z || 0,
        };

        imuDataRef.current.gyro = {
            x: gyro.alpha || 0,
            y: gyro.beta || 0,
            z: gyro.gamma || 0,
        };

        // Update display state
        setAccelerometerData({
            x: acc.x || 0,
            y: acc.y || 0,
            z: acc.z || 0,
        });

        setGyroscopeData({
            alpha: ((gyro.alpha || 0) * 180) / Math.PI,
            beta: ((gyro.beta || 0) * 180) / Math.PI,
            gamma: ((gyro.gamma || 0) * 180) / Math.PI,
        });

        // Simple velocity integration with damping
        const dt = 0.016; // ~60Hz
        const damping = 0.95;
        imuVelocityRef.current.x =
            (imuVelocityRef.current.x + acc.x * dt) * damping;
        imuVelocityRef.current.y =
            (imuVelocityRef.current.y + acc.y * dt) * damping;
    }, []);

    const handleDeviceOrientation = useCallback((event) => {
        const alpha = event.alpha ?? event.webkitCompassHeading;
        if (typeof alpha === "number" && !Number.isNaN(alpha)) {
            imuDataRef.current.orientation = alpha * (Math.PI / 180);
            setOrientationData(alpha);
        }
    }, []);

    // ======================
    // CAMERA CONTROL
    // ======================

    const startCamera = async () => {
        try {
            setError(null);

            // Request camera access with high frame rate (fallback to lower rates if unavailable)
            let stream;
            try {
                // Try high FPS first
                stream = await navigator.mediaDevices.getUserMedia({
                    video: {
                        facingMode: "environment",
                        width: { ideal: 640 },
                        height: { ideal: 480 },
                        frameRate: { ideal: 120 },
                    },
                });
            } catch (err) {
                // Fallback to standard constraints if high FPS not supported
                stream = await navigator.mediaDevices.getUserMedia({
                    video: {
                        facingMode: "environment",
                        width: { ideal: 640 },
                        height: { ideal: 480 },
                    },
                });
            }

            streamRef.current = stream;
            if (videoRef.current) {
                videoRef.current.srcObject = stream;
                videoRef.current.play();
            }

            // Request IMU permissions (iOS)
            if (typeof DeviceMotionEvent.requestPermission === "function") {
                await DeviceMotionEvent.requestPermission();
            }
            if (typeof DeviceOrientationEvent.requestPermission === "function") {
                await DeviceOrientationEvent.requestPermission();
            }

            // Add IMU listeners
            window.addEventListener("devicemotion", handleDeviceMotion);
            window.addEventListener("deviceorientation", handleDeviceOrientation);

            return true;
        } catch (err) {
            setError(`Camera error: ${err.message}`);
            return false;
        }
    };

    const stopCamera = () => {
        if (streamRef.current) {
            streamRef.current.getTracks().forEach((track) => track.stop());
            streamRef.current = null;
        }

        window.removeEventListener("devicemotion", handleDeviceMotion);
        window.removeEventListener("deviceorientation", handleDeviceOrientation);

        if (videoRef.current) {
            videoRef.current.srcObject = null;
        }

        // Stop GPS tracking
        if (watchIdRef.current !== null) {
            navigator.geolocation.clearWatch(watchIdRef.current);
            watchIdRef.current = null;
        }
    };

    const toggleTorch = async () => {
        if (!streamRef.current) return;

        const track = streamRef.current.getVideoTracks()[0];
        const capabilities = track.getCapabilities();

        if (capabilities.torch) {
            try {
                await track.applyConstraints({
                    advanced: [{ torch: !torchEnabled }],
                });
                setTorchEnabled(!torchEnabled);
            } catch (err) {
                console.error("Torch error:", err);
            }
        }
    };

    // ======================
    // SLAM PROCESSING LOOP
    // ======================

    const processSLAMFrame = useCallback(() => {
        if (!isRunning || !videoRef.current || !canvasRef.current) return;

        const video = videoRef.current;
        const canvas = canvasRef.current;
        const processCanvas = processCanvasRef.current;
        const ctx = canvas.getContext("2d");
        const processCtx = processCanvas.getContext("2d");

        // Calculate FPS
        const now = performance.now();
        const dt = (now - lastFrameTimeRef.current) / 1000;
        lastFrameTimeRef.current = now;
        if (dt > 0) {
            setFps(Math.round(1 / dt));
        }

        // Draw video to canvas
        ctx.drawImage(video, 0, 0, 640, 480);
        const imageData = ctx.getImageData(0, 0, 640, 480);

        // Convert to grayscale
        const grayData = new ImageData(640, 480);
        for (let i = 0; i < imageData.data.length; i += 4) {
            const gray =
                0.299 * imageData.data[i] +
                0.587 * imageData.data[i + 1] +
                0.114 * imageData.data[i + 2];
            grayData.data[i] = grayData.data[i + 1] = grayData.data[i + 2] = gray;
            grayData.data[i + 3] = 255;
        }

        // Extract features
        const features = extractFeatures(grayData);

        // Draw live video on process canvas
        processCtx.drawImage(video, 0, 0, 640, 480);

        // Track features if we have previous frame
        let poseDelta = { dx: 0, dy: 0, dHeading: 0 };
        let matches = [];

        if (prevFrameRef.current && prevFeaturesRef.current) {
            matches = trackFeatures(
                prevFrameRef.current,
                grayData,
                prevFeaturesRef.current
            );

            if (matches.length > 5) {
                // Estimate visual odometry pose
                const visualPose = estimatePose(matches);

                // Fuse with IMU
                poseDelta = fuseIMU(visualPose, imuDataRef.current, dt);
            }
        }

        // Draw all detected features as smooth animated dots (no tracking lines)
        // Optimize: render only every 3rd feature for better FPS
        const time = performance.now() / 1000;
        const renderStep = Math.max(1, Math.floor(features.length / 800)); // Limit rendered features

        for (let idx = 0; idx < features.length; idx += renderStep) {
            const feat = features[idx];
            // Simplified pulsing animation
            const pulse = Math.sin(time * 2 + idx * 0.05) * 0.2 + 0.8;
            const radius = 2 * pulse;

            // Bright yellow for all detected features
            processCtx.fillStyle = `rgba(255, 255, 0, ${0.7 * pulse})`;

            processCtx.beginPath();
            processCtx.arc(feat.x, feat.y, radius, 0, Math.PI * 2);
            processCtx.fill();
        }

        // Update pose
        const newPose = {
            x: poseRef.current.x + poseDelta.dx,
            y: poseRef.current.y + poseDelta.dy,
            heading: poseRef.current.heading + poseDelta.dHeading,
        };

        poseRef.current = newPose;
        setPose(newPose);

        // Update trajectory
        const newTrajectory = [
            ...trajectoryRef.current,
            { x: newPose.x, y: newPose.y },
        ];
        if (newTrajectory.length > 500) newTrajectory.shift(); // Limit trajectory length
        trajectoryRef.current = newTrajectory;
        setTrajectory(newTrajectory);

        // Update map
        const updatedLandmarks = updateMap(landmarksRef.current, features, newPose);
        landmarksRef.current = updatedLandmarks;
        setLandmarkCount(updatedLandmarks.length);

        // Update features state for UI display
        setFeatures(features);

        // Draw trajectory
        drawTrajectory();

        // Store current frame and features for next iteration
        prevFrameRef.current = grayData;
        prevFeaturesRef.current = features;

        // Continue loop
        animationFrameRef.current = requestAnimationFrame(processSLAMFrame);
    }, [
        isRunning,
        extractFeatures,
        trackFeatures,
        estimatePose,
        fuseIMU,
        updateMap,
    ]);

    /**
     * Draw 2D trajectory on canvas
     */
    const drawTrajectory = useCallback(() => {
        const canvas = trajectoryCanvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext("2d");
        const width = canvas.width;
        const height = canvas.height;

        // Clear canvas
        ctx.fillStyle = "#1a1a1a";
        ctx.fillRect(0, 0, width, height);

        // Draw grid
        ctx.strokeStyle = "#333";
        ctx.lineWidth = 1;
        for (let i = 0; i < width; i += 20) {
            ctx.beginPath();
            ctx.moveTo(i, 0);
            ctx.lineTo(i, height);
            ctx.stroke();
        }
        for (let i = 0; i < height; i += 20) {
            ctx.beginPath();
            ctx.moveTo(0, i);
            ctx.lineTo(width, i);
            ctx.stroke();
        }

        const traj = trajectoryRef.current;
        if (traj.length < 2) return;

        // Find bounds
        let minX = Infinity,
            maxX = -Infinity,
            minY = Infinity,
            maxY = -Infinity;
        traj.forEach((p) => {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        });

        const rangeX = Math.max(maxX - minX, 1);
        const rangeY = Math.max(maxY - minY, 1);
        const padding = 20;

        // Draw trajectory
        ctx.strokeStyle = "#00ff00";
        ctx.lineWidth = 2;
        ctx.beginPath();

        traj.forEach((p, i) => {
            const x = padding + ((p.x - minX) / rangeX) * (width - 2 * padding);
            const y =
                height - padding - ((p.y - minY) / rangeY) * (height - 2 * padding);

            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });

        ctx.stroke();

        // Draw start point
        const start = traj[0];
        const sx = padding + ((start.x - minX) / rangeX) * (width - 2 * padding);
        const sy =
            height - padding - ((start.y - minY) / rangeY) * (height - 2 * padding);
        ctx.fillStyle = "#0088ff";
        ctx.beginPath();
        ctx.arc(sx, sy, 5, 0, Math.PI * 2);
        ctx.fill();

        // Draw current point
        const curr = traj[traj.length - 1];
        const cx = padding + ((curr.x - minX) / rangeX) * (width - 2 * padding);
        const cy =
            height - padding - ((curr.y - minY) / rangeY) * (height - 2 * padding);
        ctx.fillStyle = "#ff0000";
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI * 2);
        ctx.fill();

        // Draw landmarks
        ctx.fillStyle = "rgba(255, 255, 0, 0.5)";
        landmarksRef.current.forEach((lm) => {
            const lx = padding + ((lm.x - minX) / rangeX) * (width - 2 * padding);
            const ly =
                height - padding - ((lm.y - minY) / rangeY) * (height - 2 * padding);
            ctx.fillRect(lx - 1, ly - 1, 2, 2);
        });
    }, []);

    // ======================
    // CONTROL HANDLERS
    // ======================

    const handleStart = async () => {
        const success = await startCamera();
        if (success) {
            setIsRunning(true);
            lastFrameTimeRef.current = performance.now();
        }
    };

    const handleStop = () => {
        setIsRunning(false);
        if (animationFrameRef.current) {
            cancelAnimationFrame(animationFrameRef.current);
        }
        stopCamera();
    };

    const handleReset = () => {
        poseRef.current = { x: 0, y: 0, heading: 0 };
        trajectoryRef.current = [{ x: 0, y: 0 }];
        landmarksRef.current = [];
        prevFrameRef.current = null;
        prevFeaturesRef.current = null;
        imuVelocityRef.current = { x: 0, y: 0 };

        setPose({ x: 0, y: 0, heading: 0 });
        setTrajectory([{ x: 0, y: 0 }]);
        setLandmarkCount(0);
        setFeatures([]);
    };

    // Start processing loop when running
    useEffect(() => {
        if (isRunning) {
            animationFrameRef.current = requestAnimationFrame(processSLAMFrame);
        }
        return () => {
            if (animationFrameRef.current) {
                cancelAnimationFrame(animationFrameRef.current);
            }
        };
    }, [isRunning, processSLAMFrame]);

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            stopCamera();
            if (watchIdRef.current !== null) {
                navigator.geolocation.clearWatch(watchIdRef.current);
            }
        };
    }, []);

    // GPS tracking
    useEffect(() => {
        if (isRunning && "geolocation" in navigator) {
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
        } else if (!isRunning && watchIdRef.current !== null) {
            navigator.geolocation.clearWatch(watchIdRef.current);
            watchIdRef.current = null;
        }

        return () => {
            if (watchIdRef.current !== null) {
                navigator.geolocation.clearWatch(watchIdRef.current);
                watchIdRef.current = null;
            }
        };
    }, [isRunning]);

    return (
        <div className="w-full h-full bg-gray-900 text-white overflow-y-auto">
            {/* Header */}
            <div className="mb-4 pt-6 text-center">
                <h2 className="text-2xl font-bold text-blue-400 mb-2">
                    SLAM (Simultaneous Localization and Mapping)
                </h2>
                <p className="text-xs text-gray-400">
                    Visual-Inertial SLAM | Feature detection with IMU fusion
                </p>
            </div>

            {/* Control Panel */}
            <div className="p-4 bg-gray-800 border-b border-gray-700">
                <div className="flex gap-2 flex-wrap justify-center">
                    <button
                        onClick={handleStart}
                        disabled={isRunning}
                        className="px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 rounded font-bold transition-colors text-sm"
                    >
                        Start SLAM
                    </button>
                    <button
                        onClick={handleStop}
                        disabled={!isRunning}
                        className="px-4 py-2 bg-red-600 hover:bg-red-700 disabled:bg-gray-600 rounded font-bold transition-colors text-sm"
                    >
                        Stop SLAM
                    </button>
                    <button
                        onClick={handleReset}
                        className="px-4 py-2 bg-yellow-600 hover:bg-yellow-700 rounded font-bold transition-colors text-sm"
                    >
                        Reset
                    </button>
                    <button
                        onClick={toggleTorch}
                        disabled={!isRunning}
                        className={`px-4 py-2 rounded font-bold transition-colors text-sm ${torchEnabled
                                ? "bg-orange-600 hover:bg-orange-700"
                                : "bg-gray-600 hover:bg-gray-700"
                            } disabled:bg-gray-600`}
                    >
                        {torchEnabled ? "Disable" : "Enable"} Torch
                    </button>
                </div>

                {error && (
                    <p className="text-red-400 text-sm mt-2 text-center">{error}</p>
                )}
            </div>

            {/* Stats Panel - Only FPS and Landmarks */}
            <div className="grid grid-cols-2 gap-2 p-4 bg-gray-800/50">
                <div className="bg-gray-700 p-2 rounded text-center">
                    <div className="text-xs text-gray-400">FPS</div>
                    <div className="text-lg font-bold text-green-400">{fps}</div>
                </div>
                <div className="bg-gray-700 p-2 rounded text-center">
                    <div className="text-xs text-gray-400">Landmarks</div>
                    <div className="text-lg font-bold text-yellow-400">
                        {landmarkCount}
                    </div>
                </div>
            </div>

            {/* Video & Canvas Display */}
            <div className="p-4">
                {/* Combined Camera Feed with Feature Detection Overlay */}
                <div className="bg-gray-800 rounded-lg overflow-hidden mb-6">
                    <h3 className="text-xs font-semibold text-gray-400 uppercase p-2 bg-gray-700">
                        Live Camera with Feature Detection
                    </h3>
                    <div className="relative w-full" style={{ paddingBottom: "75%" }}>
                        <video
                            ref={videoRef}
                            className="absolute top-0 left-0 w-full h-full opacity-0 object-cover"
                            playsInline
                            muted
                        />
                        <canvas
                            ref={canvasRef}
                            width={640}
                            height={480}
                            className="hidden"
                        />
                        <canvas
                            ref={processCanvasRef}
                            width={640}
                            height={480}
                            className="absolute top-0 left-0 w-full h-full object-cover"
                        />
                        {isRunning && (
                            <div className="absolute top-2 left-2 bg-black/50 px-2 py-1 rounded text-xs">
                                <span className="text-yellow-400">●</span>{" "}
                                {features.length || 0} Features Detected
                            </div>
                        )}
                    </div>
                </div>

                {/* SLAM Path Trajectory */}
                <div className="mb-6">
                    <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-2">
                        SLAM Path Trajectory
                    </h3>
                    <PathVisualizer path={trajectory} />
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
                                {isRunning
                                    ? "Acquiring GPS signal..."
                                    : "Start SLAM to enable GPS"}
                            </p>
                        </div>
                    )}
                </div>

                {/* Measured Position (SLAM) */}
                <div className="mb-6">
                    <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-3">
                        Measured Position (SLAM)
                    </h3>
                    <SensorCard
                        title="SLAM Position (2D)"
                        dataString={format2DData(pose)}
                        unit="meters"
                    />
                    <SensorCard
                        title="Estimated Velocity"
                        dataString={format2DData({
                            x: imuVelocityRef.current.x,
                            y: imuVelocityRef.current.y,
                        })}
                        unit="m/s"
                    />
                </div>

                {/* Raw Sensor Data */}
                <div className="mb-6">
                    <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider mb-3">
                        Raw Sensor Data
                    </h3>
                    <SensorCard
                        title="Accelerometer"
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
                        dataString={`alpha: ${(orientationData ?? 0).toFixed(2)}°`}
                        unit="0° = North"
                    />
                </div>
            </div>

            {/* Info */}
            <div className="p-4 text-xs text-gray-400 text-center">
                <p className="mt-1">
                    <span className="text-yellow-400">●</span> Detected Features
                </p>
            </div>
        </div>
    );
}
