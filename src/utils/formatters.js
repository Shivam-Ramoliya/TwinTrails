// Utility formatters extracted from App.jsx
export const formatSensorData = (data) => {
    if (!data) {
        return "x: 0.00, y: 0.00, z: 0.00";
    }

    // Handle both types of sensor data keys
    const x = data.x ?? data.alpha;
    const y = data.y ?? data.beta;
    const z = data.z ?? data.gamma;

    return `x: ${x?.toFixed(2) || "0.00"}, y: ${y?.toFixed(2) || "0.00"}, z: ${z?.toFixed(2) || "0.00"
        }`;
};

export const format2DData = (data) => {
    if (!data) {
        return "x: 0.00, y: 0.00";
    }
    const x = data.x?.toFixed(2) || "0.00";
    const y = data.y?.toFixed(2) || "0.00";
    return `x: ${x}, y: ${y}`;
};
