import React from "react";

/**
 * A reusable card component to display sensor information.
 * @param {object} props
 */
export default function SensorCard({ title, dataString, unit }) {
    return (
        <div className="bg-white/10 p-4 rounded-lg mb-4 text-white">
            <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider">
                {title}
            </h3>
            <p className="text-sm font-mono mt-2">{dataString}</p>
            {unit && <p className="text-xs text-gray-500 mt-1">{unit}</p>}
        </div>
    );
}
