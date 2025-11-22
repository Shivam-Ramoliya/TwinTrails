import React from "react";
import DRModule from "./components/DRModule";
import SLAMModule from "./components/SLAMModule";

/**
 * Main App Component - TwinTrails
 * Displays Dead Reckoning and SLAM side-by-side for comparison
 */
export default function App() {
  return (
    <div className="flex-1 bg-gray-900 min-h-screen font-sans">
      {/* Header */}
      <div className="bg-gray-800 border-b border-gray-700 p-4">
        <h1 className="text-2xl font-bold text-white">TwinTrails</h1>
        <p className="text-sm text-blue-400">
          DR vs. SLAM Localization (Web Preview)
        </p>
      </div>

      {/* Two Column Layout: DR | SLAM */}
      <div className="flex flex-col lg:grid lg:grid-cols-2 h-auto lg:h-[calc(100vh-140px)]">
        {/* LEFT: Dead Reckoning Module */}
        <DRModule />

        {/* RIGHT: SLAM Module */}
        <div className="bg-gray-900 overflow-y-auto">
          <SLAMModule />
        </div>
      </div>
    </div>
  );
}
