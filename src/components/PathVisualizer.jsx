import React, { useState, useEffect, useCallback, useRef } from "react";

/**
 * Renders the calculated path as an SVG with pan/zoom controls.
 * @param {object} props - Component props.
 * @param {Array<object>} props.path - Array of {x, y} points.
 */
export default function PathVisualizer({ path }) {
  // View state
  const [viewBox, setViewBox] = useState("0 0 100 100");
  const [polyPoints, setPolyPoints] = useState("50,50");
  const [transform, setTransform] = useState({ x: 0, y: 0, scale: 1 });
  const [bounds, setBounds] = useState({ minX: 0, maxX: 0, minY: 0, maxY: 0 });

  // Interaction state
  const [isDragging, setIsDragging] = useState(false);
  const [startDrag, setStartDrag] = useState({ x: 0, y: 0 });
  const svgRef = useRef(null);
  const containerRef = useRef(null);
  const pinchRef = useRef({ active: false, startDist: 0, startScale: 1 });

  // Center the view on the path
  const centerView = useCallback(() => {
    if (path.length < 2) return;

    const centerX = (bounds.minX + bounds.maxX) / 2;
    const centerY = (bounds.minY + bounds.maxY) / 2;

    setTransform((t) => ({
      ...t,
      x: -centerX * t.scale,
      y: centerY * t.scale, // Note: we flip Y for SVG
    }));
  }, [bounds]);

  // Auto-center when path changes significantly
  useEffect(() => {
    if (path.length < 2) return;

    // Check if current point is near the viewport edge
    const lastPoint = path[path.length - 1];
    const container = containerRef.current;
    if (!container) return;

    const rect = container.getBoundingClientRect();
    const pointX = lastPoint.x * transform.scale + transform.x + rect.width / 2;
    const pointY =
      -lastPoint.y * transform.scale + transform.y + rect.height / 2;

    const margin = 100; // pixels from edge to trigger re-center
    if (
      pointX < margin ||
      pointX > rect.width - margin ||
      pointY < margin ||
      pointY > rect.height - margin
    ) {
      centerView();
    }
  }, [path, transform.scale, centerView]);

  // Calculate base viewbox from path
  useEffect(() => {
    if (path.length < 2) {
      setViewBox("0 0 100 100");
      setPolyPoints("50,50");
      return;
    }

    // 1. Find min/max bounds of the path
    let minX = Infinity,
      maxX = -Infinity,
      minY = Infinity,
      maxY = -Infinity;
    path.forEach((p) => {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    });

    // 2. Define padding and calculate viewbox dimensions
    const padding = 40; // Increased padding
    const width = Math.max(maxX - minX + padding * 2, 200); // Minimum width
    const height = Math.max(maxY - minY + padding * 2, 200); // Minimum height
    const vbX = minX - padding;
    const vbY = minY - padding;

    setViewBox(`${vbX} ${vbY} ${width} ${height}`);

    // 3. Convert path to polyline points string
    // We flip the Y-axis because SVG Y grows downwards, but our Y grows upwards
    const pointsString = path.map((p) => `${p.x},${-p.y}`).join(" ");
    setPolyPoints(pointsString);
  }, [path]);

  // Pan handlers
  const handleMouseDown = useCallback((e) => {
    if (e.button !== 0) return; // Left click only
    setIsDragging(true);
    setStartDrag({ x: e.clientX, y: e.clientY });
  }, []);

  const handleMouseMove = useCallback(
    (e) => {
      if (!isDragging) return;

      const dx = (e.clientX - startDrag.x) / transform.scale;
      const dy = (e.clientY - startDrag.y) / transform.scale;

      setTransform((t) => ({
        ...t,
        x: t.x + dx,
        y: t.y + dy,
      }));

      setStartDrag({ x: e.clientX, y: e.clientY });
    },
    [isDragging, startDrag, transform.scale]
  );

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
  }, []);

  // Touch handlers
  const handleTouchStart = useCallback(
    (e) => {
      if (!e.touches) return;
      if (e.touches.length === 1) {
        // Single touch for panning
        const t = e.touches[0];
        setIsDragging(true);
        setStartDrag({ x: t.clientX, y: t.clientY });
      } else if (e.touches.length === 2) {
        // Two touches for pinch-to-zoom
        const t0 = e.touches[0];
        const t1 = e.touches[1];
        const dx = t1.clientX - t0.clientX;
        const dy = t1.clientY - t0.clientY;
        const dist = Math.hypot(dx, dy);
        pinchRef.current = {
          active: true,
          startDist: dist,
          startScale: transform.scale,
        };
      }
    },
    [transform.scale]
  );

  const handleTouchMove = useCallback(
    (e) => {
      if (!e.touches) return;
      e.preventDefault(); // Prevent page scrolling

      if (pinchRef.current.active && e.touches.length === 2) {
        // Handle pinch-to-zoom
        const t0 = e.touches[0];
        const t1 = e.touches[1];
        const dx = t1.clientX - t0.clientX;
        const dy = t1.clientY - t0.clientY;
        const dist = Math.hypot(dx, dy);
        const factor = dist / pinchRef.current.startDist;
        const newScale = Math.min(
          Math.max(pinchRef.current.startScale * factor, 0.1),
          10
        );
        setTransform((t) => ({ ...t, scale: newScale }));
        return;
      }

      if (isDragging && e.touches.length === 1) {
        // Handle panning
        const t = e.touches[0];
        const dx = t.clientX - startDrag.x;
        const dy = t.clientY - startDrag.y;
        setTransform((t) => ({
          ...t,
          x: t.x + dx,
          y: t.y + dy,
        }));
        setStartDrag({ x: t.clientX, y: t.clientY });
      }
    },
    [isDragging, startDrag, transform.scale]
  );

  const handleTouchEnd = useCallback(() => {
    pinchRef.current.active = false;
    setIsDragging(false);
  }, []);

  // Zoom handler
  const handleWheel = useCallback((e) => {
    e.preventDefault();
    const scaleFactor = e.deltaY > 0 ? 0.9 : 1.1; // Zoom in/out
    setTransform((t) => ({
      ...t,
      scale: Math.min(Math.max(t.scale * scaleFactor, 0.1), 10), // Limit zoom range
    }));
  }, []);

  // Zoom buttons
  const handleZoom = useCallback((direction) => {
    const scaleFactor = direction === "in" ? 1.2 : 0.8;
    setTransform((t) => ({
      ...t,
      scale: Math.min(Math.max(t.scale * scaleFactor, 0.1), 10),
    }));
  }, []);

  return (
    <div className="bg-white/10 p-4 rounded-lg mb-4 text-white">
      <div className="flex items-center justify-between mb-3">
        <h3 className="text-sm font-medium text-gray-400 uppercase tracking-wider">
          Path Trajectory
        </h3>
        <div className="flex gap-2">
          <button
            onClick={() => handleZoom("in")}
            className="px-3 py-1 bg-gray-700 hover:bg-gray-600 rounded text-sm"
            title="Zoom In"
          >
            +
          </button>
          <button
            onClick={() => handleZoom("out")}
            className="px-3 py-1 bg-gray-700 hover:bg-gray-600 rounded text-sm"
            title="Zoom Out"
          >
            −
          </button>
        </div>
      </div>

      {/* Larger visualization area */}
      <div
        className="w-full bg-gray-800 rounded-md mt-2 overflow-hidden relative"
        style={{ height: "400px" }} // Fixed height instead of aspect-square
      >
        <svg
          ref={svgRef}
          className="w-full h-full cursor-grab active:cursor-grabbing"
          viewBox={viewBox}
          preserveAspectRatio="xMidYMid meet"
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseUp}
          onWheel={handleWheel}
          onTouchStart={handleTouchStart}
          onTouchMove={handleTouchMove}
          onTouchEnd={handleTouchEnd}
          style={{ touchAction: "none" }}
        >
          <g
            transform={`translate(${transform.x},${transform.y}) scale(${transform.scale})`}
          >
            {/* Grid for reference */}
            <pattern
              id="grid"
              width="50"
              height="50"
              patternUnits="userSpaceOnUse"
            >
              <path
                d="M 50 0 L 0 0 0 50"
                fill="none"
                stroke="#ffffff10"
                strokeWidth="0.5"
              />
            </pattern>
            <rect width="100%" height="100%" fill="url(#grid)" />

            <polyline
              points={polyPoints}
              fill="none"
              stroke="#00FF00"
              strokeWidth={2 / transform.scale}
              strokeLinecap="round"
              strokeLinejoin="round"
            />
            {/* Start Point */}
            {path.length > 0 && (
              <circle
                cx={path[0].x}
                cy={-path[0].y}
                r={3 / transform.scale}
                fill="#3b82f6"
              />
            )}
            {/* Current Point */}
            {path.length > 1 && (
              <circle
                cx={path[path.length - 1].x}
                cy={-path[path.length - 1].y}
                r={3 / transform.scale}
                fill="#ef4444"
              />
            )}
          </g>
        </svg>
      </div>

      <div className="flex justify-between items-center text-xs text-gray-400 mt-2">
        <div className="flex items-center gap-4">
          <span>
            <span className="w-3 h-3 inline-block bg-blue-500 rounded-full mr-1"></span>
            Start
          </span>
          <span>
            <span className="w-3 h-3 inline-block bg-red-500 rounded-full mr-1"></span>
            Current
          </span>
        </div>
        <div className="text-gray-500">Drag to pan • Scroll to zoom</div>
      </div>
    </div>
  );
}
