import React from 'react';
import { Box } from '@mui/material';

// Compass Rose design: SVG with ticks, cardinal labels and a smooth rotating needle.
const Compass = ({ heading, mode }) => {
  const rotation = -heading; // rotate SVG needle opposite to yaw sign for intuitive display

  const size = 260;
  const pad = 20; // reduced vertical padding to make compass larger inside container
  const viewHeight = size + pad;
  const cx = size / 2;
  const cy = size / 2 + pad / 2; // move center slightly down to make room at top

  const r = Math.min(cx, cy) - 18; // outer radius used for ticks
  const labelRadius = r + 20; // place cardinal labels just outside the ring

  // build tick lines (every 10 degrees with bigger ticks at cardinal directions)
  const ticks = [];
  for (let deg = 0; deg < 360; deg += 10) {
    const isCardinal = deg % 90 === 0;
    const r2 = r; // outer tick radius
    const r1 = isCardinal ? (r2 - 20) : (deg % 30 === 0 ? (r2 - 14) : (r2 - 18));
    const rad = (deg - 90) * (Math.PI / 180);
    const x1 = cx + r1 * Math.cos(rad);
    const y1 = cy + r1 * Math.sin(rad);
    const x2 = cx + r2 * Math.cos(rad);
    const y2 = cy + r2 * Math.sin(rad);
    ticks.push({ x1, y1, x2, y2, isCardinal, deg });
  }

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', position: 'relative' }}>
      <svg width="100%" height="100%" viewBox={`0 0 ${size} ${viewHeight}`} preserveAspectRatio="xMidYMid meet">
        <defs>
          <radialGradient id="compGrad" cx="50%" cy="40%" r="70%">
            <stop offset="0%" stopColor="#222" />
            <stop offset="60%" stopColor="#141414" />
            <stop offset="100%" stopColor="#0d0d0d" />
          </radialGradient>
          <linearGradient id="needleGrad" x1="0%" x2="0%" y1="0%" y2="100%">
            <stop offset="0%" stopColor="#ff6b6b" />
            <stop offset="100%" stopColor="#ff8a65" />
          </linearGradient>
          <linearGradient id="needleBlue" x1="0%" x2="0%" y1="0%" y2="100%">
            <stop offset="0%" stopColor="#6fb3ff" />
            <stop offset="100%" stopColor="#3b82f6" />
          </linearGradient>
        </defs>
        {/* Outer ring */}
        <circle cx={cx} cy={cy} r={r + 2} fill="none" stroke="rgba(255,255,255,0.06)" strokeWidth="6" />
        <circle cx={cx} cy={cy} r={r} fill="url(#compGrad)" stroke="rgba(255,255,255,0.03)" strokeWidth="1" />

        {/* Inner shadow ring */}
        <circle cx={cx} cy={cy} r={r - 30} fill="rgba(0,0,0,0.28)" stroke="rgba(255,255,255,0.02)" strokeWidth="1" />

        {/* Ticks */}
        {ticks.map((t, i) => (
          <line
            key={i}
            x1={t.x1}
            y1={t.y1}
            x2={t.x2}
            y2={t.y2}
            stroke={t.isCardinal ? '#ffffff' : 'rgba(255,255,255,0.55)'}
            strokeWidth={t.isCardinal ? 2 : 1}
            strokeOpacity={t.isCardinal ? 1 : 0.6}
            strokeLinecap="round"
          />
        ))}

        {/* Cardinal labels (outside the ring) */}
        <text x={cx} y={cy - labelRadius} textAnchor="middle" fill={mode === 'light' ? '#000' : '#fff'} fontSize="15" fontWeight="800" dominantBaseline="middle" stroke={mode === 'light' ? '#fff' : '#000'} strokeWidth="0.6" paintOrder="stroke">N</text>
        <text x={cx + labelRadius} y={cy} textAnchor="middle" fill={mode === 'light' ? '#000' : '#fff'} fontSize="15" fontWeight="800" dominantBaseline="middle" stroke={mode === 'light' ? '#fff' : '#000'} strokeWidth="0.6" paintOrder="stroke">E</text>
        <text x={cx} y={cy + labelRadius} textAnchor="middle" fill={mode === 'light' ? '#000' : '#fff'} fontSize="15" fontWeight="800" dominantBaseline="middle" stroke={mode === 'light' ? '#fff' : '#000'} strokeWidth="0.6" paintOrder="stroke">S</text>
        <text x={cx - labelRadius} y={cy} textAnchor="middle" fill={mode === 'light' ? '#000' : '#fff'} fontSize="15" fontWeight="800" dominantBaseline="middle" stroke={mode === 'light' ? '#fff' : '#000'} strokeWidth="0.6" paintOrder="stroke">W</text>

        {/* Rotating needle group */}
        <g transform={`rotate(${rotation} ${cx} ${cy})`} style={{ transition: 'transform 0.22s cubic-bezier(.2,.9,.25,1)' }}>
          {/* Small south-facing (red) needle (larger) */}
          <path d={`M ${cx} ${cy + 4} L ${cx - 10} ${cy + 40} L ${cx + 10} ${cy + 40} Z`} fill="url(#needleGrad)" stroke="#fff" strokeWidth="1" />

          {/* Small north-facing (blue) needle (larger) */}
          <path d={`M ${cx} ${cy - 4} L ${cx - 10} ${cy - 40} L ${cx + 10} ${cy - 40} Z`} fill="url(#needleBlue)" stroke="#fff" strokeWidth="1" />

          {/* thin pointer line for subtle center (extended) */}
          <line x1={cx} y1={cy - 44} x2={cx} y2={cy + 44} stroke="rgba(255,255,255,0.06)" strokeWidth="1" />
        </g>

        {/* Center pivot ring */}
        <circle cx={cx} cy={cy} r={6} fill="#fff" />
        <circle cx={cx} cy={cy} r={3} fill="#0b0b0b" />
      </svg>

      <Box sx={{ position: 'absolute', top: 10, right: 18 }}>
        <Box sx={{ bgcolor: mode === 'light' ? 'rgba(255,255,255,0.85)' : 'rgba(0,0,0,0.85)', color: mode === 'light' ? '#000' : '#fff', px: 1.1, py: 0.4, borderRadius: '6px', boxShadow: '0 4px 10px rgba(0,0,0,0.6)', fontWeight: 800, fontSize: '0.78rem' }}>
          {`${Math.round(heading)}°`}
        </Box>
      </Box>
    </Box>
  );
};

export default Compass;