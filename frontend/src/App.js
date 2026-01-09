import React, { useState, useEffect, useMemo, useRef } from 'react';
import {
  Box, Typography, IconButton, AppBar, Toolbar, Chip,
  TextField, Button, Select, MenuItem, Divider, Dialog, DialogContent, Paper
} from '@mui/material'; import { createTheme, ThemeProvider } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import {
  ArrowUpward, ArrowDownward, ArrowBack, ArrowForward, StopCircle, SatelliteAlt,
  SignalCellularAlt, SignalCellularOff, Brightness4, Brightness7, MyLocation,
  CheckCircle, Cancel, Fullscreen, Close
} from '@mui/icons-material';

import Compass from './components/Compass';
import InfoCard from './components/InfoCard';
import TelemetryItem from './components/TelemetryItem';
import MachineDetails from './components/MachineDetails';
import Section from './components/Section';
import { useJsApiLoader } from '@react-google-maps/api';
import Map from './components/Map';

// const CAMERA_STREAM_URL = 'http://localhost:8080/stream?topic=/camera/image_raw';
const MAX_GPS_HISTORY = 200; // GPS軌跡の最大記録数

// --- メインコンポーネント ---
function App() {
  // 状態管理
  const [mode, setMode] = useState('dark'); // 'dark' or 'light'
  const [isConnected, setIsConnected] = useState(false);
  const [pose, setPose] = useState(null);
  const [currentGps, setCurrentGps] = useState(null);
  const [gpsPath, setGpsPath] = useState([]);
  const [imuData, setImuData] = useState(null);
  const [heading, setHeading] = useState(0); // Add heading state

  const [actualRad, setActualRad] = useState(null);
  const [targets, setTargets] = useState(null);
  const [goalLat, setGoalLat] = useState(0.0); // Goal Latitude input
  const [goalLng, setGoalLng] = useState(0.0); // Goal Longitude input
  const [isRosConnected, setIsRosConnected] = useState(false);
  const [confirmedGoal, setConfirmedGoal] = useState(null);
  const [machineInfo, setMachineInfo] = useState({});
  const [isTracking, setIsTracking] = useState(true);
  const [joystick, setJoystick] = useState({ linear: 0, angular: 0, axes: [], buttons: [], lin_scale: 1.0, ang_scale: 1.0 });
  const [goalReached, setGoalReached] = useState(false);
  const [cameraFront, setCameraFront] = useState(null);
  const [cameraBack, setCameraBack] = useState(null);
  const [mapType, setMapType] = useState('roadmap'); // 'roadmap' or 'satellite'
  const [updateIntervalSeconds, setUpdateIntervalSeconds] = useState(1); // in seconds
  const [maximizedCamera, setMaximizedCamera] = useState(null); // 'front', 'back', or null
  const rateOptions = [1, 2, 3, 5, 10, 20, 30, 60];

  const ws = useRef(null); // WebSocket connection


  const [gamepadConnected, setGamepadConnected] = useState(false);
  const scalesRef = useRef({ lin: 1.0, ang: 1.0 }); // Refs for loop access
  const prevButtonsRef = useRef([]); // To detect button press edges

  const toggleColorMode = () => {
    setMode((prevMode) => (prevMode === 'light' ? 'dark' : 'light'));
  };

  const theme = useMemo(
    () =>
      createTheme({
        palette: {
          mode,
          ...(mode === 'light'
            ? {
              // palette for light mode
              background: {
                default: '#f5f5f5',
                paper: 'rgba(255, 255, 255, 0.9)',
              },
            }
            : {
              // palette for dark mode
              background: {
                default: '#121212',
                paper: 'rgba(30, 30, 30, 0.85)',
              },
            }),
          primary: {
            main: '#90caf9',
          },
          secondary: {
            main: '#f48fb1',
          },
        },
        typography: {
          fontFamily: '"Exo 2", "Roboto", "Helvetica", "Arial", sans-serif',
          fontWeightBold: 700,
        },
      }),
    [mode],
  );

  // ヨー角の計算
  const getYawFromQuaternion = (q) => {
    if (!q) return 0;
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * (180 / Math.PI);
  };

  useEffect(() => {
    if (isConnected && ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify({
        type: 'set-refresh-rate',
        data: { interval_seconds: updateIntervalSeconds }
      }));
    }
  }, [updateIntervalSeconds, isConnected]);

  useEffect(() => {
    ws.current = new WebSocket('ws://localhost:8000/ws');

    ws.current.onopen = () => {
      console.log('[WebSocket] Connected to FastAPI WebSocket.');
      setIsConnected(true);
    };

    ws.current.onmessage = (event) => {
      const message = JSON.parse(event.data);

      // update latest-received timestamp for machine info
      setMachineInfo(prev => ({ ...prev, timestamp: new Date().toISOString() }));

      // joystick / cmd_vel messages
      if (message.type === 'cmd_vel' || message.type === 'twist') {
        const d = message.data || {};
        setJoystick(prev => ({ ...prev, linear: d.linear?.x ?? prev.linear, angular: d.angular?.z ?? prev.angular }));
        return;
      }

      if (message.type === 'joy') {
        // accept raw joystick axes/buttons and optional scales
        const d = message.data || {};
        setJoystick(prev => ({
          ...prev,
          axes: d.axes ?? prev.axes,
          buttons: d.buttons ?? prev.buttons,
          lin_scale: d.lin_scale ?? prev.lin_scale,
          ang_scale: d.ang_scale ?? prev.ang_scale,
        }));
        return;
      }

      if (message.type === 'goal_reached') {
        const val = Boolean(message.data);
        setGoalReached(val);
        console.log(`[Frontend] Received goal_reached: ${val}`);
        return;
      }

      if (message.type === 'camera_front') {
        setCameraFront(message.data);
        return;
      }

      if (message.type === 'camera_back') {
        setCameraBack(message.data);
        return;
      }

      if (message.type === 'gps') {

        const newPosition = {
          lat: message.data.latitude,
          lng: message.data.longitude,
          alt: message.data.altitude,
          cov: message.data.position_covariance
        };
        setCurrentGps(newPosition);
        setGpsPath(prevPath => {
          const updatedPath = [...prevPath, newPosition];
          return updatedPath.length > MAX_GPS_HISTORY ? updatedPath.slice(-MAX_GPS_HISTORY) : updatedPath;
        });
      } else if (message.type === 'imu') {
        setImuData(message.data);
        setHeading(message.data.heading); // Correctly use heading
      } else if (message.type === 'pose') {
        setPose({ orientation: message.data.pose.orientation });
        // Also update heading from pose's quaternion
        const yaw = getYawFromQuaternion(message.data.pose.orientation);
        setHeading(yaw);

      } else if (message.type === 'actual_rad') {
        setActualRad(message.data);
      } else if (message.type === 'targets') {
        setTargets(message.data);
      } else if (message.type === 'ros_status') {
        console.log(`[WebSocket] ROS connection status: ${message.data.connected}`);
        setIsRosConnected(message.data.connected);
      } else if (message.type === 'goal_set') {
        console.log(`[WebSocket] Goal set at:`, message.data);
        setConfirmedGoal(message.data);
      } else {
        console.log(`[WebSocket] Unknown message type: |${message.type}|`);
      }
    };

    ws.current.onclose = () => {
      console.log('[WebSocket] Disconnected from FastAPI WebSocket.');
      setIsConnected(false);
      setIsRosConnected(false); // Also set ROS to disconnected
    };

    ws.current.onerror = (error) => {
      console.error('[WebSocket] WebSocket error:', error);
      setIsConnected(false);
      setIsRosConnected(false); // Also set ROS to disconnected
    };

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  // --- Gamepad Polling Logic ---
  useEffect(() => {
    let animationFrameId;

    const pollGamepad = () => {
      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      let activeGamepad = null;

      for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
          activeGamepad = gamepads[i];
          break; // Use the first active gamepad found
        }
      }

      if (activeGamepad) {
        if (!gamepadConnected) setGamepadConnected(true);

        const buttons = activeGamepad.buttons;
        const axes = activeGamepad.axes;
        const prevButtons = prevButtonsRef.current;
        const factor = 1.1;

        // Button Mapping from joy_translate_node.py:
        // Y (3) -> lin_scale * factor
        // X (2) -> lin_scale / factor
        // B (1) -> ang_scale * factor
        // A (0) -> ang_scale / factor

        // Helper to check button pressed (transition 0 -> 1)
        const isPressed = (idx) => buttons[idx]?.pressed && !(prevButtons[idx]?.pressed);



        if (buttons.length > 3) {
          // Ensure prevButtons is initialized
          if (prevButtons.length !== buttons.length) {
            prevButtonsRef.current = new Array(buttons.length).fill({ pressed: false });
          }

          // Linear Scale
          if (isPressed(3)) { // Y
            scalesRef.current.lin *= factor;
            console.log('Gamepad: Y pressed -> lin_scale', scalesRef.current.lin);
          }
          if (isPressed(2)) { // X
            scalesRef.current.lin /= factor;
            console.log('Gamepad: X pressed -> lin_scale', scalesRef.current.lin);
          }

          // Angular Scale
          if (isPressed(1)) { // B
            scalesRef.current.ang *= factor;
            console.log('Gamepad: B pressed -> ang_scale', scalesRef.current.ang);
          }
          if (isPressed(0)) { // A
            scalesRef.current.ang /= factor;
            console.log('Gamepad: A pressed -> ang_scale', scalesRef.current.ang);
          }
        }

        // Save current button state
        // Need to map to a simple array or object to store 'pressed' value
        prevButtonsRef.current = buttons.map(b => ({ pressed: b.pressed }));

        // Axis Mapping (Single Stick - Left Stick)
        // Linear: axes[1] (Up is -1, Down is +1 usually) -> Invert for Forward(+)
        // Angular: axes[0] (Left is -1, Right is +1) -> Left is +Rot(CCW) usually in ROS.

        const axisLinearRaw = axes[1];
        const axisAngularRaw = axes[0];

        const deadzone = 0.05;
        let linear = Math.abs(axisLinearRaw) > deadzone ? -axisLinearRaw : 0;
        let angular = Math.abs(axisAngularRaw) > deadzone ? axisAngularRaw : 0; // Check rotation direction later

        // Apply scale
        linear *= scalesRef.current.lin;
        angular *= scalesRef.current.ang;

        // Update Joystick UI State (Throttle or only on significant change? For now, update always for smoothness)
        // We only update if something changed to avoid spamming setState? 
        // Actually, let's update.
        // Also update scales in UI if they changed.

        setJoystick(prev => ({
          ...prev,
          linear: linear,
          angular: angular,
          lin_scale: scalesRef.current.lin,
          ang_scale: scalesRef.current.ang
        }));

        if (ws.current && ws.current.readyState === WebSocket.OPEN) {
          ws.current.send(JSON.stringify({
            type: 'twist-command',
            data: { linear: linear, angular: angular }
          }));
        }
      } else {
        if (gamepadConnected) setGamepadConnected(false);
      }

      animationFrameId = requestAnimationFrame(pollGamepad);
    };

    // Start polling
    animationFrameId = requestAnimationFrame(pollGamepad);

    return () => {
      cancelAnimationFrame(animationFrameId);
    };
  }, [gamepadConnected]);


  // コマンド送信
  const sendCommand = (command) => {
    ws.current.send(JSON.stringify({
      type: 'rover-command',
      data: { command: command }
    }));
  }


  const publishGoal = () => {
    if (isFullyOnline && ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify({
        type: 'publish-goal',
        data: {
          goalData: {
            latitude: goalLat,
            longitude: goalLng,
            altitude: 0.0,
          }
        }
      }));
      console.log('Publishing GPS goal via WebSocket:', { lat: goalLat, lng: goalLng });
    }
  };

  const formatNumber = (num, fractionDigits = 3) => (typeof num === 'number') ? num.toFixed(fractionDigits) : 'N/A';

  const formatTimestamp = (d) => {
    if (!d) return 'N/A';
    try { return new Date(d).toLocaleString(); } catch (e) { return String(d); }
  };

  // const MANUAL_KEY = ""; 
  // const API_KEY = MANUAL_KEY || process.env.REACT_APP_GOOGLE_MAPS_API_KEY;

  const { isLoaded, loadError } = useJsApiLoader({
    id: 'google-map-script',
    googleMapsApiKey: process.env.REACT_APP_GOOGLE_MAPS_API_KEY,
  });

  useEffect(() => {
    console.log('[App] Google Maps API Key Present:', !!process.env.REACT_APP_GOOGLE_MAPS_API_KEY);
    console.log('[App] Map Load Status:', { isLoaded, loadError });
  }, [isLoaded, loadError]);

  const isFullyOnline = isConnected && isRosConnected;

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Box sx={{ display: 'flex', flexDirection: 'column', height: '100%', width: '100%', overflow: 'hidden', bgcolor: 'background.default' }}>

        {/* Header */}
        <AppBar position="static" elevation={0} sx={{
          background: mode === 'dark' ? 'rgba(18, 18, 18, 0.8)' : 'rgba(245, 245, 245, 0.8)',
          backdropFilter: 'blur(10px)'
        }}>          <Toolbar>
            <Box component="img" src="/logo.png" alt="Logo" sx={{ height: 40, mr: 2 }} />
            <Typography variant="h6" component="div" sx={{ fontWeight: 'bold' }}>
              Lunar Rover Console
            </Typography>
            <Box sx={{ flexGrow: 1 }} />
            <Box sx={{ display: 'flex', alignItems: 'center', mr: 2 }}>
              <Chip
                icon={goalReached ? <CheckCircle /> : <Cancel />}
                label={goalReached === null ? 'GOAL: N/A' : (goalReached ? 'GOAL: YES' : 'GOAL: NO')}
                color={goalReached === true ? 'success' : (goalReached === false ? 'default' : 'info')}
                variant="outlined"
                sx={{ mr: 2 }}
              />
            </Box>
            <Box sx={{ width: 200, mr: 3, display: 'flex', alignItems: 'center' }}>
              <Typography variant="caption" sx={{ mr: 2 }}>Rate</Typography>
              <Select
                value={updateIntervalSeconds}
                onChange={(e) => setUpdateIntervalSeconds(e.target.value)}
                size="small"
              >
                {rateOptions.map((rate) => (
                  <MenuItem key={rate} value={rate}>
                    {rate} s
                  </MenuItem>
                ))}
              </Select>
            </Box>
            <IconButton sx={{ color: 'inherit', borderRadius: '8px', p: '4px 8px' }} onClick={toggleColorMode} title="Toggle light/dark mode">
              {mode === 'dark' ? <Brightness7 sx={{ mr: 1 }} /> : <Brightness4 sx={{ mr: 1 }} />}
              <Typography variant="button">{mode === 'dark' ? 'Light' : 'Dark'}</Typography>
            </IconButton>
            <IconButton size="small" onClick={() => setMapType(prev => prev === 'roadmap' ? 'satellite' : 'roadmap')} sx={{ color: 'inherit', borderRadius: '8px', p: '4px 8px', ml: 1 }}>
              <SatelliteAlt sx={{ mr: 1 }} />
              <Typography variant="button">
                {mapType}
              </Typography>
            </IconButton>
            <Chip
              icon={isFullyOnline ? <SignalCellularAlt /> : <SignalCellularOff />}
              label={isFullyOnline ? 'ONLINE' : 'OFFLINE'}
              color={isFullyOnline ? 'success' : 'error'}
              variant="outlined"
              sx={{ ml: 2 }}
            />
            {gamepadConnected && (
              <Chip
                icon={<Brightness7 />} // Placeholder icon or use Gamepad icon if available
                label="GAMEPAD"
                color="secondary"
                variant="filled"
                sx={{ ml: 2 }}
              />
            )}
          </Toolbar>
        </AppBar>

        {/* Main Content */}
        <Box sx={{ flexGrow: 1, p: 2, minHeight: 0, display: 'flex', justifyContent: 'center' }}>
          <Box sx={{ display: 'grid', gridTemplateColumns: 'minmax(0, 3fr) minmax(0, 1fr)', width: '100%', height: '100%', gap: 4 }}>

            {/* Left Column (Map) */}
            <Box sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
              <Paper sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column', borderRadius: 3, overflow: 'hidden', bgcolor: 'background.paper', boxShadow: 3 }}>
                <Box sx={{ p: 2, borderBottom: 1, borderColor: 'divider' }}>
                  <Typography variant="h6" sx={{ fontWeight: 'bold' }}>Global Position</Typography>
                </Box>
                <Box sx={{ flexGrow: 1, p: 2, display: 'flex' }}>
                  <Box sx={{ position: 'relative', flexGrow: 1, borderRadius: 4, overflow: 'hidden', boxShadow: 4, bgcolor: 'background.default' }}>
                    <Map
                      isLoaded={isLoaded}
                      apiKey={process.env.REACT_APP_GOOGLE_MAPS_API_KEY}
                      currentPosition={currentGps}
                      path={gpsPath}
                      heading={getYawFromQuaternion(pose?.orientation)}
                      mode={mode}
                      mapType={mapType}
                      isTracking={isTracking}
                      setIsTracking={setIsTracking}
                      goalPosition={confirmedGoal}
                    />
                    <Box sx={{ position: 'absolute', bottom: 16, right: 16, zIndex: 1 }}>
                      <IconButton onClick={() => setIsTracking(!isTracking)} color={isTracking ? "primary" : "default"} sx={{ backgroundColor: 'background.paper', '&:hover': { backgroundColor: 'background.default' } }}>
                        <MyLocation />
                      </IconButton>
                    </Box>
                    <Box sx={{ position: 'absolute', top: 12, right: 12, width: '260px', height: '260px' }}>
                      <InfoCard title="" sx={{ height: '100%', backgroundColor: 'transparent', border: 'none', p: 1 }}>
                        <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                          <Box sx={{ width: '95%', height: '95%' }}>
                            <Compass heading={heading} mode={mode} />
                          </Box>
                        </Box>
                      </InfoCard>
                    </Box>
                  </Box>
                </Box>
              </Paper>
            </Box>

            {/* Right Column (All other info) */}
            <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2, maxHeight: '100%', overflowY: 'auto' }}>

              {/* Camera Feed (WebSocket) */}
              <InfoCard title="" sx={{ p: 0 }}>
                <Section title="Camera" titleVariant="subtitle1">
                  <Box sx={{ flexGrow: 1, minHeight: '200px', borderRadius: '8px', overflow: 'hidden', background: '#000', display: 'flex', flexDirection: 'column', gap: 1 }}>
                    {/* Front Camera */}
                    <Box sx={{ position: 'relative', flex: 1, display: 'flex', justifyContent: 'center', alignItems: 'center', overflow: 'hidden' }}>
                      {cameraFront ? (
                        <img src={cameraFront} alt="Camera Front" style={{ width: '100%', height: '100%', objectFit: 'contain' }} />
                      ) : (
                        <Typography variant="body2" sx={{ color: 'grey.500' }}>No Front Signal</Typography>
                      )}
                      <Typography variant="caption" sx={{ position: 'absolute', top: 4, left: 4, color: 'white', bgcolor: 'rgba(0,0,0,0.5)', px: 0.5, borderRadius: 1 }}>Front</Typography>
                      <IconButton
                        size="small"
                        onClick={() => setMaximizedCamera('front')}
                        sx={{ position: 'absolute', top: 4, right: 4, color: 'white', bgcolor: 'rgba(0,0,0,0.3)', '&:hover': { bgcolor: 'rgba(0,0,0,0.5)' } }}
                      >
                        <Fullscreen fontSize="small" />
                      </IconButton>
                    </Box>
                    <Divider sx={{ borderColor: 'rgba(255,255,255,0.2)' }} />
                    {/* Back Camera */}
                    <Box sx={{ position: 'relative', flex: 1, display: 'flex', justifyContent: 'center', alignItems: 'center', overflow: 'hidden' }}>
                      {cameraBack ? (
                        <img src={cameraBack} alt="Camera Back" style={{ width: '100%', height: '100%', objectFit: 'contain' }} />
                      ) : (
                        <Typography variant="body2" sx={{ color: 'grey.500' }}>No Back Signal</Typography>
                      )}
                      <Typography variant="caption" sx={{ position: 'absolute', top: 4, left: 4, color: 'white', bgcolor: 'rgba(0,0,0,0.5)', px: 0.5, borderRadius: 1 }}>Back</Typography>
                      <IconButton
                        size="small"
                        onClick={() => setMaximizedCamera('back')}
                        sx={{ position: 'absolute', top: 4, right: 4, color: 'white', bgcolor: 'rgba(0,0,0,0.3)', '&:hover': { bgcolor: 'rgba(0,0,0,0.5)' } }}
                      >
                        <Fullscreen fontSize="small" />
                      </IconButton>
                    </Box>
                  </Box>
                </Section>
              </InfoCard>

              {/* Machine Details (below camera) */}
              <InfoCard title="" sx={{ minHeight: 'auto', p: 0, backgroundColor: 'transparent', boxShadow: 'none' }}>
                <Box sx={{ p: 0 }}>
                  <MachineDetails
                    machine={{ ...machineInfo, timestamp: formatTimestamp(machineInfo.timestamp) }}
                    gps={{ lat: currentGps?.lat, lng: currentGps?.lng, alt: currentGps?.alt, cov: currentGps?.cov }}
                    additional={{}}
                    imu={imuData}
                    raw={pose}
                  />
                </Box>
              </InfoCard>

              {/* Rover Status Removed */}

              {/* Joystick */}
              <InfoCard title="" sx={{ p: 0 }}>
                <Section title="Joystick">
                  <TelemetryItem label="Linear" value={formatNumber(joystick.linear, 3)} unit="m/s" />
                  <TelemetryItem label="Angular" value={formatNumber(joystick.angular, 3)} unit="rad/s" />
                  <TelemetryItem label="Lin scale" value={joystick.lin_scale?.toFixed(2) ?? 'N/A'} />
                  <TelemetryItem label="Ang scale" value={joystick.ang_scale?.toFixed(2) ?? 'N/A'} />
                </Section>
              </InfoCard>

              {/* Wheel Status */}
              <InfoCard title="" sx={{ p: 0 }}>
                <Section title="Wheel Status">
                  <TelemetryItem label="Actual Rad/s" value={actualRad ? `[${actualRad.map(r => formatNumber(r, 2)).join(', ')}]` : 'N/A'} unit="" />
                  <TelemetryItem label="Target Rad/s" value={targets ? `[${targets.map(t => formatNumber(t, 2)).join(', ')}]` : 'N/A'} unit="" />
                </Section>
              </InfoCard>



              {/* Goal Reached Indicator removed — header shows goal status */}

              {/* IMU Data */}
              <InfoCard title="" sx={{ p: 0 }}>
                <Section title="IMU Data">
                  <TelemetryItem label="Heading" value={formatNumber(heading, 2)} unit="°" />
                  <TelemetryItem label="Roll" value={formatNumber(imuData?.roll, 2)} unit="°" />
                  <TelemetryItem label="Pitch" value={formatNumber(imuData?.pitch, 2)} unit="°" />
                </Section>
              </InfoCard>

              {/* Controls */}
              <InfoCard title="" sx={{ p: 0 }}>
                <Section title="Manual Control">
                  <Box sx={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 1, alignItems: 'center', justifyItems: 'center' }}>
                    <Box />
                    <IconButton color="primary" onClick={() => sendCommand('forward')}><ArrowUpward fontSize="large" /></IconButton>
                    <Box />
                    <IconButton color="primary" onClick={() => sendCommand('left')}><ArrowBack fontSize="large" /></IconButton>
                    <IconButton color="error" onClick={() => sendCommand('stop')}><StopCircle sx={{ fontSize: 50 }} /></IconButton>
                    <IconButton color="primary" onClick={() => sendCommand('right')}><ArrowForward fontSize="large" /></IconButton>
                    <Box />
                    <IconButton color="primary" onClick={() => sendCommand('backward')}><ArrowDownward fontSize="large" /></IconButton>
                    <Box />
                  </Box>
                </Section>
              </InfoCard>

              {/* Goal Setting */}
              <InfoCard title="" sx={{ p: 0 }}>
                <Section title="Goal Setting">
                  <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
                    <TextField
                      label="Latitude"
                      type="number"
                      size="small"
                      value={goalLat}
                      onChange={(e) => setGoalLat(parseFloat(e.target.value))}
                      fullWidth
                    />
                    <TextField
                      label="Longitude"
                      type="number"
                      size="small"
                      value={goalLng}
                      onChange={(e) => setGoalLng(parseFloat(e.target.value))}
                      fullWidth
                    />
                    <Button
                      variant="contained"
                      color="primary"
                      fullWidth
                      onClick={publishGoal}
                      disabled={!isFullyOnline}
                    >
                      Publish Goal
                    </Button>
                  </Box>
                </Section>
              </InfoCard>


            </Box>
          </Box>
        </Box>
      </Box>
      {/* Camera Maximized Dialog */}
      <Dialog
        open={maximizedCamera !== null}
        onClose={() => setMaximizedCamera(null)}
        maxWidth="lg"
        fullWidth
        PaperProps={{
          style: {
            backgroundColor: 'black',
            color: 'white',
            overflow: 'hidden'
          },
        }}
      >
        <DialogContent sx={{ p: 0, position: 'relative', height: '80vh', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
          <IconButton
            onClick={() => setMaximizedCamera(null)}
            sx={{ position: 'absolute', top: 8, right: 8, color: 'white', bgcolor: 'rgba(0,0,0,0.5)', '&:hover': { bgcolor: 'rgba(0,0,0,0.7)' }, zIndex: 10 }}
          >
            <Close />
          </IconButton>
          {maximizedCamera === 'front' && cameraFront && (
            <img src={cameraFront} alt="Front Camera Full" style={{ width: '100%', height: '100%', objectFit: 'contain' }} />
          )}
          {maximizedCamera === 'back' && cameraBack && (
            <img src={cameraBack} alt="Back Camera Full" style={{ width: '100%', height: '100%', objectFit: 'contain' }} />
          )}
          {((maximizedCamera === 'front' && !cameraFront) || (maximizedCamera === 'back' && !cameraBack)) && (
            <Typography variant="h5" sx={{ color: 'grey.500' }}>No Signal</Typography>
          )}
        </DialogContent>
      </Dialog>

    </ThemeProvider >
  );
}

export default App;