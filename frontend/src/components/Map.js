import React from 'react';
import { GoogleMap, Marker, Polyline } from '@react-google-maps/api';
import { darkMapStyle, lightmapstyles } from '../mapStyles';

const DEFAULT_LATITUDE = 35.681236; // Tokyo Station
const DEFAULT_LONGITUDE = 139.767125;

const Map = ({ isLoaded, apiKey, currentPosition, path, heading, mode, mapType, isTracking, setIsTracking, goalPosition }) => {
  const mapRef = React.useRef(null);
  const mapOptions = React.useMemo(() => ({
    disableDefaultUI: true,
    styles: mode === 'dark' ? darkMapStyle : lightmapstyles,
    minZoom: 2,
    maxZoom: 22,
  }), [mode]);

  React.useEffect(() => {
    if (mapRef.current && isTracking && currentPosition) {
      mapRef.current.panTo(currentPosition);
    }
  }, [currentPosition, isTracking]);

  if (!isLoaded) return <div>Loading Map...</div>;

  return (
    <div style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0 }}>
      <GoogleMap
        mapContainerStyle={{ width: '100%', height: '100%' }}
        center={isTracking && currentPosition ? currentPosition : undefined}
        defaultCenter={{ lat: DEFAULT_LATITUDE, lng: DEFAULT_LONGITUDE }}
        zoom={12}
        options={mapOptions}
        mapTypeId={mapType}
        onLoad={(map) => (mapRef.current = map)}
        onDragStart={() => setIsTracking(false)}
      >
        {currentPosition && (
          <Marker
            position={currentPosition}
            icon={{
              path: window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
              scale: 7,
              rotation: heading,
              fillColor: '#FF0000',
              fillOpacity: 1,
              strokeWeight: 2,
              strokeColor: '#1e1e1e',
            }}
          />
        )}
        {path.length > 0 && (
          <Polyline
            path={path}
            options={{
              strokeColor: '#90caf9',
              strokeOpacity: 0.7,
              strokeWeight: 3,
            }}
          />
        )}
        {goalPosition && (
          <Marker
            position={goalPosition}
            icon={{
              path: 'M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z',
              fillColor: '#FF0000', // Red color for the flag
              fillOpacity: 1,
              strokeWeight: 2,
              strokeColor: '#FFFFFF',
              scale: 1,
            }}
          />
        )}
        {currentPosition && goalPosition && (
          <Polyline
            path={[currentPosition, goalPosition]}
            options={{
              strokeColor: '#800080', // Purple color
              strokeOpacity: 0.8,
              strokeWeight: 2,
              geodesic: true,
            }}
          />
        )}
      </GoogleMap>
    </div>
  );
};

export default Map;
