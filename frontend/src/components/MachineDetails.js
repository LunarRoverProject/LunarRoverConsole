import React from 'react';
import { Box, Typography } from '@mui/material';
import TelemetryItem from './TelemetryItem';
import Section from './Section';

const MachineDetails = ({ machine = {}, gps = {}, imu = null, additional = {}, raw = null }) => {

  return (
    <Box>
      <Typography variant="h6" sx={{ mb: 1, fontWeight: 'bold' }}>Machine Details</Typography>

      <Section title="Machine Information">
        <TelemetryItem label="Timestamp" value={machine.timestamp ?? 'N/A'} />
        <TelemetryItem label="Machine Time" value={machine.time ?? 'N/A'} />
        <TelemetryItem label="Loss Time" value={machine.loss ?? 'N/A'} />
      </Section>

      <Section title="GPS Data">
        <TelemetryItem label="Latitude" value={gps.lat ?? 'N/A'} unit="" />
        <TelemetryItem label="Longitude" value={gps.lng ?? 'N/A'} unit="" />
        <TelemetryItem label="Altitude" value={gps.alt ?? 'N/A'} />
        <TelemetryItem label="Covariance" value={gps.cov && gps.cov.length > 0 ? gps.cov[0].toFixed(2) : 'N/A'} />
      </Section>

      <Section title="IMU Data">
        <TelemetryItem label="Heading" value={imu ? `${imu.heading.toFixed(2)}°` : 'N/A'} />
        <TelemetryItem label="Roll" value={imu ? `${imu.roll.toFixed(2)}°` : 'N/A'} />
        <TelemetryItem label="Pitch" value={imu ? `${imu.pitch.toFixed(2)}°` : 'N/A'} />
      </Section>

      {/* Additional Information moved to top of Raw Data panel in App.js */}

      {/* Raw Data moved to bottom panel */}
    </Box>
  );
};

export default MachineDetails;
