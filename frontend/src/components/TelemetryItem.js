import React from 'react';
import { Box, Typography } from '@mui/material';

const TelemetryItem = ({ label, value, unit }) => (
  <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
    <Typography variant="body1" sx={{ fontFamily: '"Consolas", monospace' }}>{label}</Typography>
    <Typography variant="body1" sx={{ lineHeight: 1, fontFamily: '"Consolas", monospace' }}>
      {value} <span style={{ fontSize: '0.8rem' }}>{unit}</span>
    </Typography>
  </Box>
);

export default TelemetryItem;
