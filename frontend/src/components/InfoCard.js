import React from 'react';
import { Box, Paper, Typography } from '@mui/material';

const InfoCard = ({ title, children, sx = {} }) => (
  <Paper sx={{ p: 2, height: '100%', display: 'flex', flexDirection: 'column', ...sx }}>
    <Typography variant="h6" gutterBottom>{title}</Typography>
    <Box sx={{ flexGrow: 1, minHeight: 0, position: 'relative' }}>
      {children}
    </Box>
  </Paper>
);

export default InfoCard;
