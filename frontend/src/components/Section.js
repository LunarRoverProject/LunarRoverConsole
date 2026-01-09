import React from 'react';
import { Box, Paper, Typography, Divider } from '@mui/material';

const Section = ({ title, children, titleVariant = 'subtitle1', titleSx = {} }) => (
  <Paper sx={{ p: 2, mb: 1 }} elevation={1}>
    <Typography variant={titleVariant} sx={{ fontWeight: 700, mb: 1, ...titleSx }}>{title}</Typography>
    <Divider sx={{ mb: 1 }} />
    <Box>
      {children}
    </Box>
  </Paper>
);

export default Section;
