import React, { useState } from 'react';
import { Paper, Box, Typography, IconButton, Collapse } from '@mui/material';
import { ExpandLess, ExpandMore } from '@mui/icons-material';

const RawDataPanel = ({ raw }) => {
  const [open, setOpen] = useState(false);

  return (
    <Paper sx={{ p: 1, mb: 1 }} elevation={1}>
      <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <Typography variant="subtitle1" sx={{ fontWeight: 700 }}>Raw Data</Typography>
        <IconButton size="small" onClick={() => setOpen(v => !v)}>
          {open ? <ExpandLess /> : <ExpandMore />}
        </IconButton>
      </Box>
      <Collapse in={open}>
        <Box sx={{ p: 1, fontFamily: 'Consolas, monospace', fontSize: '0.8rem', color: 'text.secondary' }}>
          <pre style={{ whiteSpace: 'pre-wrap', margin: 0 }}>{raw ? JSON.stringify(raw, null, 2) : 'No raw data'}</pre>
        </Box>
      </Collapse>
    </Paper>
  );
};

export default RawDataPanel;
