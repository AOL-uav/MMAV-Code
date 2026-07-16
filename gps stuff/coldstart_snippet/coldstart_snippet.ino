static void coldRestartGps() {
  // UBX-CFG-RST:
  // 0xFFFF = clear retained navigation data
  // 0x01   = controlled software restart
  const uint8_t payload[] = {
    0xFF, 0xFF,
    0x01,
    0x00
  };

  sendUbx(0x06, 0x04, payload, sizeof(payload));

  delay(3000);  // Allow GPS to restart

  // Reapply UART message/rate settings after restart.
  configureUbxReceiver();
}
