import sys

file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"

with open(file_path, 'r') as f:
    content = f.read()

serial1_block = """  // Optional GPS: configure a default GY-NEO6MV2 without requiring u-center.
  // If the module is absent, these UART writes are harmless and boot continues.
  Serial1.begin(GPS_BAUD);
  gps.lastFusedTowMs = 0xFFFFFFFFUL;
  delay(300);"""

if serial1_block in content:
    content = content.replace(serial1_block, "")
    
    # insert before "logClockStartMs = millis();" at the end of setup
    insert_point = content.find("  logClockStartMs = millis();")
    if insert_point != -1:
        new_serial_block = """  // Initialize GPS Serial AFTER SD card is initialized to prevent UART interrupts
  // from disrupting the fragile SD.begin() SPI communication.
  Serial1.begin(GPS_BAUD);
  gps.lastFusedTowMs = 0xFFFFFFFFUL;
  delay(300);

"""
        content = content[:insert_point] + new_serial_block + content[insert_point:]
        with open(file_path, 'w') as f:
            f.write(content)
        print("Moved Serial1.begin to the end of setup!")
    else:
        print("Could not find insertion point!")
else:
    print("Could not find Serial1 block!")
