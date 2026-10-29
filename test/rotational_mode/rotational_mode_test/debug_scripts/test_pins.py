import sys
file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"
with open(file_path, 'r') as f:
    content = f.read()

insert = """
  pinMode(12, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  delay(10);
  int misoState = digitalRead(12);
  int mosiState = digitalRead(11);
  int sckState = digitalRead(13);
  
  safeSerialPrintln(F("==========================="));
  safeSerialPrintln(F("SPI PIN DIAGNOSTICS:"));
  safeSerialPrint(F("MISO (12) State (expected HIGH): ")); safeSerialPrintln(misoState == HIGH ? F("HIGH") : F("LOW (SHORTED OR SINKING)"));
  safeSerialPrint(F("MOSI (11) State (expected HIGH): ")); safeSerialPrintln(mosiState == HIGH ? F("HIGH") : F("LOW (SHORTED OR SINKING)"));
  safeSerialPrint(F("SCK (13) State (expected HIGH): ")); safeSerialPrintln(sckState == HIGH ? F("HIGH") : F("LOW (SHORTED OR SINKING)"));
  safeSerialPrintln(F("==========================="));
  delay(1000);
"""

pos = content.find('bool sdReady = false;')
if pos != -1:
    content = content[:pos] + insert + content[pos:]
    with open(file_path, 'w') as f:
        f.write(content)
    print("Patched.")
else:
    print("Not found.")
