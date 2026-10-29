import sys
file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"
with open(file_path, 'r') as f:
    content = f.read()

insert = """
    if (globalSdReady) {
      File root = SD.open("/");
      if (root) {
        int count = 0;
        while (true) {
          File entry = root.openNextFile();
          if (!entry) break;
          count++;
          entry.close();
        }
        root.close();
        Serial.print(F("SD Files:    ")); Serial.println(count);
      }
    }
"""

pos = content.find('Serial.print(F("SD Status:   "));')
if pos != -1:
    content = content[:pos] + insert + content[pos:]
    with open(file_path, 'w') as f:
        f.write(content)
    print("Patched.")
else:
    print("Not found.")
