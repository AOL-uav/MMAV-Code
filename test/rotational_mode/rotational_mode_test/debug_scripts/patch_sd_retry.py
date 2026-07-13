import sys

file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"

with open(file_path, 'r') as f:
    content = f.read()

old_sd_init = """  bool sdReady = SD.begin(SD_CS_PIN);
  File logFile;

  if (!sdReady) {"""

new_sd_init = """  bool sdReady = false;
  for (int retries = 0; retries < 5; retries++) {
    if (SD.begin(SD_CS_PIN)) {
      sdReady = true;
      break;
    }
    delay(400); // Wait for SD to power up, sometimes GPS brownouts cause failures
  }
  File logFile;

  if (!sdReady) {"""

if old_sd_init in content:
    content = content.replace(old_sd_init, new_sd_init)
    with open(file_path, 'w') as f:
        f.write(content)
    print("Added SD retry loop!")
else:
    print("Could not find SD init block!")
