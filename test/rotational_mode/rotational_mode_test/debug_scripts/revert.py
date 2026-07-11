import sys
file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"
with open(file_path, 'r') as f:
    content = f.read()

bad_insert = """
    int misoState = digitalRead(12);
    int mosiState = digitalRead(11);
    int sckState = digitalRead(13);
    Serial.print(F("SPI PINS: MISO=")); Serial.print(misoState);
    Serial.print(F(" MOSI=")); Serial.print(mosiState);
    Serial.print(F(" SCK=")); Serial.println(sckState);
"""

if bad_insert in content:
    content = content.replace(bad_insert, "")
    with open(file_path, 'w') as f:
        f.write(content)
    print("Reverted.")
else:
    print("Not found.")
