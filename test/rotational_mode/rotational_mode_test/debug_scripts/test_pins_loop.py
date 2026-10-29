import sys
file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"
with open(file_path, 'r') as f:
    content = f.read()

insert = """
    int misoState = digitalRead(12);
    int mosiState = digitalRead(11);
    int sckState = digitalRead(13);
    Serial.print(F("SPI PINS: MISO=")); Serial.print(misoState);
    Serial.print(F(" MOSI=")); Serial.print(mosiState);
    Serial.print(F(" SCK=")); Serial.println(sckState);
"""

pos = content.find('Serial.print(F("SD Status:   "));')
if pos != -1:
    content = content[:pos] + insert + content[pos:]
    with open(file_path, 'w') as f:
        f.write(content)
    print("Patched.")
else:
    print("Not found.")
