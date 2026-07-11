import sys
file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"
with open(file_path, 'r') as f:
    content = f.read()

insert_point = content.find('Serial.print(F("SD Status:   "));')
if insert_point != -1:
    content = content[:insert_point] + 'Serial.print(F("IMU Fault:   ")); Serial.println(imuFaultLocked ? F("YES") : F("NO"));\n    Serial.print(F("ESEKF Fault: ")); Serial.println(esekfFaultLocked ? F("YES") : F("NO"));\n    ' + content[insert_point:]
    with open(file_path, 'w') as f:
        f.write(content)
    print("Patched.")
else:
    print("Not found.")
