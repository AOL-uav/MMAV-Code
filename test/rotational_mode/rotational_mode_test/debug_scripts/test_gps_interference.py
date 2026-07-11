import sys
file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"
with open(file_path, 'r') as f:
    content = f.read()

# Comment out Serial1.begin
content = content.replace('Serial1.begin(GPS_BAUD);', '// Serial1.begin(GPS_BAUD);')

with open(file_path, 'w') as f:
    f.write(content)
print("Patched Serial1.begin")
