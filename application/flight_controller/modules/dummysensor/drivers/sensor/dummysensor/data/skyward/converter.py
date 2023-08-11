import csv

imu_raw = open("imu.csv", "r")
baro_raw = open("baro.csv", "r")

out = open("skyward.h", "w")

imu_reader = csv.reader(imu_raw)
baro_reader = csv.reader(baro_raw)
out.write("#include <stdint.h>\n")
out.write("#define DATA_DELAY 8.0\n")
out.write("float imu_data[][7] = {\n")

imu_length = 0
for row in imu_reader:
    if row[0] == "time":
        continue
    line = "{"
    for value in row:
        line += value + ","
    line += "},\n"
    out.write(line)
    imu_length += 1
out.write("};\n")
out.write(f"uint32_t imu_length = {imu_length};\n\n")

out.write("float baro_data[][2] = {\n")
baro_length = 0
for row in baro_reader:
    if row[0] == "time":
        continue
    line = "{" + row[0] + "," +  str(float(row[1]) / 1000) + "},\n"
    out.write(line)
    baro_length += 1
out.write("};\n\n")
out.write(f"uint32_t baro_length = {baro_length};\n\n")
