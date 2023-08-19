import csv

imu_raw = open("imu.csv", "r")
baro_raw = open("baro.csv", "r")

out = open("piccard.h", "w")

imu_reader = csv.reader(imu_raw)
baro_reader = csv.reader(baro_raw)

out.write("#include <stdint.h>\n")
out.write("#define DATA_DELAY -3150\n")
out.write("float imu_data[][7] = {\n")

acc_scale = 32 / 2**15 * 9.82

imu_length = 0
for row in imu_reader:
    if row[1] == "ts":
        continue
    if row[2] != "IMU0":
        continue
    line = "{"
    line += str(float(row[1]) / 1000) + ","
    line += str(float(row[6]) * acc_scale) + ","
    line += str(float(row[7]) * acc_scale) + ","
    line += str(float(row[8]) * acc_scale) + ","
    line += row[3] + ","
    line += row[4] + ","
    line += row[5] + ","
    line += "},\n"
    out.write(line)
    imu_length += 1
out.write("};\n")
out.write(f"uint32_t imu_length = {imu_length};\n\n")

out.write("float baro_data[][2] = {\n")
baro_length = 0
for row in baro_reader:
    if row[1] == "ts":
        continue
    if row[2] != "BARO0":
        continue
    line = "{" + str(float(row[1]) / 1000) + "," +  str(float(row[4]) / 100) + "},\n"
    out.write(line)
    baro_length += 1
out.write("};\n\n")
out.write(f"uint32_t baro_length = {baro_length};\n\n")
