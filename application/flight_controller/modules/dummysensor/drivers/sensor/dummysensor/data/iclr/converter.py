import csv

telemetry = open("telemetry.csv", "r")

out = open("iclr.h", "w")

reader = csv.reader(telemetry)
out.write("#include <stdint.h>\n")
out.write("#define DATA_DELAY -240.0\n")
out.write("float imu_data[][7] = {\n")

div = 10000

imu_length = 0
for row in reader:
    if row[0] == "pn":
        continue
    line = "{" + str(float(row[38]) / div) + ","
    for value in row[20:26]:
        line += str(float(value) * 9.8) + ","
    line += "},\n"
    out.write(line)
    imu_length += 1
out.write("};\n")
out.write(f"uint32_t imu_length = {imu_length};\n\n")

telemetry.seek(0)
out.write("float baro_data[][2] = {\n")
baro_length = 0
for row in reader:
    if row[0] == "pn":
        continue
    line = "{" + str(float(row[38]) / div) + "," + str(float(row[30]) / 100) + "},\n"
    out.write(line)
    baro_length += 1
out.write("};\n\n")
out.write(f"uint32_t baro_length = {baro_length};\n\n")
