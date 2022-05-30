

# read the txt file on setting dir
def read_txt_file(file_path):
    name = []
    value = []
    dict_data = {}
    str_data = ''
    try:
        f = open(file_path)
        temp_data = f.readlines()
        for temp_dat in temp_data:
            for dat in temp_dat:
                if dat == ':':
                    name.append(str_data)
                    str_data = ''
                    continue
                elif dat == '\n':
                    value.append(str_data)
                    str_data = ''
                    continue
                else:
                    str_data += dat
        dict_data = dict_data.fromkeys(name)
        for n in range(len(name)):
            if n == 3 or n == 4:  # start, end frame
                dict_data[name[n]] = int(value[n])
            elif n == 5 or n == 6:  # init pos, rot
                m = value[n].split(',')
                dict_data[name[n]] = [float(m[0]), float(m[1]), float(m[2])]
            elif n == 7:   # frame duration
                dict_data[name[n]] = float(value[n])
            else:
                dict_data[name[n]] = value[n]
        return dict_data
    except():
        print("Can not open file!")
