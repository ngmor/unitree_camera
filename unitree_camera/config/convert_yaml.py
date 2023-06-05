
base = 'head_front'


with open(base+'.yaml', 'w') as f1:
  for side in ['left']:
    print(f"\n\nside={side}")
    f1.write(side+'\n')
    with open(base+'_'+side+'.yaml', 'r') as f2:
      for line in f2.readlines():
        # print(line.strip('\n'))
        lastchar = line.strip('\n')[-1]
        id = line.split()[0].strip(':')
        id = id.strip()
        print(f"ID: {id}")
        # print(f"last char: {lastchar}")
        if id != 'rows' and id != 'cols':
          if id == 'data':
            line = ' '+' '.join(line.split()[1:])
            if lastchar == ']':
              line += '\n'
          if lastchar == ',' or id == 'camera_matrix' or id == 'projection_matrix' or id == 'rectification_matrix' or id == 'distortion_coefficients':
            f1.write(line.strip('\n'))
          else:
            f1.write(line)
        print()