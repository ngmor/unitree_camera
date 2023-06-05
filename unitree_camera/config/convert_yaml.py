
bases = ['head_front', 'body_left', 'body_right']
for base in bases:
  with open(base+'.yaml', 'w') as f1:
    f1.write('/**:\n')
    f1.write('  ros__parameters:\n')
    for side in ['left', 'right']:
      with open(base+'_'+side+'.yaml', 'r') as f2:
        for line in f2.readlines():
          # print(line.strip('\n'))
          lastchar = line.strip('\n')[-1]
          id = line.split()[0].strip(':')
          id = id.strip()
          # print(f"last char: {lastchar}")
          if id != 'rows' and id != 'cols':
            if id == 'data':
              line = ' '+' '.join(line.split()[1:])
              if lastchar == ']':
                line += '\n'
            if '_' in id:
              line = '    '+side+'_'+line
            if lastchar == ',' or id == 'camera_matrix' or id == 'projection_matrix' or id == 'rectification_matrix' or id == 'distortion_coefficients':
              f1.write(line.strip('\n'))
            else:
              f1.write(line)