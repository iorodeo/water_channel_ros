"""
Test code -- creates and example position array for the  positon array server to load
"""

filename = 'parray_test.txt'
N = 1000

with open(filename,'w') as f:
    f.write('relative\n')
    for i in range(0,N):
        value = float(i)
        f.write('%f\n'%(value,))

