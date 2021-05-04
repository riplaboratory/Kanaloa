import os, re
 
# Create a path to the directory containing your labeled images
# ex. path = '/Users/name/labelimg/labeled'

path = '/Users/name/labelimg/labeled'
directory = os.listdir(path)
os.chdir(path)
print(directory)

# regex for all lines but the first /(?<=\n)3/ where 3 can be replaced with 1 or 2
# regex for first line only /^3/ where 3 can be replaced with 1 or 2

for file in directory:
    open_file = open(file,'r')
    read_file = open_file.read()
    regex = re.compile('(?<=\n)3')
    # regex = re.compile('^3')
    read_file = regex.sub('2', read_file)
    write_file = open(file,'w')
    write_file.write(read_file)
