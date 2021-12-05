import os

def read_directory(directory_name):
    i = 634
    for filename in os.listdir(r"./"+directory_name):
        os.rename('./'+directory_name+'/'+filename, './'+directory_name+'/'+'frame0'+str(i)+'.jpg')
        print("{0}.{1}\n".format(i, filename))
        i = i + 1

if __name__ == '__main__':
    read_directory('image_9')
