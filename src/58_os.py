import os

def main():
    print(os.name)
    print(os.getcwd())
    print(os.listdir())
    
    os.mkdir('source')
    os.rmdir('source')
    with open('original.txt', 'w') as f:
        f.write('hello')
    os.rename('original.txt', 'new.txt')
    os.remove('new.txt')
    
    os.system('ls')
    os.system('ls')
    dir = '/home/aa/ict_robot_dajung/src'
    os.chdir(dir)
    os.mkdir('source')
    os.rmdir('source')
    print(os.getcwd())
    os.system('ls')
    
if __name__ == '__main__':
    main()
