def print_n_times(n, *args, **keyargs):
    for i in range(n):
        print (args)
    
    print(keyargs)
    print(type(keyargs))
    for i in keyargs:
        print(keyargs[i])
    return

def main():
    print_n_times(2, 'a','b','c', d=1, e=2, f=3)
    
if __name__ == '__main__':
    main()