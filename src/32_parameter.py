
def print_n_times(n, *value):
    print(type(value))
    for i in range(n):
        for v in value:
            print(v)
        print()

def main():
    print_n_times(3, 'abc', 'def', 'ghi', 'jk')
    tu = tuple()
    li = []
    dic = {}
    tu = 1, 
    print(type(tu))

if __name__ == '__main__':
    main()
