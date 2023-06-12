def print_n_times(value, n=2):
    for i in range(n):
        print(value)

def print_n_times(a, v, d, *value, n=2):
    for i in range(n):
        print(value)

def main():
    print_n_times(2, '안녕하세요')

if __name__ == '__main__':
    main()
