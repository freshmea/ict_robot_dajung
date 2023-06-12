import time

def print_3_time():
    print('안녕하세요')
    print('안녕하세요')
    print('안녕하세요')
    
def print_n_time(value , n):
    for i in range(n):
        print(value)

def main():
    # print_3_time()
    # print_3_time()
    # print_3_time()
    print_n_time('안녕하세요', 10)
    # print_n_time('안녕하세요', '세번')

if __name__ == '__main__':
    main()
