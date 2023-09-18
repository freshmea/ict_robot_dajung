# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 함수를 사용하여 3번 출력하는 프로그램


def print_3_time():
    print("안녕하세요")
    print("안녕하세요")
    print("안녕하세요")


def print_n_time(value, n):
    for i in range(n):
        print(value)


def main():
    # print_3_time()
    # print_3_time()
    # print_3_time()
    print_n_time("안녕하세요", 10)
    # print_n_time('안녕하세요', '세번')


if __name__ == "__main__":
    main()
