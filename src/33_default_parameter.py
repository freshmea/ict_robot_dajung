# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 기본 매개변수를 사용하여 함수를 호출하는 프로그램
def print_n_times1(value, n=2):  # n: 기본 매개변수 O
    for i in range(n):
        print(value)


def print_n_times2(a, v, d, *value, n=2):  # n: 기본 매개변수 X, 키워드 매개변수 O
    print(a, v, d)
    for i in range(n):
        print(value)


def main():
    print_n_times1("안녕하세요", 2)
    print_n_times2(1, 2, 3, 4, 5, 6, n=2)


if __name__ == "__main__":
    main()
