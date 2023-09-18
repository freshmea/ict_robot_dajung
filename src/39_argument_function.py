# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 함수를 인자로 전달하는 프로그램
def call_10_times(func):
    for i in range(10):
        func()


def print_hello():
    print("안녕하세요.")


def main():
    call_10_times(print_hello)


if __name__ == "__main__":
    main()
