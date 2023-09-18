# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: hello, python! 출력하기


def main():
    input_num = int(input("반복할 횟수를 입력하세요: "))
    for _ in range(input_num):
        print("Hello, python!")


if __name__ == "__main__":
    main()
