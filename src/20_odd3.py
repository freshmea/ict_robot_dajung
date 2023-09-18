# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 짝수와 홀수를 구분하는 프로그램
def main():
    number = int(input("정수입력 :"))

    if number % 2 == 0:
        print("짝수 입니다.")
    if number % 2 == 1:
        print("홀수 입니다.")


if __name__ == "__main__":
    main()
