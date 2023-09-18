# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 예외처리를 사용하는 프로그램
import math


def main():
    # user_input = input('정수입력: ')
    # number_input = int()
    # try:
    #     number_input = int(user_input)
    # except:
    #     print('정수를 입력하지 않았습니다.')
    #     sys.exit()
    # print('원의 반지름: ', number_input)
    # print('원의 둘레: ', 2*math.pi* number_input)
    # print('원의 넓이: ', math.pi * number_input * number_input)

    user_input = input("정수입력: ")
    try:
        number_input = int(user_input)
        print("원의 반지름: ", number_input)
        print("원의 둘레: ", 2 * math.pi * number_input)
        print("원의 넓이: ", math.pi * number_input * number_input)
    except Exception as exception:
        print("정수를 입력하지 않았습니다.\n", exception)


if __name__ == "__main__":
    main()
