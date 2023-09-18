# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 예외처리를 사용하는 프로그램
import math


def main():
    a = [1, 2, 3]
    user_input = input("정수입력: ")
    try:
        number_input = int(user_input)
        if number_input == 100:
            a[3]
    except ValueError as e:
        print(e)
        print("ValueError정수를 입력하지 않았습니다.")
        # sys.exit()
        print("end of while")
    except IndexError as e:
        print(e)
        print("IndexError다른 에러입니다.")
    else:
        print("원의 반지름: ", number_input)
        print("원의 둘레: ", 2 * math.pi * number_input)
        print("원의 넓이: ", math.pi * number_input * number_input)
    finally:
        print("----- 프로그램이 끝났습니다. -----")


if __name__ == "__main__":
    main()
