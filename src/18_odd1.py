# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 짝수와 홀수를 구분하는 프로그램
def main():
    number = input("정수입력 :")
    last_character = number[-1]
    last_number = int(last_character)

    if (
        last_number == 0
        or last_number == 2
        or last_number == 4
        or last_number == 6
        or last_number == 8
    ):
        print("짝수 입니다.")

    if (
        last_number == 1
        or last_number == 3
        or last_number == 5
        or last_number == 7
        or last_number == 9
    ):
        print("홀수 입니다.")


if __name__ == "__main__":
    main()
