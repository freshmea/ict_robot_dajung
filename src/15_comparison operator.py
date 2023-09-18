# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 비교 연산자 사용하기
def main():
    # 비교 연산자 연습
    print(10 == 100)  # False
    print(10 != 100)  # True
    print(10 < 100)  # True
    print(10 > 100)  # False
    print(10 <= 100)  # True
    print(100 <= 100)  # True
    print(100 < 100)  # False

    print("\n\n\n-------------------------------구분선")
    # 논리 연산자
    print(not True)  # False
    print(not (10 < 100))  # False

    # 무족건 실행
    if True:
        print("True 입니다...!")
    # 무족건 실행 안됨
    if False:
        print("False 입니다...!")

    a = int(input("100보다 큰 숫자를 넣으세요:"))

    # if a >100:
    #     print('입력한 숫자가 100보다 큽니다.')
    # if a <= 100:
    #     print('입력한 숫자가 100보다 작거나 같습니다.')

    # if a >100:
    #     print('입력한 숫자가 100보다 큽니다.')
    # else:
    #     print('입력한 숫자가 100보다 작거나 같습니다.')

    if a > 200:
        print("입력한 숫자가 200보다 큽니다.")
    elif a > 100:
        print("입력한 숫자가 100보다 크고 200보다 작습니다.")
    else:
        print("입력한 숫자가 100보다 작거나 같습니다.")


if __name__ == "__main__":
    main()
