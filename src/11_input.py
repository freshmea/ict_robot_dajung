# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: input 사용하기
def main():
    input_var = input("숫자를 입력하세요:")
    print(type(input_var), input_var)
    # print(input_var + 100)
    print(int(input_var) + 100)  # 캐스팅 str -> int


if __name__ == "__main__":
    main()
