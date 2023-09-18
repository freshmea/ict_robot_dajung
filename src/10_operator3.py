# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 연산자 출력하기
def main():
    pi = 3.141592
    r = 10

    print("원주율 = ", pi)
    print("반지름 = ", r)
    print("원의 둘레 =", 2 * pi * r)
    print("원의 넓이 =", pi * r * r)

    number = 100
    number += 10
    number += 20
    number += 30
    print("number:", number)

    string = "안녕하세요"
    string += "!"
    string += "!"
    print("string:", string)


if __name__ == "__main__":
    main()
