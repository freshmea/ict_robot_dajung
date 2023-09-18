# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 튜플을 사용하여 두변수의 값을 교환하는 프로그램
def main():
    # 교환 하려면 잘 못 된 예.
    a = 10
    b = 20
    a = b  # a= 20, b =20
    b = a  # b= 20, a = 20
    print(f"a : {a}, b {b}")

    # temp 라는 변수를 씀. C/C++
    a = 10
    b = 20
    temp = a  # a=10, b=20, temp=10
    a = b  # a=20, b=20, temp=10
    b = temp  # a=20, b=10, temp=10
    print(f"a : {a}, b {b}")
    # python
    a = 10
    b = 20
    a, b = b, a  # (a, b) = (b, a)
    print(f"a : {a}, b {b}")


if __name__ == "__main__":
    main()
