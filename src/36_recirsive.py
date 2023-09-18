# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 재귀함수를 사용하여 팩토리얼을 구하는 프로그램
def for_fac(n):
    output = 1
    for i in range(n):
        output *= i + 1
    return output


def rec_fac(n):
    if n == 0:
        return 1
    else:
        return n * rec_fac(n - 1)


def fibonacci(n):
    if n == 1:
        return 1
    elif n == 2:
        return 1
    else:
        return fibonacci(n - 1) + fibonacci(n - 2)


def main():
    print(for_fac(10))
    print(rec_fac(10))
    print(fibonacci(100))


if __name__ == "__main__":
    main()
