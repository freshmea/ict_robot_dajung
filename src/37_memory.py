# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 피보나치 수열을 구하는 프로그램
counter = 0

dictionary = {1: 1, 2: 1}


def fibonacci(n):
    global counter
    print(f"fibonnacci({n}을 구합니다.)")
    counter += 1
    if n in dictionary:
        return dictionary[n]
    else:
        output = fibonacci(n - 1) + fibonacci(n - 2)
        dictionary[n] = output
        return output


def fibonacci2(n):
    global counter
    print(f"fibonnacci({n}을 구합니다.)")
    counter += 1
    if n in dictionary:
        return dictionary[n]
    output = fibonacci(n - 1) + fibonacci(n - 2)
    dictionary[n] = output
    return output


def main():
    print(fibonacci(100))
    print("---")
    print(f"fibonacci(100) 계산에 활용된 덧셈 횟수는 {counter} 입니다.")


if __name__ == "__main__":
    main()
