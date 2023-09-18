# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: list comprehension을 사용하여 1부터 100까지의 짝수를 출력하는 프로그램
def main():
    li = [i + 1 for i in range(100) if i % 2 == 0]
    print(li)


if __name__ == "__main__":
    main()
