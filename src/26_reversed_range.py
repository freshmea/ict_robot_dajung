# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: range() 함수를 사용하여 5, 10, 15, 20을 출력하는 프로그램
def main():
    for i in range(5, 20, 5):
        print(i)

    for i in range(6, -1, -1):
        print(i)

    for i in reversed(range(7)):
        print(i)


if __name__ == "__main__":
    main()
