# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 소멸자를 사용하는 프로그램
class Test:
    def __init__(self, name) -> None:
        self.name = name
        print(f"{self.name} - 생성 되었습니다.")

    def __del__(self):
        print(f"{self.name} - 파괴 되었습니다.")


def main():
    a = Test("A")
    b = Test("B")
    c = Test("C")
    print(a, b, c)
    del c


if __name__ == "__main__":
    main()
