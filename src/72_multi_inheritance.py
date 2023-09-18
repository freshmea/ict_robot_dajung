# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 다중 상속을 사용하는 프로그램
class Person:
    def __init__(self, b) -> None:
        self.b = b

    def greeting(self):
        print("안녕하세요")


class University:
    def __init__(self, a) -> None:
        self.a = a

    def masage_credit(self):
        print("학점 관리")


class Undergraduate(Person, University):
    def __init__(self) -> None:
        Person.__init__(self, 1)
        University.__init__(self, 1)

    def study(self):
        print("공부하기")


def main():
    james = Undergraduate()
    james.greeting()
    james.masage_credit()
    james.study()


if __name__ == "__main__":
    main()
