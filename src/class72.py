class Person:
    def __init__(self) -> None:
        pass
    def greeting(self):
        print('안녕하세요')

class University:
    def __init__(self) -> None:
        pass
    def masage_credit(self):
        print('학점 관리')

class Undergraduate(Person, University):
    def __init__(self) -> None:
        super().__init__()
    def study(self):
        print('공부하기')

def main():
    james = Undergraduate()
    james.greeting()
    james.masage_credit()
    james.study()

if __name__ == '__main__':
    main()
