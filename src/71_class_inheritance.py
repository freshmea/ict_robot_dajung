# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 클래스 상속을 사용하는 프로그램
class Parent:
    def __init__(self, value) -> None:
        self.value = "테스트"
        self.value2 = value
        print("Parent 클래스의 __init__ 메소드가 호출 되었습니다.")

    def test(self):
        print("Parent 클래스의 test 메소드 입니다.")


class Child(Parent):
    def __init__(self) -> None:
        # super().__init__('자식에서 넘어온 값')
        Parent.__init__(self, "자식에서 넘어온 값")
        print("Child 클래스의 __init__ 메소드가 호출 되었습니다.")

    # 파이선에서는 오버라이딩만 된다.
    # def test(self):
    #     print('Child 클래스의 test 메소드 입니다.')


def main():
    child = Child()
    child.test()
    print(child.value2)


if __name__ == "__main__":
    main()
