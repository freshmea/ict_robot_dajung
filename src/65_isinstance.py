# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description:  isinstace() 함수를 사용하는 프로그램
class Student:
    def __init__(self) -> None:
        pass

    def study(self):
        print("공부를 합니다.")


class Teacher:
    def __init__(self) -> None:
        pass

    def teach(self):
        print("학생을 가리킵니다.")


def main():
    student = Student()
    print(isinstance(student, Student))
    print(isinstance(student, int))

    # 파이썬의 모든 클래스는 object 를 상속 받아서 생성된다.
    print(isinstance(student, object))
    print(isinstance(1, object))
    print(isinstance([1, 2, 3], object))

    classroom = [Student(), Student(), Teacher(), Student(), Student()]

    for person in classroom:
        if isinstance(person, Student):
            person.study()
        if isinstance(person, Teacher):
            person.teach()


if __name__ == "__main__":
    main()
