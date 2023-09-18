# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description:  __str__ 메서드를 사용하는 프로그램
class Student:
    def __init__(self, name, korean, math, english, science):
        self.name = name
        self.korean = korean
        self.math = math
        self.english = english
        self.science = science

    def get_sum(self):
        return self.korean + self.math + self.english + self.science

    def get_average(self):
        return self.get_sum() / 4

    def __str__(self):
        return f"{self.name}\t{self.get_sum()}\t{self.get_average()}"

    def __gt__(self, value):
        if isinstance(value, Student):
            return self.get_sum() > value.get_sum()
        if isinstance(value, int):
            return self.get_sum() > value

    def __eq__(self, value):
        if isinstance(value, Student):
            return self.get_sum() == value.get_sum()
        if isinstance(value, int):
            return self.get_sum() == value

    def __ne__(self, value):
        if isinstance(value, Student):
            return self.get_sum() != value.get_sum()
        if isinstance(value, int):
            return self.get_sum() != value

    def __ge__(self, value):
        if isinstance(value, Student):
            return self.get_sum() >= value.get_sum()
        if isinstance(value, int):
            return self.get_sum() >= value

    def __lt__(self, value):
        if isinstance(value, Student):
            return self.get_sum() < value.get_sum()
        if isinstance(value, int):
            return self.get_sum() < value

    def __le__(self, value):
        if isinstance(value, Student):
            return self.get_sum() <= value.get_sum()
        if isinstance(value, int):
            return self.get_sum() <= value


def main():
    students = [
        Student("윤인성", 87, 98, 88, 95),
        Student("연하진", 92, 98, 96, 98),
        Student("구지연", 76, 96, 94, 90),
        Student("나선주", 98, 92, 96, 92),
        Student("윤아린", 95, 98, 98, 98),
        Student("윤명월", 64, 88, 92, 92),
        Student("최수길", 93, 23, 13, 42),
    ]
    print("이름\t총점\t평균")
    for student in students:
        print(student)

    student_a = Student("윤인성", 87, 98, 88, 95)
    student_b = Student("연하진", 92, 98, 96, 82)

    print("student_a == student_b = ", student_a == student_b)
    print("student_a != student_b = ", student_a != student_b)
    print("student_a > student_b = ", student_a > student_b)
    print("student_a >= student_b = ", student_a >= student_b)
    print("student_a < student_b = ", student_a < student_b)
    print("student_a <= student_b = ", student_a <= student_b)


if __name__ == "__main__":
    main()
