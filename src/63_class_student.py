# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 클래스를 사용하는 프로그램
class Student:
    def __init__(self, name, korean, math, english, science):
        self.name = name
        self.korean = korean
        self.math = math
        self.english = english
        self.science = science


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
        score_sum = student.korean + student.math + student.english + student.science
        score_average = score_sum / 4
        print(f"{student.name}\t{score_sum}\t{score_average}")


if __name__ == "__main__":
    main()
