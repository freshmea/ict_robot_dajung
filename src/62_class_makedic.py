# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 학생 리스트를 선언하고 학생 정보를 딕셔너리로 만들어 출력하는 프로그램
def create_student(name, korean, math, english, science):
    return {
        "name": name,
        "korean": korean,
        "math": math,
        "english": english,
        "science": science,
    }


def main():
    students = [
        create_student("윤인성", 87, 98, 88, 95),
        create_student("연하진", 92, 98, 96, 98),
        create_student("구지연", 76, 96, 94, 90),
        create_student("나선주", 98, 92, 96, 92),
        create_student("윤아린", 95, 98, 98, 98),
        create_student("윤명월", 64, 88, 92, 92),
        create_student("최수길", 93, 23, 13, 42),
    ]
    print("이름\t총점\t평균")
    for student in students:
        score_sum = (
            student["korean"]
            + student["math"]
            + student["english"]
            + student["science"]
        )
        score_average = score_sum / 4
        print(f"{student['name']}\t{score_sum}\t{score_average}")


if __name__ == "__main__":
    main()
