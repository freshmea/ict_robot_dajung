# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 학생의 성적을 출력하는 프로그램
def main():
    students = [
        {"name": "윤인성", "korean": 87, "math": 98, "english": 88, "science": 95},
        {"name": "연하진", "korean": 92, "math": 98, "english": 96, "science": 98},
        {"name": "구지연", "korean": 76, "math": 96, "english": 94, "science": 90},
        {"name": "나선주", "korean": 98, "math": 92, "english": 96, "science": 92},
        {"name": "윤아린", "korean": 95, "math": 98, "english": 98, "science": 98},
        {"name": "윤명월", "korean": 64, "math": 88, "english": 92, "science": 92},
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
