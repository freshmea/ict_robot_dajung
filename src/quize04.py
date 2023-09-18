# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 퀴즈 프로그램
import time


def main():
    question = [
        "13+ 24*2 는 무엇일까요?",
        "파이는 무엇일까요?",
        "print() 함수의 print 는 무엇일까요?",
        "print 함수의 키워드가 아닌것은?",
        "ROS의 풀 네임은?",
        "다음 중 튜플의 특징이 아닌것은?",
        "람다 함수의 스펠링은?",
        "가변 매개변수에 쓰이는 기호는?",
        "빨간색의 영어는?",
        "파이썬에서 bool 타입으로 옳다의 스펠링은?",
    ]
    question2 = [
        [0, 10, 61, 71],
        [4.23, 3.14, 5.13, 3.15],
        ["키워드", "식별자", "연산자", "자료"],
        ["end", "sep", "sup"],
        [
            "robust of superman",
            "robot of seoul",
            "robstar operator start",
            "robot operating system",
        ],
        [
            "인덱싱으로 원소를 불러 올 수 있다.",
            "슬라이싱으로 부분을 리턴할 수 있다.",
            "원소에 접근하여 변경할 수 있다.",
            "순서가 있어서 for문을 사용할 수 있다.",
        ],
        ["lamda", "ramdae", "rambda", "lambda"],
        ["?", "#", "&", "*"],
        ["led", "red", "rad", "raid"],
        ["throw", "true", "True", "TRUE"],
    ]
    correct = [3, 2, 2, 3, 4, 3, 4, 4, 2, 3]
    print(correct)
    answer = int()
    score = int()
    print("아주 쉬운 퀴즈 지금 부터 시작합니다!!")

    for i, qu in enumerate(question):
        print(f"{i+1}. 번 문제 {qu}")
        for j, qu2 in enumerate(question2[i]):
            print(f"  {j+1}) {qu2}")
        answer = int(input())
        if answer == correct[i]:
            score += 10
            print("정답입니다.")
        else:
            print("오답입니다.")
        time.sleep(1)

    print(f"퀴즈가 끝났습니다. 당신의 점수는 {score} 입니다.")


if __name__ == "__main__":
    main()
