# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 퀴즈 데이터를 pickle로 저장하는 프로그램
import pickle


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

    with open("data/question.pickle", "wb") as f:
        pickle.dump(question, f)
        pickle.dump(question2, f)
        pickle.dump(correct, f)


if __name__ == "__main__":
    main()
