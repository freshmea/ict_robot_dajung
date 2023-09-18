# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 퀴즈 프로그램
import time


def main():
    question = []
    question2 = []
    correct = []
    with open("data/question6.txt", "r") as f:
        question = f.readline().split(", ")
        for i in range(10):
            question2.append(f.readline().split(", "))
        correct = list(map(lambda x: int(x), f.readline().split(", ")))
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
