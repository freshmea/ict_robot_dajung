# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 퀴즈 프로그램
import time


def main():
    question = [
        """
1. 번 문제 13+ 24*2 는 무엇일까요?
  1) 0
  2) 10
  3) 61
  4) 71""",
        """
2. 번 문제 파이는 무엇일까요?
  1) 4.23
  2) 3.14
  3) 5.13
  4) 3.15""",
        """
3. 번 문제 print() 함수의 print 는 무엇일까요?
  1) 키워드
  2) 식별자
  3) 연산자
  4) 자료""",
        """
4. 번 문제 print 함수의 키워드가 아닌것은?
  1) end
  2) sep
  3) sup""",
        """
5. 번 문제 ROS의 풀 네임은?
  1) robust of superman
  2) robot of seoul
  3) robstar operator start
  4) robot operating system""",
        """
6. 번 문제 다음 중 튜플의 특징이 아닌것은?
  1) 인덱싱으로 원소를 불러 올 수 있다.
  2) 슬라이싱으로 부분을 리턴할 수 있다.
  3) 원소에 접근하여 변경할 수 있다.
  4) 순서가 있어서 for문을 사용할 수 있다.""",
        """
7. 번 문제 람다 함수의 스펠링은?
  1) lamda
  2) ramdae
  3) rambda
  4) lambda""",
        """
8. 번 문제 가변 매개변수에 쓰이는 기호는?
  1) ?
  2) #
  3) &
  4) *""",
        """
9. 번 문제 빨간색의 영어는?
  1) led
  2) red
  3) rad
  4) raid""",
        """
10. 번 문제 파이썬에서 bool 타입으로 옳다의 스펠링은?
  1) throw
  2) true
  3) True
  4) TRUE""",
    ]
    correct = [3, 2, 2, 3, 4, 3, 4, 4, 2, 3]
    answer = int()
    score = int()
    print("아주 쉬운 퀴즈 지금 부터 시작합니다!!")

    for i, qu in enumerate(question):
        print(qu)
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
