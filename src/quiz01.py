import time

def main():
    answer = int()
    score = int()
    print('아주 쉬운 퀴즈 지금 부터 시작합니다!!')
    
    # 1번 문제
    print('''
1. 번 문제 13+ 24*2 는 무엇일까요?
  1) 0
  2) 10
  3) 61
  4) 71''')
    answer = int(input())
    if answer == 3:
        print('정답입니다.')
        score += 10
    else:
        print('오답입니다.')
    time.sleep(1)
    
    # 2번 문제
    print('''
2. 번 문제 파이는 무엇일까요?
  1) 4.23
  2) 3.14
  3) 5.13
  4) 3.15''')
    answer = int(input())
    if answer == 2:
        print('정답입니다.')
        score += 10
    else:
        print('오답입니다.')
    time.sleep(1)
    
    print(f'퀴즈가 끝났습니다. 당신의 점수는 {score} 입니다.')

if __name__ == '__main__':
    main()
