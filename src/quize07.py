import time
import pickle
import threading

score = int()
timeout = 4

def yes_no(answer, correct, time_spent):
    global score, timeout
    if answer == correct and time_spent < timeout:
        score += 10
        print('정답입니다.')
    else:
        print('오답입니다.')
    time.sleep(1)
    
def read_file():
    with open('data/question.pickle', 'rb') as f:
        question = pickle.load(f)
        question2 = pickle.load(f)
        correct = pickle.load(f)
    return question, question2, correct

def overtime():
    print('시간 초과되었습니다.')
    
def main():
    global score, timeout
    question, question2, correct = read_file()
    print('아주 쉬운 퀴즈 지금 부터 시작합니다!!')
    
    for i, qu in enumerate(question):
        print(f'{i+1}. 번 문제 {qu}')
        for j, qu2 in enumerate(question2[i]):
            print(f'  {j+1}) {qu2}')
        start_time = time.time()
        timer = threading.Timer(timeout, overtime)
        timer.start()
        answer = int(input())
        timer.cancel()
        time_spent = time.time() - start_time
        yes_no(answer, correct[i], time_spent)
    
    print(f'퀴즈가 끝났습니다. 당신의 점수는 {score} 입니다.')

if __name__ == '__main__':
    main()
