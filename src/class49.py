import math, sys

def main():
    user_input = input('정수입력: ')
    try:
        number_input = int(user_input)
    except:
        print('정수를 입력하지 않았습니다.')
        # sys.exit()
    else:
        print('원의 반지름: ', number_input)
        print('원의 둘레: ', 2*math.pi* number_input)
        print('원의 넓이: ', math.pi * number_input * number_input)

if __name__ == '__main__':
    main()
