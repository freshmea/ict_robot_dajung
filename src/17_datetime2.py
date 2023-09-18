# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: datetime 모듈을 사용하여 현재 시간을 출력하는 프로그램
import datetime


def main():
    now = datetime.datetime.now()

    if now.hour < 12:
        print(f"현재 시각은 {now.hour}로 오전입니다.!")
    else:
        print(f"현재 시각은 {now.hour}로 오후입니다.!")

    if now.month < 3:
        print(f"이번 달은 {now.month}로 겨울입니다.")
    elif now.month <= 5:
        print(f"이번 달은 {now.month}로 봄입니다.")
    elif now.month <= 8:
        print(f"이번 달은 {now.month}로 여름입니다.")
    elif now.month <= 11:
        print(f"이번 달은 {now.month}로 가을입니다.")
    else:
        print(f"이번 달은 {now.month}로 겨울입니다.")


if __name__ == "__main__":
    main()
