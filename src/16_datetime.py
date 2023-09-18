# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: datetime 사용하기
import datetime


def main():
    ptime = datetime.datetime.now()
    print(ptime)

    print(int(str(ptime)[5:7]))

    print(ptime.month)
    print(type(ptime.month))
    # print(ptime.year, '년')
    # print(ptime.month, '월')
    # print(ptime.day, '일')
    # print(ptime.hour, '시')
    # print(ptime.minute, '분')
    # print(ptime.second, '초')

    print(f"{ptime.year}년 {ptime.month}월 {ptime.day}일 {ptime.hour}시")
    print(f"{ptime.minute}분 {ptime.second}초")


if __name__ == "__main__":
    main()
