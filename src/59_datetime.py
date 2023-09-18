# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: bs4를 사용하는 프로그램
import datetime
import time

# from bs4 import BeautifulSoup


def main():
    now = datetime.datetime.now()

    print("after time 만들기")
    after = now + datetime.timedelta(weeks=1, days=1, hours=1, minutes=1, seconds=1)
    after = after.replace(year=(after.year + 1))
    after = after.replace(month=(after.month + 1))
    print(now, after)

    print(time.time())
    print(time.asctime())
    print(time.clock())
    print(time.clock_gettime(1))
    time.sleep(5)
    print(time.clock_gettime(1))
    print(time.ctime())
    print(time.gmtime())


if __name__ == "__main__":
    main()
