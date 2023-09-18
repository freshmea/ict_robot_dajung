# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: time 모듈을 사용하여 현재 시간을 출력하는 프로그램
import time


def main():
    print(time.asctime())
    print(time.time())
    print(time.clock_gettime_ns(1))

    ctime = time.time() + 5
    cnt = int()
    while time.time() < ctime:
        cnt += 1

    print(f"이 컴퓨터는 5초 동안 {cnt} 카운트가 가능하다.")


if __name__ == "__main__":
    main()
