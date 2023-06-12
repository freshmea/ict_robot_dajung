import time

def main():
    print(time.asctime())
    print(time.time())
    print(time.clock_gettime_ns(1))

    ctime = time.time() + 5
    cnt = int()
    while time.time() < ctime:
        cnt += 1
        
    print(f'이 컴퓨터는 5초 동안 {cnt} 카운트가 가능하다.')

if __name__ == '__main__':
    main()
