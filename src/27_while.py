# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: while 반복문을 사용하여 1부터 10까지 출력하는 프로그램
def main():
    # while True: # 무한 반복
    #     print('.', end='')
    i = int()
    while i < 10:
        print(f"{i}번째 실행 중...")
        i += 1

    li = [1, 2, 3, 4, 2, 4, 2]

    while 2 in li:
        li.remove(2)
    print(li)

    string = "this is a python class in a seojong city!"

    while "a" in string:
        print(string.find("a"))
        print(string)
        string = string.replace("a", "", 1)
    print(string)


if __name__ == "__main__":
    main()
