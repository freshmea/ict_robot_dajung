# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 문자열 인덱싱
def main():
    print("안녕하세요")
    print(type("안녕하세요"))
    print("안녕하세요"[0])
    print(type("안녕하세요"[0]))
    print("안녕하세요"[1])
    print("안녕하세요"[2])
    print("안녕하세요"[3])
    print("안녕하세요"[4])

    say_hellow = "안녕하세요"
    print(say_hellow)

    # 반복된 문장
    print(say_hellow[0])
    print(say_hellow[1])
    print(say_hellow[2])
    print(say_hellow[3])
    print(say_hellow[4])

    # 파이썬 스타일
    for i in say_hellow:
        print(i)

    say_hellow2 = say_hellow * 3
    print(say_hellow2)
    print(say_hellow2[5:10])
    print(say_hellow2[-3:])
    print(say_hellow2[5:10:2])
    print(say_hellow2[-1::-1])
    say_hellow3 = say_hellow2[-1::-1]
    print(say_hellow3)


if __name__ == "__main__":
    main()
