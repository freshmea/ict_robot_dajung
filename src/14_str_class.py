# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: str 클래스 사용하기
def main():
    a = "choisugil"
    b = "CHOISUGIL"
    print(a.upper())
    print(b.lower())

    c = "    choi    "
    print(c + "end")
    print(c.strip() + "end")

    d = "this is a python class in ict"
    print(d.find("a"))
    print(d[d.find("a")])
    print(d.rfind("a"))

    idx = int()
    while d.find("a", idx) != -1:
        idx = d.find("a", idx)
        print(idx)
        idx += 1

    f = "TrainA10".isalnum()
    print(f)

    f = "Train_A10!".isalnum()
    print(f)

    f = "10".isdigit()
    print(f)

    f = "10a".isdigit()
    print(f)

    g = "this is python class".split()
    print(g[0])
    print(g[1])
    print(g[2])
    print(g[3])

    print("안녕" in "안녕하세요")
    print("요요" in "안녕하세요")


if __name__ == "__main__":
    main()
