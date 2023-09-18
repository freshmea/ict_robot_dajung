# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: format 사용하기
def main():
    string = "abc"
    string2 = "this is format test: {}".format(10)
    print(string)
    print(string2)

    string3 = "this is format test: {2}, {1}, {0}".format(10, 20, 30)
    print(string3)

    string3 = "this is format test: {2:d}, {1:5d}, {0:05d}".format(10, 20, 30)
    print(string3)

    string4 = "this is format test: {2:+d}, {1:+5d}, {0:+05d}".format(-10, -20, -30)
    print(string4)

    string5 = "this is format test: {2:+.2f}, {1:+5.2f}, {0:+05.2f}".format(
        10.126, -20.42, -30
    )
    print(string5)

    string6 = 10.126
    print(f"this is fstring test: {string6:+.2f}")


if __name__ == "__main__":
    main()
