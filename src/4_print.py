# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 출력하기


def main():
    # one variable
    print(123456)
    print("choi su gil")
    print(3.141592)

    # multiple variable
    print("this is", "python", "class!")
    print(10, 20, 30, 40, 50)
    print()

    print("this", "is", "python", "class!", sep=" ", end="")
    print("this", "is", "python", "class!", sep=" ", end="\n")


if __name__ == "__main__":
    main()
