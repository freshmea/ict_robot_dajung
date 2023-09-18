# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 파일을 읽고 쓰는 프로그램
import random


def main():
    hanguls = list("가나다라마바사아자차카파타하")
    with open("data/info.txt", "w") as f:
        for i in range(1000):
            name = (
                random.choice(hanguls) + random.choice(hanguls) + random.choice(hanguls)
            )
            weight = random.randrange(40, 100)
            height = random.randrange(140, 200)
            f.write(f"{name}, {weight}, {height}\n")


if __name__ == "__main__":
    main()
