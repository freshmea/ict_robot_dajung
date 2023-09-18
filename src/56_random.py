# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description:
import random


def main():
    print("random(): ", random.random())
    print("uniform(10, 20): ", random.uniform(10, 20))
    print("randrange(10): ", random.randrange(10))
    print("choice([1, 2, 3, 4, 5]): ", random.choice([1, 2, 3, 4, 5]))
    print("choice([1, 2, 3, 4, 5]): ", random.choices([1, 2, 3, 4, 5], k=6))
    li = [1, 2, 3, 4, 5]
    random.shuffle(li)
    print("shuffel([1, 2, 3, 4, 5]): ", li)
    print("sample([1, 2, 3, 4, 5]): ", random.sample([1, 2, 3, 4, 5], k=5))


if __name__ == "__main__":
    main()
