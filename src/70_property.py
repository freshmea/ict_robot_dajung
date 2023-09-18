# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 프라이빗 변수를 사용하는 프로그램
import math


class Circle:
    def __init__(self, radius) -> None:
        self.__radius = radius

    @property
    def radius(self):
        return self.__radius

    @radius.setter
    def radius(self, value):
        print("setter")
        if isinstance(value, int) and value > 0:
            self.__radius = value
        else:
            print("양의 정수만 넣으시오.")

    @radius.getter
    def radius(self):
        print("getter")
        return self.__radius

    def get_circumference(self):
        return 2 * math.pi * self.__radius

    def get_area(self):
        return math.pi * (self.__radius**2)


def main():
    circle = Circle(10)
    print(" 원의 둘레와 넓이를 구합니다.")
    print("원의 둘레: ", circle.get_circumference())
    print("원의 넓이: ", circle.get_area())

    print("__radius 에 접근 합니다.")
    # print(circle.__radius) # 프라이빗이라서 안됨.
    circle.radius = 20
    print(circle.radius)


if __name__ == "__main__":
    main()
