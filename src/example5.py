# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 람다함수 활용하기
numbers = [1, 2, 3, 4, 5, 6]
print("::".join(map(lambda x: str(x), numbers)))

numbers = list(range(1, 10 + 1))

print("홀수만")
print(list(filter(lambda x: x % 2 == 1, numbers)))

print("3 이상 7 미만 추출")
print(list(filter(lambda x: 3 <= x < 7, numbers)))

print("# 제골해서 50 미만 추출하기")
print(list(filter(lambda x: x * x < 50, numbers)))
