# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 리스트 메소드
def main():
    list_a = [1, 2, 3]
    list_b = [4, 5, 6]
    print(list_a + list_b)
    print(list_a, list_b)

    list_a.extend(list_b)
    print(list_a)
    list_a = list_a * 4
    print(list_a)
    print(len(list_a))
    list_b.append(7)
    list_b.append(8)
    print(list_b)
    list_b.insert(0, 1)
    print(list_b)
    list_b.insert(1, 2)
    print(list_b)
    list_b.insert(2, 3)
    print(list_b)


if __name__ == "__main__":
    main()
