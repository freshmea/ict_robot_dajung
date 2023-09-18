# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: dictionary 자료형을 사용하는 프로그램
def main():
    dict_a = dict()
    # dict_a = {}
    print(type(dict_a))
    dict_a = {"a": 100, "b": 200, "c": 40}

    print(dict_a["a"])  # key 에러가 발생.
    dict_a["d"] = 400
    print(dict_a.get("d"))  # key 에러가 발생하지 않음. None

    print(dict_a)
    # print(dict_a['e'])
    del dict_a["b"]
    print(dict_a)
    print(dict_a.pop("c"))
    print(dict_a)

    dict_a = {"a": 100, "b": 200, "c": 40, "d": 400}

    for i in dict_a:
        print(i, dict_a[i])

    # key, value 같이 출력하기.
    for key, value in dict_a.items():
        print(key, value)


if __name__ == "__main__":
    main()
