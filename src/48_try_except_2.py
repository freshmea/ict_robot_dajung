def main():
    li = ['52', '273', '32', '스파이', '103']
    
    list_number = []
    for i in li:
        try:
            float(i)
            list_number.append(i)
        except:
            pass
    print(list_number)
        
    # map 함수 안에서는 작동 안됨.
    # try:
    #     list_number2 = list(map(lambda x: float(x), li))
    # except:
    #     pass
    # print(list_number2)

if __name__ == '__main__':
    main()
