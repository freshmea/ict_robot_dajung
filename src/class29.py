def main():
    li = [1, 2, 5, 7, 9, 10, 13, 24, 64, 76]
    
    for i in li:
        print(i)
    
    for i in li:
        if i == 24:
            break
        print(i)
    print('---------------- 구분선')
    for i in li:
        if i == 24:
            continue
        print(i)

if __name__ == '__main__':
    main()
